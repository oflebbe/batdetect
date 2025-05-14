/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ivan Belokobylskiy
 * Copyright (c) 2023 Olaf Flebbe
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <hardware/spi.h>
#include <math.h>
#include <pico/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "st7789.h"
#include "st7789_parallel.pio.h"

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0
#define ST7789_135x240_XSTART 52
#define ST7789_135x240_YSTART 40

// color modes
#define COLOR_MODE_65K 0x50
#define COLOR_MODE_262K 0x60
#define COLOR_MODE_12BIT 0x03
#define COLOR_MODE_16BIT 0x05
#define COLOR_MODE_18BIT 0x06
#define COLOR_MODE_16M 0x07

// commands
#define ST7789_NOP 0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID 0x04
#define ST7789_RDDST 0x09

#define ST7789_SLPIN 0x10
#define ST7789_SLPOUT 0x11
#define ST7789_PTLON 0x12
#define ST7789_NORON 0x13

#define ST7789_INVOFF 0x20
#define ST7789_INVON 0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON 0x29
#define ST7789_CASET 0x2A
#define ST7789_RASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_RAMRD 0x2E

#define ST7789_PTLAR 0x30
#define ST7789_COLMOD 0x3A
#define ST7789_MADCTL 0x36

// shamelessly copied from piromoni
#define ST7789_TEON 0x35
#define ST7789_PORCTRL 0xB2
#define ST7789_LCMCTRL 0xC0
#define ST7789_VDVVRHEN 0xC2
#define ST7789_VRHS 0xC3
#define ST7789_VDVS 0xC4
#define ST7789_PWCTRL1 0xD0
#define ST7789_FRCTRL2 0xC6
#define ST7789_GCTRL 0xB7
#define ST7789_VCOMS 0xBB
#define ST7789_GMCTRP1 0xE0
#define ST7789_GMCTRN1 0xE1

#define ST7789_MADCTL_MY 0x80 // Page Address Order
#define ST7789_MADCTL_MX 0x40 // Column Address Order
#define ST7789_MADCTL_MV 0x20 // Page/Column Order
#define ST7789_MADCTL_ML 0x10 // Line Address Order
#define ST7789_MADCTL_MH 0x04 // Display Data Latch Order
#define ST7789_MADCTL_RGB 0x00
#define ST7789_MADCTL_BGR 0x08

#define ST7789_RDID1 0xDA
#define ST7789_RDID2 0xDB
#define ST7789_RDID3 0xDC
#define ST7789_RDID4 0xDD

typedef struct _ST7789_t
{
  uint width;
  uint height;
  uint reset;
  uint xstart;
  uint ystart;
  uint dc;
  uint cs;
  uint backlight;
  uint dma_channel;
  bool is_spi; // is_spi == true -> spi_obj valid , otherwise parallel_pio and parallel_sm
  union
  {
    spi_inst_t *spi_obj;
    struct
    {
      PIO parallel_pio;
      uint parallel_sm;
    };
  };
} ST7789_t;

extern const uint8_t font[];

static void write_blocking_parallel(ST7789_t *self, unsigned int len, const uint8_t src[len])
{
  for (int i = 0; i < len; i++)
  {
    uint32_t p = src[i] << 24;
    pio_sm_put_blocking(self->parallel_pio, self->parallel_sm, p);
    asm("nop;");
  }
}

static void write_non_blocking_dma(ST7789_t *self, unsigned int len, const uint8_t src[len])
{
  dma_channel_set_trans_count(self->dma_channel, len, false);
  dma_channel_set_read_addr(self->dma_channel, src, true);
}

static void write_blocking(ST7789_t *self, unsigned int len, const uint8_t src[len])
{
  if (self->is_spi)
  {
    spi_write_blocking(self->spi_obj, src, len);
  }
  else
  {
    write_blocking_parallel(self, len, src);
  }
}

static void write_cmd_repeat_rest(ST7789_t *self, uint8_t cmd,
                                  size_t len, const uint8_t data[len], int repeat,
                                  unsigned int rest)
{
  // Make sure previous transfer has ended
  ST7789_flush(self);
  gpio_put(self->cs, 0);
  if (cmd)
  {
    gpio_put(self->dc, 0);
    write_blocking(self, 1, &cmd);
  }
  if (len > 0)
  {
    gpio_put(self->dc, 1);
    while (repeat-- > 0)
    {
      write_non_blocking_dma(self, len, data);
      if (repeat > 0 || rest > 0)
      {
        // flush only when we are going to send a next block
        ST7789_flush(self);
      }
    }
    if (rest > 0)
    {
      write_non_blocking_dma(self, rest, data);
    }
  }
  // Why bothering with CS (would need an interrupt handler when DMA is
  // finished) gpio_put(self->cs, 1);
}

static void write_cmd(ST7789_t *self, uint8_t cmd, const uint8_t *data,
                      size_t len)
{
  write_cmd_repeat_rest(self, cmd, len, data, 1, 0);
}

static void set_window(ST7789_t *self, uint16_t x0, uint16_t y0, uint16_t x1,
                       uint16_t y1)
{

  const uint8_t bufx[4] = {(x0 + self->xstart) >> 8, (x0 + self->xstart) & 0xFF,
                           (x1 + self->xstart) >> 8,
                           (x1 + self->xstart) & 0xFF};
  const uint8_t bufy[4] = {(y0 + self->ystart) >> 8, (y0 + self->ystart) & 0xFF,
                           (y1 + self->ystart) >> 8,
                           (y1 + self->ystart) & 0xFF};
  write_cmd(self, ST7789_CASET, bufx, 4);
  write_cmd(self, ST7789_RASET, bufy, 4);
}

#define BUFFER_PIXEL_NUM 32
static uint8_t buffer[BUFFER_PIXEL_NUM * 2];

static void fill_color_buffer(ST7789_t *self, uint16_t color, unsigned int length)
{
  uint8_t hi = color >> 8, lo = color;
  const int chunks = length / BUFFER_PIXEL_NUM;
  const unsigned int rest = length % BUFFER_PIXEL_NUM;
  const unsigned int buffer_num = min_unsigned(length, BUFFER_PIXEL_NUM);
  // fill buffer with color data
  for (size_t i = 0; i < buffer_num; i++)
  {
    buffer[i * 2] = hi;
    buffer[i * 2 + 1] = lo;
  }

  write_cmd_repeat_rest(self, ST7789_RAMWR, buffer_num * 2, buffer,
                        chunks, rest * 2);
}

static void ST7789_hard_reset(ST7789_t *self)
{
  gpio_put(self->reset, 1);
  sleep_ms(50);
  gpio_put(self->reset, 0);
  sleep_ms(50);
  gpio_put(self->reset, 1);
  sleep_ms(150);
  gpio_put(self->cs, 1);
}

static void ST7789_soft_reset(ST7789_t *self)
{
  write_cmd(self, ST7789_SWRESET, NULL, 0);
  sleep_ms(150);
}

void ST7789_flush(ST7789_t *self)
{
  dma_channel_wait_for_finish_blocking(self->dma_channel);
  sleep_us(1);
}

void ST7789_draw_pixel(ST7789_t *self, uint16_t x, uint16_t y, uint16_t color)
{
  set_window(self, x, y, x, y);
  fill_color_buffer(self, color, 1);
}

void ST7789_hline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                  uint16_t color)
{
  set_window(self, x, y, x + w - 1, y);
  fill_color_buffer(self, color, w);
}

void ST7789_vline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                  uint16_t color)
{
  set_window(self, x, y, x, y + w - 1);
  fill_color_buffer(self, color, w);
}

void ST7789_fill_rect(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                      uint16_t h, uint16_t color)
{
  set_window(self, x, y, x + w - 1, y + h - 1);
  fill_color_buffer(self, color, w * h);
}

void ST7789_blit_flo_pixmap_t(ST7789_t *self, const flo_pixmap_t *pixmap,
                              uint16_t x, uint16_t y)
{
  if (pixmap == NULL)
  {
    abort();
  }

  set_window(self, x, y, x + pixmap->width - 1, y + pixmap->height - 1);
  write_cmd(self, ST7789_RAMWR, (const uint8_t *)pixmap->buf,
            pixmap->width * pixmap->height * sizeof(uint16_t));
}

void ST7789_backlight(ST7789_t *self, uint8_t brightness)
{
  // gamma correct the provided 0-255 brightness value onto a
  // 0-65535 range for the pwm counter
  const float gamma = 2.8;
  const uint16_t value =
      (uint16_t)(powf((float)(brightness) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(self->backlight, value);
}

ST7789_t *ST7789_spi_create(spi_inst_t *spi_inst, uint16_t width, uint16_t height,
                            ST7789_rotation_t rotation, uint cs, uint reset, uint dc,
                            uint backlight, uint tx, uint sck)
{

  ST7789_t *self = calloc(sizeof(ST7789_t), 1);
  if (self == NULL)
  {
    abort();
  }
  *self = (ST7789_t){.is_spi = true,
                     .spi_obj = spi_inst,
                     .width = width,
                     .height = height,
                     .reset = reset,
                     .dc = dc,
                     .cs = cs,
                     .backlight = backlight};

  // init dditional pins
  gpio_init(self->cs);
  gpio_set_dir(self->cs, GPIO_OUT);

  gpio_init(self->dc);
  gpio_set_dir(self->dc, GPIO_OUT);

  gpio_init(self->reset);
  gpio_set_dir(self->reset, GPIO_OUT);

  // BL PWM Config
  pwm_config pwm_cfg = pwm_get_default_config();
  pwm_set_wrap(pwm_gpio_to_slice_num(self->backlight), 65535);
  pwm_init(pwm_gpio_to_slice_num(self->backlight), &pwm_cfg, true);
  gpio_set_function(self->backlight, GPIO_FUNC_PWM);
  ST7789_backlight(self, 0); // Turn backlight off initially

  spi_init(self->spi_obj, 1500000000);

  gpio_set_function(tx, GPIO_FUNC_SPI);
  gpio_set_function(sck, GPIO_FUNC_SPI);

  spi_set_format(self->spi_obj, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
  int channel = dma_claim_unused_channel(true);
  if (channel < 0)
  {
    panic("no channel available");
  }
  self->dma_channel = (uint)channel;
  dma_channel_config dma_config =
      dma_channel_get_default_config(self->dma_channel);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
  channel_config_set_irq_quiet(&dma_config, true);

  channel_config_set_dreq(&dma_config, spi_get_dreq(self->spi_obj, true));
  dma_channel_set_config(self->dma_channel, &dma_config, false);
  dma_channel_set_write_addr(self->dma_channel, &spi_get_hw(self->spi_obj)->dr,
                             false);

  ST7789_hard_reset(self);
  ST7789_soft_reset(self);
  write_cmd(self, ST7789_SLPOUT, NULL, 0);

  const uint8_t color_mode[] = {COLOR_MODE_65K | COLOR_MODE_16BIT};
  write_cmd(self, ST7789_COLMOD, color_mode, 1);
  sleep_ms(10);
  int ctl = 0;
  switch (rotation)
  {
  case ROTATE_0:
    self->xstart = 240 - width;
    self->ystart = 0;
    ctl = ST7789_MADCTL_MX | ST7789_MADCTL_MY;
    break;
  case ROTATE_180:
    self->xstart = 0;
    self->ystart = 320 - width;
    ctl = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_ML;
    break;
  case ROTATE_90:
  case ROTATE_270:
    self->xstart = 240 - width;
    self->ystart = 0;
    ctl = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_MV;
    break;
  default:
    abort();
  }

  const uint8_t madctl[] = {ctl};
  write_cmd(self, ST7789_MADCTL, madctl, 1);
  write_cmd(self, ST7789_INVON, NULL, 0);
  sleep_ms(10);
  write_cmd(self, ST7789_NORON, NULL, 0);
  sleep_ms(10);
  write_cmd(self, ST7789_DISPON, NULL, 0);
  sleep_ms(100);
  return self;
}

ST7789_t *ST7789_parallel_create(uint16_t width, uint16_t height, uint cs,
                                 uint dc, uint backlight, uint wr_sck,
                                 uint rd_sck, uint d0)
{
  ST7789_t *self = calloc(sizeof(ST7789_t), 1);
  if (self == NULL)
  {
    abort();
  }
  int sm = pio_claim_unused_sm(pio1, true);
  if (sm < 1)
  {
    panic("no sm available");
  }

  *self = (ST7789_t){
      .is_spi = false,
      .width = width,
      .height = height,
      .xstart = 0,
      .ystart =
          320 -
          width, // see
                 // https://github.com/zephyrproject-rtos/zephyr/issues/32286,
      .dc = dc,
      .cs = cs,
      .backlight = backlight,
      .parallel_pio = pio1,
      .parallel_sm = (uint)sm};

  gpio_init(self->cs);
  gpio_set_dir(self->cs, GPIO_OUT);

  gpio_init(self->dc);
  gpio_set_dir(self->dc, GPIO_OUT);

  // BL PWM Config
  pwm_config pwm_cfg = pwm_get_default_config();
  pwm_set_wrap(pwm_gpio_to_slice_num(self->backlight), 65535);
  pwm_init(pwm_gpio_to_slice_num(self->backlight), &pwm_cfg, true);
  gpio_set_function(self->backlight, GPIO_FUNC_PWM);
  ST7789_backlight(self, 0); // Turn backlight off initially

  const int offset = pio_add_program(self->parallel_pio, &st7789_parallel_program);
  if (offset < 0)
  {
    panic("couldn't load program");
  }
  const uint parallel_offset = (uint)offset;

  pio_gpio_init(self->parallel_pio, wr_sck);

  gpio_set_function(rd_sck, GPIO_FUNC_SIO);
  gpio_set_dir(rd_sck, GPIO_OUT);

  for (unsigned int i = 0; i < 8; i++)
  {
    gpio_set_function(d0 + i, GPIO_FUNC_SIO);
    gpio_set_dir(d0 + i, GPIO_OUT);
  }

  for (unsigned int i = 0; i < 8; i++)
  {
    pio_gpio_init(self->parallel_pio, d0 + i);
  }

  pio_sm_set_consecutive_pindirs(self->parallel_pio, self->parallel_sm, d0, 8,
                                 true);
  pio_sm_set_consecutive_pindirs(self->parallel_pio, self->parallel_sm, wr_sck,
                                 1, true);

  pio_sm_config c = st7789_parallel_program_get_default_config(parallel_offset);

  sm_config_set_out_pins(&c, d0, 8);
  sm_config_set_sideset_pins(&c, wr_sck);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  sm_config_set_out_shift(&c, false, true, 8);

  // Determine clock divider
  const uint32_t max_pio_clk = 32 * MHZ;
  const uint32_t sys_clk_hz = clock_get_hz(clk_sys);
  const uint32_t clk_div = (sys_clk_hz + max_pio_clk - 1) / max_pio_clk;
  sm_config_set_clkdiv(&c, clk_div);

  pio_sm_init(self->parallel_pio, self->parallel_sm, parallel_offset, &c);
  pio_sm_set_enabled(self->parallel_pio, self->parallel_sm, true);

  // Allocate, config and init an dma
  const int channel = dma_claim_unused_channel(true);
  if (channel < 1)
  {
    panic("couldn't alloc channel");
  }
  self->dma_channel = (uint)channel;
  dma_channel_config dma_config =
      dma_channel_get_default_config(self->dma_channel);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
  channel_config_set_write_increment(&dma_config, false);
  channel_config_set_read_increment(&dma_config, true);
  channel_config_set_bswap(&dma_config, false);

  channel_config_set_dreq(
      &dma_config, pio_get_dreq(self->parallel_pio, self->parallel_sm, true));
  dma_channel_configure(self->dma_channel, &dma_config,
                        &self->parallel_pio->txf[self->parallel_sm], NULL, 0,
                        false);

  gpio_put(rd_sck, 1);

  ST7789_soft_reset(self);
  const uint8_t color_mode[] = {COLOR_MODE_65K | COLOR_MODE_16BIT};
  write_cmd(self, ST7789_COLMOD, color_mode, 1);

  write_cmd(self, ST7789_INVON, NULL, 0);  // set inversion mode
  write_cmd(self, ST7789_SLPOUT, NULL, 0); // leave sleep mode
  write_cmd(self, ST7789_DISPON, NULL, 0); // turn display on

  const uint8_t madctl[] = {ST7789_MADCTL_MY | ST7789_MADCTL_MV |
                            ST7789_MADCTL_RGB};
  write_cmd(self, ST7789_MADCTL, madctl, 1);

  return self;
}
