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
#define COLOR_MODE_65K      0x50
#define COLOR_MODE_262K     0x60
#define COLOR_MODE_12BIT    0x03
#define COLOR_MODE_16BIT    0x05
#define COLOR_MODE_18BIT    0x06
#define COLOR_MODE_16M      0x07

// commands
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A 
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

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


#define ST7789_MADCTL_MY  0x80  // Page Address Order
#define ST7789_MADCTL_MX  0x40  // Column Address Order
#define ST7789_MADCTL_MV  0x20  // Page/Column Order
#define ST7789_MADCTL_ML  0x10  // Line Address Order
#define ST7789_MADCTL_MH  0x04  // Display Data Latch Order
#define ST7789_MADCTL_RGB 0x00
#define ST7789_MADCTL_BGR 0x08

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

typedef struct _ST7789_t {
  spi_inst_t *spi_obj;
  uint width;
  uint height;
  uint xstart;
  uint ystart;
  uint reset;
  uint dc;
  uint cs;
  uint backlight;
  uint parallel_sm;
  PIO parallel_pio;
  uint st_dma;
} ST7789_t;

extern const unsigned char font[];

uint16_t swap(uint16_t color) {
  uint8_t hi = color >> 8, lo = color;
  return hi | (lo << 8);
}

static void write_blocking_parallel(ST7789_t *self, const uint8_t *src,
                                    size_t len) {
  while (len--) {
    uint32_t p = *src++ << 24;
    pio_sm_put_blocking(self->parallel_pio, self->parallel_sm, p);
    asm("nop;");
  }
}

static void write_blocking_dma(ST7789_t *self, const uint8_t *src, size_t len) {
  dma_channel_set_trans_count(self->st_dma, len, false);
  dma_channel_set_read_addr(self->st_dma, src, true);
  dma_channel_wait_for_finish_blocking(self->st_dma);

  sleep_us(20); // We shouldn't overrun the ST7789
}

static void write_blocking(ST7789_t *self, const uint8_t *src, size_t len) {
  if (len > 100) {
    write_blocking_dma(self, src, len);
  }
  if (self->spi_obj) {
    spi_write_blocking(self->spi_obj, src, len);
  } else {
    write_blocking_parallel(self, src, len);
  }
}

static void write_cmd_repeat_rest(ST7789_t *self, uint8_t cmd,
                                  const uint8_t *data, size_t len, int repeat,
                                  int rest) {
  gpio_put(self->cs, 0);
  if (cmd) {
    gpio_put(self->dc, 0);
    write_blocking(self, &cmd, 1);
  }
  if (len > 0) {
    gpio_put(self->dc, 1);
    while (repeat-- > 0) {
      write_blocking(self, data, len);
    }
    if (rest > 0) {
      write_blocking(self, data, rest);
    }
  }
  gpio_put(self->cs, 1);
}

static void write_cmd(ST7789_t *self, uint8_t cmd, const uint8_t *data,
                      size_t len) {
  write_cmd_repeat_rest(self, cmd, data, len, 1, 0);
}

static void set_window(ST7789_t *self, uint16_t x0, uint16_t y0, uint16_t x1,
                       uint16_t y1) {

  uint8_t bufx[4] = {(x0 + self->xstart) >> 8, (x0 + self->xstart) & 0xFF,
                     (x1 + self->xstart) >> 8, (x1 + self->xstart) & 0xFF};
  uint8_t bufy[4] = {(y0 + self->ystart) >> 8, (y0 + self->ystart) & 0xFF,
                     (y1 + self->ystart) >> 8, (y1 + self->ystart) & 0xFF};
  write_cmd(self, ST7789_CASET, bufx, 4);
  write_cmd(self, ST7789_RASET, bufy, 4);
}

static void fill_color_buffer(ST7789_t *self, uint16_t color, int length) {
  uint8_t hi = color >> 8, lo = color;
  const int buffer_pixel_size = 240*2;
  int chunks = length / buffer_pixel_size;
  int rest = length % buffer_pixel_size;

  uint8_t buffer[buffer_pixel_size * 2];
  // fill buffer with color data
  for (int i = 0; i < length && i < buffer_pixel_size; i++) {
    buffer[i * 2] = hi;
    buffer[i * 2 + 1] = lo;
  }

  write_cmd_repeat_rest(self, ST7789_RAMWR, buffer, buffer_pixel_size * 2,
                        chunks, rest * 2);
}

static void ST7789_hard_reset(ST7789_t *self) {
  gpio_put(self->reset, 1);
  sleep_ms(50);
  gpio_put(self->reset, 0);
  sleep_ms(50);
  gpio_put(self->reset, 1);
  sleep_ms(150);
  gpio_put(self->cs, 1);
}

static void ST7789_soft_reset(ST7789_t *self) {
  write_cmd(self, ST7789_SWRESET, NULL, 0);
  sleep_ms(150);
}

void ST7789_draw_pixel(ST7789_t *self, uint16_t x, uint16_t y, uint16_t color) {
  set_window(self, x, y, x, y);
  fill_color_buffer(self, color, 1);
}

void ST7789_hline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                  uint16_t color) {
  set_window(self, x, y, x + w - 1, y);
  fill_color_buffer(self, color, w);
}

void ST7789_vline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                  uint16_t color) {
  set_window(self, x, y, x, y + w - 1);
  fill_color_buffer(self, color, w);
}

void ST7789_fill_rect(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                      uint16_t h, int16_t color) {
  set_window(self, x, y, x + w - 1, y + h - 1);
  fill_color_buffer(self, color, w * h);
}


void ST7789_draw_char_bitmap( ST7789_bitmap_t *bitmap, int off_x, int off_y, unsigned char c, int16_t color, uint16_t bg,
                            int8_t size_x, int8_t size_y) {
  if (off_x < 0 || off_y < 0) {
    return;
  }
  if (off_x + (size_x - 1) + ST7789_char_width > bitmap->width || off_y + (size_y - 1) +  ST7789_char_height> bitmap->height) {
    return;
  }
  
  if (c >= 176)
    c++; // Handle 'classic' charset behavior

  color = swap(color);
  bg = swap(bg);
  

  for (int8_t i = 0; i < ST7789_char_width; i++) { // Char bitmap = 5 columns
    uint8_t line = font[(int)c * ST7789_char_width + i];
    for (int8_t j = 0; j < ST7789_char_height; j++, line >>= 1) {
      for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
          bitmap->buf[((off_y + j*size_y + y) * bitmap->width) + (off_x + i*size_x + x)] = line & 1 ? color: bg;
        }
      }
    }
  }
}

void ST7789_create_str_bitmap( ST7789_bitmap_t *bitmap, const char *str, int len, int16_t color, uint16_t bg,   int8_t size_x, int8_t size_y) {
    if (!bitmap->buf) {
      bitmap->width = len * (ST7789_char_width + ST7789_char_space) * size_x;
      bitmap->height =  ST7789_char_height * size_y;
      bitmap->buf = calloc( bitmap->width * bitmap->height, sizeof(int16_t));
    }
    for (int i = 0; i < len; i++) {
      ST7789_draw_char_bitmap( bitmap, i*(ST7789_char_width+ST7789_char_space)*size_x, 0, str[i], color, bg, size_x, size_y);
    }
}

void ST7789_blit_bitmap(ST7789_t *self, const ST7789_bitmap_t *bitmap,
                        int16_t x, int16_t y) {
  set_window(self, x, y, x + bitmap->width - 1, y + bitmap->height - 1);
  write_cmd(self, ST7789_RAMWR, (const uint8_t *) bitmap->buf, bitmap->width * bitmap->height * sizeof(int16_t));
}

void ST7789_backlight(ST7789_t *self, uint8_t brightness) {
  // gamma correct the provided 0-255 brightness value onto a
  // 0-65535 range for the pwm counter
  float gamma = 2.8;
  uint16_t value =
      (uint16_t)(powf((float)(brightness) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(self->backlight, value);
}

void ST7789_blit_buffer(ST7789_t *self, const uint8_t *buf, size_t buf_len,
                        int16_t x, int16_t y, int16_t w, int16_t h) {
  set_window(self, x, y, x + w - 1, y + h - 1);
  write_cmd(self, ST7789_RAMWR, buf, buf_len);
}

static struct _ST7789_t st7789;

ST7789_t *ST7789_spi_create(spi_inst_t *spi_obj, int16_t width, int16_t height,
                            uint cs, uint reset, uint dc, uint backlight,
                            uint tx, uint sck) {

  // YUCK
  ST7789_t *self = &st7789;
  self->spi_obj = spi_obj;
  self->width = width;
  self->height = height;


  self->xstart = 0;
  // We have it rotated it 180°
  // otherwise we show wrong part of framebuffer
  self->ystart = 320-width;  // see https://github.com/zephyrproject-rtos/zephyr/issues/32286

  self->reset = reset;
  self->dc = dc;
  self->cs = cs;
  self->backlight = backlight;

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

  spi_init(self->spi_obj, 50000000);

  gpio_set_function(tx, GPIO_FUNC_SPI);
  gpio_set_function(sck, GPIO_FUNC_SPI);

  spi_set_format(self->spi_obj, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);

  self->st_dma = dma_claim_unused_channel(true);
  dma_channel_config dma_config = dma_channel_get_default_config(self->st_dma);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
  channel_config_set_irq_quiet(&dma_config, true);

  channel_config_set_dreq(&dma_config, spi_get_dreq(self->spi_obj, true));
  dma_channel_set_config(self->st_dma, &dma_config, false);
  dma_channel_set_write_addr(self->st_dma, &spi_get_hw(self->spi_obj)->dr,
                             false);

  ST7789_hard_reset(self);
  ST7789_soft_reset(self);
  write_cmd(self, ST7789_SLPOUT, NULL, 0);

  const uint8_t color_mode[] = {COLOR_MODE_65K | COLOR_MODE_16BIT};
  write_cmd(self, ST7789_COLMOD, color_mode, 1);
  sleep_ms(10);
  // 180° rotated
  const uint8_t madctl[] = {ST7789_MADCTL_MX |  ST7789_MADCTL_MY | ST7789_MADCTL_ML};
  write_cmd(self, ST7789_MADCTL, madctl, 1);

  write_cmd(self, ST7789_INVON, NULL, 0);
  sleep_ms(10);
  write_cmd(self, ST7789_NORON, NULL, 0);
  sleep_ms(10);
  write_cmd(self, ST7789_DISPON, NULL, 0);
  sleep_ms(100);
  return self;
}

ST7789_t *ST7789_parallel_create(int16_t width, int16_t height, uint cs,
                                 uint dc, uint backlight, uint wr_sck,
                                 uint rd_sck, uint d0) {
  // YUCK
  ST7789_t *self = &st7789;
  self->spi_obj = NULL;
  self->width = width;
  self->height = height;
  self->xstart = 0;
  self->ystart = 0;
  self->reset = 0; // no reset pin

  self->dc = dc;
  self->cs = cs;
  self->backlight = backlight;

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

  self->parallel_pio = pio1;
  self->parallel_sm = pio_claim_unused_sm(self->parallel_pio, true);
  uint parallel_offset =
      pio_add_program(self->parallel_pio, &st7789_parallel_program);
  pio_gpio_init(self->parallel_pio, wr_sck);

  gpio_set_function(rd_sck, GPIO_FUNC_SIO);
  gpio_set_dir(rd_sck, GPIO_OUT);

  for (int i = 0; i < 8; i++) {
    gpio_set_function(d0 + i, GPIO_FUNC_SIO);
    gpio_set_dir(d0 + i, GPIO_OUT);
  }

  for (int i = 0u; i < 8; i++) {
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
  self->st_dma = dma_claim_unused_channel(true);
  dma_channel_config dma_config = dma_channel_get_default_config(self->st_dma);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
  channel_config_set_write_increment(&dma_config, false);
  channel_config_set_read_increment(&dma_config, true);
  channel_config_set_bswap(&dma_config, false);

  channel_config_set_dreq(
      &dma_config, pio_get_dreq(self->parallel_pio, self->parallel_sm, true));
  dma_channel_configure(self->st_dma, &dma_config,
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
