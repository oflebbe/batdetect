// Sample from the ADC continuously at a particular sample rate
// and then compute an FFT over the data
//
// much of this code is from pico-examples/adc/dma_capture/dma_capture.c
// the rest is written by Alex Wulff (www.AlexWulff.com)

#include <math.h>
#include <stdio.h>
#include <strings.h>

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/structs/systick.h"
#include "kiss_fftr.h"
#include "pico/stdlib.h"
#include "st7789.h"

#define SMALL_SPI 1

const int WIDTH = 240;
const int HEIGHT = 240;
const int ROTATION = 180;

#ifdef BREADBOARD
#define PIN_CS 18
#define PIN_DC 19
#define PIN_RST 20
#define PIN_BL 21
#define LED_PIN 25
#define SPI_INSTANCE spi_default
#define SPI_TX PICO_DEFAULT_SPI_TX_PIN
#define SPI_CLK PICO_DEFAULT_SPI_SCK_PIN
#else
#include "pico/cyw43_arch.h"

#define PIN_CS 9
#define PIN_DC 8
#define PIN_RST 7
#define PIN_BL 6
#define SPI_INSTANCE spi1
#define SPI_TX 11
#define SPI_CLK 10
#endif

// set this to determine sample rate
// 96    = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
const int CLOCK_DIV = 96 * 2;
const int FSAMP = 500000 / 2;

// Channel 0 is GPIO26
const int CAPTURE_CHANNEL = 0;

// because NSAMP/2 > WIDTH
const int NSAMP = 512;

void start_timer() {
  systick_hw->csr = 0x5;
  systick_hw->rvr = 0x00FFFFFF;
}

int stop_timer() {
  int ticks = 0x00FFFFFF - systick_hw->cvr;
  systick_hw->csr = 0x0;
  systick_hw->rvr = 0x00FFFFFF;
  return ticks;
}

static inline float sqr(float x) { return x * x; }

void led(uint on) {
#ifdef BREADBOARD
  gpio_put(LED_PIN, on);
#else
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
#endif
}

ST7789_t *setup(uint sample_dma_channel) {
  stdio_init_all();
#ifdef BREADBOARD
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
#else
  cyw43_arch_init();
#endif

  adc_gpio_init(26 + CAPTURE_CHANNEL);
  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_drain();
  adc_fifo_setup(
      true,  // Write each completed conversion to the sample FIFO
      true,  // Enable DMA data request (DREQ)
      1,     // DREQ (and IRQ) asserted when at least 1 sample present
      false, // We won't see the ERR bit because of 8 bit reads; disable.
      false  // Shift each sample to 8 bits when pushing to FIFO
  );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);

  sleep_ms(1000);

  dma_channel_config cfg = dma_channel_get_default_config(sample_dma_channel);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);
  dma_channel_set_config(sample_dma_channel, &cfg, false);

  dma_channel_set_read_addr(sample_dma_channel, &adc_hw->fifo, false);
  dma_channel_set_trans_count(sample_dma_channel, NSAMP, false);

#ifdef SMALL_SPI
  ST7789_t *sobj =
      ST7789_spi_create(SPI_INSTANCE, WIDTH, HEIGHT, ROTATION, PIN_CS, PIN_RST,
                        PIN_DC, PIN_BL, SPI_TX, SPI_CLK);
#else
  ST7789_t *sobj = ST7789_parallel_create(WIDTH, HEIGHT, PIN_CS, PIN_DC, PIN_BL,
                                          PIN_WR, PIN_RD, PIN_D0);
#endif
  ST7789_backlight(sobj, 0xff);
  return sobj;
}

void sample(uint dma_chan, uint16_t *capture_buf) {
  adc_run(false);

  // set write address and start
  dma_channel_set_write_addr(dma_chan, capture_buf, true);
  led(1);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
  led(0);
}

void measure(uint sample_dma_chan, kiss_fftr_cfg cfg, int len,
             uint8_t pix[len]) {
  uint16_t cap_buf[NSAMP];
  kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float

  // get NSAMP samples at FSAMP
  sample(sample_dma_chan, cap_buf);
  // fill fourier transform input
  for (int i = 0; i < NSAMP; i++) {
    fft_in[i] = (float)cap_buf[i];
  }

  kiss_fft_cpx fft_out[NSAMP];
  // compute fast fourier transform
  kiss_fftr(cfg, fft_in, fft_out);
  // compute power and calculate max freq component

  // consider the freq in WIDTH raster
  // 12 bit ADC 0 - 4095
  // 3V max Ampl, 3.3V reference
  // SQR( 4095  * 256 / 2)
  float scale = HEIGHT / 1.09383754e+11;

  for (int i = 0; i < len; i++) {
    int j = NSAMP / 2 - len + i;
    float power = sqr(fft_out[j].r) + sqr(fft_out[j].i);

    pix[i] = power * scale;
  }
}

float hue2rgb(float p, float q, float t) {
  if (t < 0)
    t += 1;
  if (t > 1)
    t -= 1;
  if (t < (1.0 / 6))
    return p + (q - p) * 6 * t;
  if (t < (1. / 2.))
    return q;
  if (t < (2. / 3.))
    return p + (q - p) * ((2. / 3.) - t) * 6;
  return p;
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

uint16_t hslToRgb565(float h, float s, float l) {
  float r, g, b;

  if (s == 0) {
    r = g = b = l; // achromatic
  } else {

    float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    float p = 2 * l - q;

    r = hue2rgb(p, q, h + 1 / 3.);
    g = hue2rgb(p, q, h);
    b = hue2rgb(p, q, h - 1 / 3.);
  }

  return color565((uint8_t)(r * 255.f), (uint8_t)(b * 255.f),
                  (uint8_t)(g * 255.f));
}

const int label_size = 16;

ST7789_bitmap_t *graphics_init(int width, int height, int ncolors,
                               uint16_t color_table[ncolors]) {
  for (int i = 0; i < ncolors; i++) {
    color_table[i] = hslToRgb565(((float)i) / (float)ncolors, 1.0f, 0.5f);
  }
  return ST7789_create_bitmap(width, height);
}

void graphics_pixel(ST7789_bitmap_t *bitmap, int x, int y, int16_t color,
                    int ncolors, uint16_t color_table[ncolors]) {
  int16_t c = 0;
  if (color > 5) {
    c = swap(color_table[color % ncolors]);
  }
  bitmap->buf[y * bitmap->width + x] = c;
}

// scroll on colum to right
void scroll(ST7789_bitmap_t *bitmap) {
  for (int y = 0; y < bitmap->height; y++) {
    memmove(&bitmap->buf[y * bitmap->width + 1],
            &bitmap->buf[y * bitmap->width], 2 * (bitmap->width - 1));
  }
}

int main() {
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);

  const uint sample_dma_chan = dma_claim_unused_channel(true);
  // setup ports and outputs
  ST7789_t *sobj = setup(sample_dma_chan);

  // calculate frequencies of each bin

  int freqs[WIDTH];
  for (int i = 0; i < WIDTH; i++) {
    int j = NSAMP / 2 - WIDTH + i;
    freqs[i] = (j * FSAMP / 1000) / NSAMP;
  }
  const int NLABELS = 4;
  ST7789_bitmap_t *label_bitmap[NLABELS];

  for (int i = 0; i < NLABELS; i++) {
    char text[4] = {0};
    snprintf(text, sizeof(text), "%2d", freqs[(i * WIDTH) / NLABELS]);
    label_bitmap[i] =
        ST7789_create_str_bitmap(sizeof(text), text, WHITE, BLACK, 1, 1);
  }

  ST7789_fill_rect(sobj, 0, 0, WIDTH, HEIGHT, BLACK);
  for (int i = 0; i < NLABELS; i++) {
    ST7789_blit_bitmap(sobj, label_bitmap[i], 0, (i * WIDTH) / NLABELS);
  }



  const uint NCOLORS = 256;
  uint16_t color_table[NCOLORS];
  ST7789_bitmap_t *display = graphics_init(WIDTH-label_size, HEIGHT, NCOLORS, color_table);
  while (1) {
    uint8_t pix[HEIGHT];
    measure(sample_dma_chan, cfg, HEIGHT, pix);

    scroll(display);
    for (int y = 0; y < HEIGHT; y++) {
      graphics_pixel(display, 0, y, pix[y], NCOLORS, color_table);
    }

    // clear display
    ST7789_blit_bitmap(sobj, display, label_size, 0);
  }
  // should never get here
  kiss_fft_free(cfg);
}
