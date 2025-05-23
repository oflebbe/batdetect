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
#include "hardware/clocks.h"
#define MEOW_FFT_IMPLEMENTATION
#include "meow_fft.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#define FLO_PIXMAP_IMPLEMENTATION
#include "flo_pixmap.h"
#include "st7789.h"

#include "f_util.h"
#include "ff.h"

#ifdef PICO_W
#include "boards/pico_w.h"
#include "pico/cyw43_arch.h"
#else
#include "boards/pico.h"
#endif

#define SMALL_SPI 1

const bool STEREO = true;

const unsigned int WIDTH = 240;
const unsigned int HEIGHT = 240;
const unsigned int ROTATION = ROTATE_180;

const unsigned int LABEL_SIZE = 2 * 6;

const unsigned int WIDTH_DISPLAY = WIDTH - LABEL_SIZE;
const unsigned int HEIGHT_DISPLAY = HEIGHT - 8;
#define NCOLORS 256

const int ST7789_CS = 9;
const int ST7789_DC = 8;
const int ST7789_RST = 7;
const int ST7789_BL = 6;
#define ST7789_INSTANCE spi1
const int ST7789_TX = 11;
const int ST7789_CLK = 10;

// set this to determine sample rate
// 96    = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz

const int CLOCK_DIV = 96 * 2;
const int CLOCK_DIV_STEREO = 96;
const unsigned int FSAMP = 500000 / 2;

#define USE_FFT 1
#include "hw_config.h"

// Channel 0 is GPIO26
const int CAPTURE_CHANNEL = 0;

// because FFT_SIZE/2 > HEIGHT
constexpr unsigned int FFT_SIZE=512u;

const uint FACTOR = 300u;

const uint NUM_SAMPLES = FFT_SIZE * FACTOR;

// Queue for multiprocessing
queue_t capture_queue;
queue_t display_queue;

static inline float squaref(float x) { return x * x; }

void led(uint on)
{
#ifndef PICO_W
  gpio_put(PICO_DEFAULT_LED_PIN, on);
#else
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
#endif
}

// setup audio
// for stereo capture channel and capture_channel+1

void setup_for_audio(uint capture_channel, bool stereo, uint sample_dma_channel)
{
  adc_gpio_init(26 + capture_channel);
  if (stereo)
  {
    adc_gpio_init(26 + capture_channel + 1);
  }
  adc_init();

  // adc_select_input(capture_channel+1);

  if (stereo)
  {
    adc_set_round_robin(3); // channel 1 + 2
  }
  adc_select_input( 1);

  adc_fifo_drain();
  adc_fifo_setup(
      true,  // Write each completed conversion to the sample FIFO
      true,  // Enable DMA data request (DREQ)
      1,     // DREQ (and IRQ) asserted when at least 1 sample present
      true, // We won't see the ERR bit because of 8 bit reads; disable.
      false  // Shift each sample to 8 bits when pushing to FIFO
  );

  // set sample rate
  adc_set_clkdiv(stereo ? CLOCK_DIV_STEREO : CLOCK_DIV);
  sleep_ms(1000);

  dma_channel_config cfg = dma_channel_get_default_config(sample_dma_channel);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_high_priority(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);
  dma_channel_set_config(sample_dma_channel, &cfg, false);

  dma_channel_set_read_addr(sample_dma_channel, &adc_hw->fifo, false);
}

ST7789_t *setup(uint sample_dma_channel, bool stereo)
{
#ifndef PICO_W
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#else
  cyw43_arch_init();
#endif

  setup_for_audio(0, stereo, sample_dma_channel);

#ifdef SMALL_SPI
  ST7789_t *sobj = ST7789_spi_create(ST7789_INSTANCE, WIDTH, HEIGHT, ROTATION,
                                     ST7789_CS, ST7789_RST, ST7789_DC,
                                     ST7789_BL, ST7789_TX, ST7789_CLK);
#else
  ST7789_t *sobj = ST7789_parallel_create(WIDTH, HEIGHT, PIN_CS, PIN_DC, PIN_BL,
                                          PIN_WR, PIN_RD, PIN_D0);
#endif
  ST7789_backlight(sobj, 0xff);
  return sobj;
}

// capture number of samples in capture_buf
// stereo: true it is alternating L and R
void sample(uint dma_chan, uint32_t samples, bool stereo, uint16_t capture_buf[samples])
{
  
 adc_select_input( 1);

  // set number of samples (2 bytes each) to read

  dma_channel_set_trans_count(dma_chan, samples, false);
  // set write address and start
  dma_channel_set_write_addr(dma_chan, capture_buf, true);

  adc_fifo_drain();
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
  adc_run(false);

  adc_fifo_drain();

}

static void init_hamming(int len, float window[len])
{
  const float a0 = 25. / 46.;
  for (int i = 0; i < len; i++)
  {
    window[i] = a0 - (1.0 - a0) * cosf((M_TWOPI * i) / (len - 1));
  }
}

static unsigned int get_freqs(unsigned int pos, unsigned int num_freq)
{
  unsigned int start_freq = FFT_SIZE / 2u - num_freq + pos;
  return (start_freq * FSAMP / 1000) / FFT_SIZE;
}

static int draw_labels(ST7789_t *sobj, unsigned int num_freq)
{
  const unsigned int NLABELS = 4;

  for (size_t i = 0; i < NLABELS; i++)
  {
    char text[4] = {0};
    assert((i * num_freq) / NLABELS < num_freq);
    snprintf(text, sizeof(text), "%2d", get_freqs((i * num_freq) / NLABELS, num_freq));
    const flo_pixmap_t *label_o1af_pixmap =
        flo_pixmap_create_str(sizeof(text), text, WHITE, BLACK, 1, 1);
    ST7789_blit_flo_pixmap_t(sobj, label_o1af_pixmap, 0, HEIGHT_DISPLAY - (i * num_freq) / NLABELS - 1);
    ST7789_flush(sobj);
    free((void *)label_o1af_pixmap);
  }
  return 2 * 8; // because of %2d an 1 scale
}

void draw_int(ST7789_t *sobj, uint value, int xoffset)
{
  char text[5] = {0};
  snprintf(text, sizeof(text), "%4u", value);
  const flo_pixmap_t *label_pixmap =
      flo_pixmap_create_str(sizeof(text), text, WHITE, BLACK, 1, 1);
  ST7789_blit_flo_pixmap_t(sobj, label_pixmap, xoffset, HEIGHT - 8);
  ST7789_flush(sobj);
  free((void *)label_pixmap);
}

const char filename_pattern[] = "bat%03d/capture%05d.raw";

uint sd_card_search(FATFS *fs)
{
  DIR *dir = calloc(1, sizeof(DIR));
  FILINFO *fi = calloc(1, sizeof(FILINFO));
  uint num = 0;
  uint last = 0;
  FRESULT res = f_findfirst(dir, fi, "/", "bat???");
  if (res != FR_OK)
  {
    panic("f_findfirst failed");
  }
  do
  {
    if (fi->fname[0] == 0)
    {
      break;
    }
    sscanf(fi->fname, "bat%u", &num);
    if (num > last)
    {
      last = num;
    }
  } while (f_findnext(dir, fi) == FR_OK);
  last++;
  // FAT only supports 500 something entries in Root
  if (last > 500)
  {
    panic("too many subdirs");
  }
  const int STRLEN = 10;
  static char str[10];
  snprintf(str, STRLEN, "bat%03d", last);
  if (FR_OK != f_mkdir(str))
  {
    panic("canoot create subdir");
  }
  free(fi);
  free(dir);
  return last;
}

FIL fil;
bool is_open = false;

void sd_write(FATFS *fs, unsigned int dir_count, unsigned int file_count, UINT size, uint16_t cap_buf[size])
{
  static int counter = 0;
  led(1);
  if (!is_open)
  {
    const int buf_len = snprintf(NULL, 0, filename_pattern, dir_count, file_count);
    if (buf_len <= 0)
    {
      panic("snprintf failed");
    }
    char str_buffer[buf_len + 1];

    snprintf(str_buffer, sizeof(str_buffer), filename_pattern, dir_count, file_count);

    int fr = f_open(&fil, str_buffer, FA_WRITE | FA_CREATE_ALWAYS);
    if (FR_OK != fr && FR_EXIST != fr)
      panic("f_open(%s) error: %s (%d)\n", str_buffer, FRESULT_str(fr), fr);
    is_open = true;
  }
  UINT written;
  int res = f_write(&fil, cap_buf, size * 2, &written);
   if (written != size * 2 || res != FR_OK)
  {
    panic("could not write buffer len %d, written %d, result %d\n", size * 2,
          written, res);
  }
  counter = (counter + 1) % 10;
  if (counter==0) {
     f_sync(&fil);
  }
  /*fr = f_close(&fil);
  if (FR_OK != fr)
  {
    printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
  }*/

  led(0);
}

void worker_display(void)
{
  // create hamming window for FFT
  // FFT_SIZE entries
  static float hamming[FFT_SIZE] = {0};
  init_hamming(FFT_SIZE, hamming);

  // create color_table
  uint16_t color_table[NCOLORS] = {0};
  for (int i = 0; i < NCOLORS; i++)
  {
    color_table[i] = flo_hslToRgb565(((float)i) / (float)NCOLORS, 1.0f, 0.5f);
  }

  flo_pixmap_t *display = flo_pixmap_create(WIDTH_DISPLAY, HEIGHT_DISPLAY);
  size_t workset_bytes = meow_fft_generate_workset_real(FFT_SIZE, NULL);
  Meow_FFT_Workset_Real *fft_real =
      (Meow_FFT_Workset_Real *)malloc(workset_bytes);
  meow_fft_generate_workset_real(FFT_SIZE, fft_real);
  while (true)
  {
    uint16_t *cap_buf = NULL;
    queue_remove_blocking(&capture_queue, &cap_buf);

    const unsigned int NUM_MONO_SAMPLES = STEREO ? NUM_SAMPLES / 2 : NUM_SAMPLES;
    const unsigned int last_valid_start = NUM_MONO_SAMPLES - FFT_SIZE;

    float *fft_in = malloc(FFT_SIZE * sizeof(float));
    Meow_FFT_Complex *fft_out = malloc((FFT_SIZE / 2 + 1) * sizeof(Meow_FFT_Complex));
    for (unsigned int column = 0; column < display->width; column++)
    {

      const unsigned int start = (column * last_valid_start) / display->width;
      for (unsigned int i = 0; i < FFT_SIZE; i++)
      {
        unsigned int index = i + start;
        if (STEREO)
        {
          index *= 2;
        }
        assert(index < NUM_SAMPLES);
        // convert to float
        // fill fourier transform input
        // subtract voltage offset
        // multiply with window function

        fft_in[i] = ((float)cap_buf[index] - 2048.0f) * hamming[i];
      }

      // compute fast fourier transform
      meow_fft_real(fft_real, fft_in, fft_out);

      // compute power and calculate max freq component

      // consider the freq in WIDTH raster
      // 12 bit ADC 0 - 4095
      // 3V max Ampl, 3.3V reference
      // scale is SQR( 4095  * 256 / 2)
      // log10(scale) = 11.43892
      // empirical lower limit log10(lower limit ) = 4
      assert(FFT_SIZE >= HEIGHT_DISPLAY);
      for (unsigned int y = 0; y < HEIGHT_DISPLAY; y++)
      {
        const float power = squaref(fft_out[y].r) + squaref(fft_out[y].j);
        int pix = (log10f(power) - 4) / (11. - 4.) * NCOLORS;
        if (pix < 0)
        {
          pix = 0;
        }
        if (pix >= NCOLORS)
        {
          pix = NCOLORS - 1;
        }
        uint16_t c = color_table[pix];
        assert(column >= 0);
        assert(column < WIDTH_DISPLAY);

        flo_pixmap_set_pixel(display, column, HEIGHT_DISPLAY - y - 1, c);
      }
    }
    free(fft_in);
    free(fft_out);

    queue_add_blocking(&display_queue, &display);
  }
}

int main()
{
  // set_sys_clock_khz(250000, true);
  // Queue initialization
  sleep_ms(100);
  stdio_init_all();

  queue_init(&capture_queue, sizeof(uint16_t *), 1);
  queue_init(&display_queue, sizeof(flo_pixmap_t *), 1);
  multicore_reset_core1();
  sleep_ms(100);
  multicore_launch_core1(worker_display);

  // SDCard init, look for free slot
  FATFS fs;
  FRESULT fr = f_mount(&fs, "", 1);
  if (FR_OK != fr)
  {
    panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
  }
  uint dir_count = sd_card_search(&fs);
  uint file_count = 0;
  int channel = dma_claim_unused_channel(true);
  if (channel < 0) {
    panic("no channel available");
  }
  const uint sample_dma_chan = (uint) channel;

  // setup ports and outputs
  ST7789_t *sobj = setup(sample_dma_chan, STEREO);

  ST7789_fill_rect(sobj, 0, 0, WIDTH, HEIGHT, BLACK);
  // calculate frequencies of each bin and draw legend

  draw_labels(sobj, HEIGHT_DISPLAY);
  uint16_t *cap_buf = calloc(NUM_SAMPLES, sizeof(uint16_t));

  while (1)
  {
    // get NUM_SAMPLES samples at FSAMP
    sample(sample_dma_chan, NUM_SAMPLES, STEREO, cap_buf);
    absolute_time_t start_processing = get_absolute_time();

    // send to FFT
    queue_add_blocking(&capture_queue, &cap_buf);

    /// write out in parallel to FFT
    sd_write(&fs, dir_count, file_count, NUM_SAMPLES, cap_buf);
    draw_int(sobj, dir_count, 150);
    draw_int(sobj, file_count++, 200);

    // get audiogram result from FFT
    flo_pixmap_t *display = NULL;
    queue_remove_blocking(&display_queue, &display);
    absolute_time_t end_processing = get_absolute_time();
    int64_t diff = absolute_time_diff_us(start_processing, end_processing);
    draw_int(sobj, diff / 1000, 50);
    // draw audiogram display
    ST7789_blit_flo_pixmap_t(sobj, display, LABEL_SIZE, 0);
  }

  // should never get here
}
