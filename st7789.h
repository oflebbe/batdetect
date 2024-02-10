#ifndef __ST7789_H__
#define __ST7789_H__

// Color definitions
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#include <hardware/spi.h>

typedef struct _ST7789_t ST7789_t;

// 16bit color bitmap
typedef struct bitmap {
  uint16_t *buf;
  int width;
  int height;
} ST7789_bitmap_t;

static const int ST7789_char_width = 5;
static const int ST7789_char_space = 1;
static const int ST7789_char_height = 8;

ST7789_t *ST7789_spi_create( spi_inst_t *spi_obj, int16_t width, int16_t height, uint cs, uint reset, uint dc, uint backlight, uint tx, uint sck);
ST7789_t *ST7789_parallel_create( int16_t width, int16_t height, uint cs, uint dc, uint backlight, uint wr, uint rd, uint d0);
void ST7789_blit_buffer(ST7789_t *self, const uint8_t *buf, size_t buf_len,
                        int16_t x, int16_t y, int16_t w, int16_t h);
void ST7789_backlight(ST7789_t *self, uint8_t bl);
void ST7789_hline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7789_vline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7789_fill_rect(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w, uint16_t h, int16_t color);

void ST7789_create_str_bitmap( ST7789_bitmap_t *bitmap, const char *str, int len, int16_t color, uint16_t bg,   int8_t size_x, int8_t size_y);
void ST7789_blit_bitmap(ST7789_t *self, const ST7789_bitmap_t *bitmap, int16_t x, int16_t y);

#define min(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a < _b ? _a : _b;                                                         \
  })


#endif  /*  __ST7789_H__ */
