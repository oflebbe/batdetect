#ifndef __ST7789_H__
#define __ST7789_H__

#include "of_pixmap.h"
#include <hardware/spi.h>

typedef struct _ST7789_t ST7789_t;

typedef enum {
  ROTATE_0,
  ROTATE_90,
  ROTATE_270,
  ROTATE_180
} ST7789_rotation_t;


ST7789_t * ST7789_spi_create(spi_inst_t *spi_inst, int16_t width, int16_t height,
                            ST7789_rotation_t rotation, uint cs, uint reset, uint dc,
                            uint backlight, uint tx, uint sck);

ST7789_t * ST7789_parallel_create(int16_t width, int16_t height, uint cs,
                                 uint dc, uint backlight, uint wr, uint rd,
                                 uint d0);

void ST7789_backlight(ST7789_t *self, uint8_t bl);
void ST7789_hline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                  uint16_t color);
void ST7789_vline(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                  uint16_t color);
void ST7789_fill_rect(ST7789_t *self, uint16_t x, uint16_t y, uint16_t w,
                      uint16_t h, int16_t color);

void ST7789_blit_of_pixmap(ST7789_t *self, const of_pixmap_t *pixmap,
                        int16_t x, int16_t y);

void ST7789_flush(ST7789_t *self);

static inline int min( int x, int y) {
  if (x < y) { 
    return x;
  }
  return y;
}

#endif /*  __ST7789_H__ */
