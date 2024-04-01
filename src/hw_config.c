
/* hw_config.c
Copyright 2021 Carl John Kugler III

Licensed under the Apache License, Version 2.0 (the License); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an AS IS BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
*/
/*
This file should be tailored to match the hardware design.

See
https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/tree/main#customizing-for-the-hardware-configuration
*/

#include "hw_config.h"

/* Configuration of RP2040 hardware SDIO object */
static sd_sdio_if_t sdio_if = {
    .D0_gpio = 2,
    .SDIO_PIO = pio0,
    .CMD_gpio = 1,
    .baud_rate = 170 * 1000 * 1000 / 10,
    .use_exclusive_DMA_IRQ_handler = true
};


/* Configuration of the SD Card socket object */
static sd_card_t sd_card = {
    .type = SD_IF_SDIO,
    .sdio_if_p = &sdio_if // Pointer to the SPI interface driving this card
};

/* ********************************************************************** */

size_t sd_get_num() { return 1; }

sd_card_t *sd_get_by_num(size_t num) {
  if (0 == num) {
    return &sd_card;
  } else {
    return NULL;
  }
}

/* [] END OF FILE */
