#ifndef HAL_SDCARD_H
#define HAL_SDCARD_H

#include "ha_hal.h"
#include "fatfs.h"
#include "ffconf.h"
extern SD_HandleTypeDef hsd;
#include "string.h"

#define SDPath "backup/"

uint8_t ha_hal_sd_card_init();
uint8_t ha_hal_sd_card_mount();
uint8_t ha_hal_sd_card_unmount();
uint8_t ha_hal_sd_card_write_file(const char *filename, const char *data);
uint8_t ha_hal_sd_card_read_file(const char *filename, char *buffer, uint32_t buffer_size);
uint8_t ha_hal_sd_card_get_free_space(uint32_t *free_space);

#endif // HAL_SDCARD_H
