#include "ha_hal_sd_card.h"
#include "ha_hal_gpio.h"

uint8_t ha_hal_sd_card_init()
{
	 FATFS FatFs;
    // Initialize the SD card hardware
    MX_SDIO_SD_Init();
    // Check if the SD card is present and initialized
    if (HAL_SD_Init(&hsd) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Mount the SD card filesystem
    if (f_mount(&FatFs, SDPath, 1) != FR_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_sd_card_mount()
{
	 FATFS FatFs;
    // Mount the SD card filesystem
    if (f_mount(&FatFs, SDPath, 1) != FR_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_sd_card_unmount()
{

    // Unmount the SD card filesystem
    if (f_mount(NULL, SDPath, 1) != FR_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}
uint8_t ha_hal_sd_card_write_file(const char *filename, const char *data)
{
    FIL file;
    UINT bytes_written;
    // Open the file for writing
    if (f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    {
        return HAL_ERROR;
    }
    // Write data to the file

    if (f_write(&file, data, strlen(data), &bytes_written) != FR_OK || bytes_written < strlen(data))
    {
        f_close(&file);
        return HAL_ERROR;
    }
    // Close the file
    if (f_close(&file) != FR_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}
uint8_t ha_hal_sd_card_read_file(const char *filename, char *buffer, uint32_t buffer_size)
{
    FIL file;
    UINT bytes_read;
    // Open the file for reading
    if (f_open(&file, filename, FA_READ) != FR_OK)
    {
        return HAL_ERROR;
    }
    // Read data from the file
    if (f_read(&file, buffer, buffer_size - 1, &bytes_read) != FR_OK)
    {
        f_close(&file);
        return HAL_ERROR;
    }
    // Null-terminate the buffer
    buffer[bytes_read] = '\0';
    // Close the file
    if (f_close(&file) != FR_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_sd_card_get_free_space(uint32_t *free_space)
{
    FATFS *fs;
    DWORD free_clusters;
    // Get free space on the SD card
    if (f_getfree(SDPath, &free_clusters, &fs) != FR_OK)
    {
        return HAL_ERROR;
    }
    // Calculate free space in bytes
    *free_space = (fs->n_fatent - 2) * fs->csize * 512; // Assuming 512 bytes per sector
    return HAL_OK;
}
