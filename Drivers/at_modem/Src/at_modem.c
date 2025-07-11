#include "at_modem.h"

uint8_t ha_drv_at_modem_send_command_wait_response(UART_HandleTypeDef *huart, const char *command, char *response, uint32_t timeout)
{
    uint8_t status = HAL_OK;
    char cmd_buffer[256];
    snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\r\n", command);
    if (ha_hal_uart_transmit(huart, (uint8_t *)cmd_buffer, strlen(cmd_buffer)) != HAL_OK)
    {
        return HAL_ERROR;
    }
    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout)
    {
        if (ha_hal_uart_receive(huart, (uint8_t *)response, 256) == HAL_OK)
        {
            if (strstr(response, AT_MODEM_OK) != NULL)
            {
                return HAL_OK;
            }
            else if (strstr(response, AT_MODEM_ERROR) != NULL)
            {
                return HAL_ERROR;
            }
        }
        ha_hal_hardware_delay(100); // Polling delay
    }
    return HAL_TIMEOUT;
}