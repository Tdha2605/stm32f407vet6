#ifndef DRIVER_AT_MODEM_H
#define DRIVER_AT_MODEM_H

#include "ha_hal_uart.h"
#include "ha_hal_delay.h"
#include "stdint.h"

#include "stm32f4xx_hal.h"
#define AT_MODEM_OK "OK"
#define AT_MODEM_ERROR "ERROR"

uint8_t ha_drv_at_modem_send_command_wait_response(UART_HandleTypeDef *huart, const char *command, char *response, uint32_t timeout);

#endif // DRIVER_AT_MODEM_H
