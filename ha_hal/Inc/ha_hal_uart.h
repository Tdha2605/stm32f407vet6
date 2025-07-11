#ifndef HAL_UART_H
#define HAL_UART_H

#include "stm32f4xx_hal.h"

uint8_t ha_hal_uart_init(UART_HandleTypeDef *huart, uint32_t baudrate);
uint8_t ha_hal_uart_transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
uint8_t ha_hal_uart_receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
uint8_t ha_hal_uart_transmit_it(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
uint8_t ha_hal_uart_receive_it(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
void ha_hal_uart_irq_handler(UART_HandleTypeDef *huart);
uint8_t ha_hal_uart_deinit(UART_HandleTypeDef *huart);
uint8_t ha_hal_uart_set_baudrate(UART_HandleTypeDef *huart, uint32_t baudrate);

#endif