#include "ha_hal_uart.h"

uint8_t ha_hal_uart_init(UART_HandleTypeDef *huart, uint32_t baudrate)
{
    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(huart) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_uart_transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    if (HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_uart_receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    if (HAL_UART_Receive(huart, data, size, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_uart_transmit_it(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    if (HAL_UART_Transmit_IT(huart, data, size) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_uart_receive_it(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    if (HAL_UART_Receive_IT(huart, data, size) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}
void ha_hal_uart_irq_handler(UART_HandleTypeDef *huart)
{
    HAL_UART_IRQHandler(huart);
}

uint8_t ha_hal_uart_deinit(UART_HandleTypeDef *huart)
{
    if (HAL_UART_DeInit(huart) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t ha_hal_uart_set_baudrate(UART_HandleTypeDef *huart, uint32_t baudrate)
{
    huart->Init.BaudRate = baudrate;
    if (HAL_UART_Init(huart) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}
