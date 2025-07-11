#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include "stm32f4xx_hal.h"

uint8_t ha_hal_gpio_set_level(GPIO_TypeDef *_gpio_type, uint16_t _gpio_number, GPIO_PinState _gpio_level);
uint8_t ha_hal_gpio_get_level(GPIO_TypeDef *_gpio_type, uint16_t _gpio_number);
uint8_t ha_hal_gpio_toggle(GPIO_TypeDef *_gpio_type, uint16_t _gpio_number);

#endif // HAL_GPIO_H