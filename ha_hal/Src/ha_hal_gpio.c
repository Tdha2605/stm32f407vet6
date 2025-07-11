#include "ha_hal_gpio.h"

uint8_t ha_hal_gpio_set_level(GPIO_TypeDef *_gpio_type, uint16_t _gpio_number, GPIO_PinState _gpio_level)
{
    if (_gpio_type == NULL)
    {
        return 0; // Error: GPIO type is NULL
    }
    if (_gpio_level != GPIO_PIN_SET && _gpio_level != GPIO_PIN_RESET)
    {
        return 0; // Error: Invalid GPIO level
    }
    HAL_GPIO_WritePin(_gpio_type, _gpio_number, _gpio_level);
    return 1; // Success
}

uint8_t ha_hal_gpio_get_level(GPIO_TypeDef *_gpio_type, uint16_t _gpio_number)
{
    if (_gpio_type == NULL)
    {
        return 0; // Error: GPIO type is NULL
    }
    GPIO_PinState level = HAL_GPIO_ReadPin(_gpio_type, _gpio_number);
    if (level != GPIO_PIN_SET && level != GPIO_PIN_RESET)
    {
        return 0; // Error: Invalid GPIO level read
    }
    return (level == GPIO_PIN_SET) ? 1 : 0; // Return 1 for SET, 0 for RESET
}

uint8_t ha_hal_gpio_toggle(GPIO_TypeDef *_gpio_type, uint16_t _gpio_number)
{
    if (_gpio_type == NULL)
    {
        return 0; // Error: GPIO type is NULL
    }
    HAL_GPIO_TogglePin(_gpio_type, _gpio_number);
    return 1; // Success
}
