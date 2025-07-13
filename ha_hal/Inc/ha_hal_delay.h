#ifndef HAL_DELAY_H
#define HAL_DELAY_H

#include "stm32f4xx_hal.h"

#define SYSTICK_LOAD (SystemCoreClock / 1000000)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define DELAY_US(us)                                                \
    do                                                              \
    {                                                               \
        uint32_t start = SysTick->VAL;                              \
        uint32_t ticks = (us * SYSTICK_LOAD) - SYSTICK_DELAY_CALIB; \
        while ((start - SysTick->VAL) < ticks)                      \
            ;                                                       \
    } while (0)

#define DELAY_MS(ms)                      \
    do                                    \
    {                                     \
        for (uint32_t i = 0; i < ms; ++i) \
        {                                 \
            DELAY_US(1000);               \
        }                                 \
    } while (0)

void ha_hal_hardware_delay(uint8_t _ms);
void ha_hal_software_delay(uint8_t _ms);

#endif // HAL_DELAY_H
