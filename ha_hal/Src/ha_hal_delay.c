#include "ha_hal_delay.h"

void ha_hal_hardware_delay(uint8_t _ms)
{
	HAL_Delay(_ms);
}

void ha_hal_software_delay(uint8_t _ms)
{
	uint32_t start = HAL_GetTick();
	while ((HAL_GetTick() - start) < _ms)
	{
	}
}