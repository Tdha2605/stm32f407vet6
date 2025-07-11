################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ha_hal/Src/ha_hal_delay.c \
../ha_hal/Src/ha_hal_gpio.c \
../ha_hal/Src/ha_hal_uart.c 

OBJS += \
./ha_hal/Src/ha_hal_delay.o \
./ha_hal/Src/ha_hal_gpio.o \
./ha_hal/Src/ha_hal_uart.o 

C_DEPS += \
./ha_hal/Src/ha_hal_delay.d \
./ha_hal/Src/ha_hal_gpio.d \
./ha_hal/Src/ha_hal_uart.d 


# Each subdirectory must supply rules for building sources it contributes
ha_hal/Src/%.o ha_hal/Src/%.su ha_hal/Src/%.cyclo: ../ha_hal/Src/%.c ha_hal/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"E:/stm32_workspace/f407vet6/debug_rtt/Inc" -I"E:/stm32_workspace/f407vet6/ha_hal/Inc" -I"E:/stm32_workspace/f407vet6/Drivers/at_modem/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ha_hal-2f-Src

clean-ha_hal-2f-Src:
	-$(RM) ./ha_hal/Src/ha_hal_delay.cyclo ./ha_hal/Src/ha_hal_delay.d ./ha_hal/Src/ha_hal_delay.o ./ha_hal/Src/ha_hal_delay.su ./ha_hal/Src/ha_hal_gpio.cyclo ./ha_hal/Src/ha_hal_gpio.d ./ha_hal/Src/ha_hal_gpio.o ./ha_hal/Src/ha_hal_gpio.su ./ha_hal/Src/ha_hal_uart.cyclo ./ha_hal/Src/ha_hal_uart.d ./ha_hal/Src/ha_hal_uart.o ./ha_hal/Src/ha_hal_uart.su

.PHONY: clean-ha_hal-2f-Src

