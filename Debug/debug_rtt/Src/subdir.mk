################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../debug_rtt/Src/SEGGER_RTT.c 

OBJS += \
./debug_rtt/Src/SEGGER_RTT.o 

C_DEPS += \
./debug_rtt/Src/SEGGER_RTT.d 


# Each subdirectory must supply rules for building sources it contributes
debug_rtt/Src/%.o debug_rtt/Src/%.su debug_rtt/Src/%.cyclo: ../debug_rtt/Src/%.c debug_rtt/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"E:/stm32_workspace/f407vet6/debug_rtt/Inc" -I"E:/stm32_workspace/f407vet6/ha_hal/Inc" -I"E:/stm32_workspace/f407vet6/Drivers/at_modem/Inc" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-debug_rtt-2f-Src

clean-debug_rtt-2f-Src:
	-$(RM) ./debug_rtt/Src/SEGGER_RTT.cyclo ./debug_rtt/Src/SEGGER_RTT.d ./debug_rtt/Src/SEGGER_RTT.o ./debug_rtt/Src/SEGGER_RTT.su

.PHONY: clean-debug_rtt-2f-Src

