################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/at_modem/Src/at_modem.c 

OBJS += \
./Drivers/at_modem/Src/at_modem.o 

C_DEPS += \
./Drivers/at_modem/Src/at_modem.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/at_modem/Src/%.o Drivers/at_modem/Src/%.su Drivers/at_modem/Src/%.cyclo: ../Drivers/at_modem/Src/%.c Drivers/at_modem/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"E:/stm32_workspace/f407vet6/debug_rtt/Inc" -I"E:/stm32_workspace/f407vet6/ha_hal/Inc" -I"E:/stm32_workspace/f407vet6/Drivers/at_modem/Inc" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-at_modem-2f-Src

clean-Drivers-2f-at_modem-2f-Src:
	-$(RM) ./Drivers/at_modem/Src/at_modem.cyclo ./Drivers/at_modem/Src/at_modem.d ./Drivers/at_modem/Src/at_modem.o ./Drivers/at_modem/Src/at_modem.su

.PHONY: clean-Drivers-2f-at_modem-2f-Src

