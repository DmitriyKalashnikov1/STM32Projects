################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/NRF24.c \
../Core/Src/Drivers/myGPIO.c \
../Core/Src/Drivers/myNRF.c 

OBJS += \
./Core/Src/Drivers/NRF24.o \
./Core/Src/Drivers/myGPIO.o \
./Core/Src/Drivers/myNRF.o 

C_DEPS += \
./Core/Src/Drivers/NRF24.d \
./Core/Src/Drivers/myGPIO.d \
./Core/Src/Drivers/myNRF.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/NRF24.cyclo ./Core/Src/Drivers/NRF24.d ./Core/Src/Drivers/NRF24.o ./Core/Src/Drivers/NRF24.su ./Core/Src/Drivers/myGPIO.cyclo ./Core/Src/Drivers/myGPIO.d ./Core/Src/Drivers/myGPIO.o ./Core/Src/Drivers/myGPIO.su ./Core/Src/Drivers/myNRF.cyclo ./Core/Src/Drivers/myNRF.d ./Core/Src/Drivers/myNRF.o ./Core/Src/Drivers/myNRF.su

.PHONY: clean-Core-2f-Src-2f-Drivers

