################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/ACS712.c \
../Core/Src/Drivers/ADC.c \
../Core/Src/Drivers/myGPIO.c \
../Core/Src/Drivers/myNRF.c \
../Core/Src/Drivers/myVoltageRead.c 

OBJS += \
./Core/Src/Drivers/ACS712.o \
./Core/Src/Drivers/ADC.o \
./Core/Src/Drivers/myGPIO.o \
./Core/Src/Drivers/myNRF.o \
./Core/Src/Drivers/myVoltageRead.o 

C_DEPS += \
./Core/Src/Drivers/ACS712.d \
./Core/Src/Drivers/ADC.d \
./Core/Src/Drivers/myGPIO.d \
./Core/Src/Drivers/myNRF.d \
./Core/Src/Drivers/myVoltageRead.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_NUCLEO_64 -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/STM32G4xx_Nucleo -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/ACS712.cyclo ./Core/Src/Drivers/ACS712.d ./Core/Src/Drivers/ACS712.o ./Core/Src/Drivers/ACS712.su ./Core/Src/Drivers/ADC.cyclo ./Core/Src/Drivers/ADC.d ./Core/Src/Drivers/ADC.o ./Core/Src/Drivers/ADC.su ./Core/Src/Drivers/myGPIO.cyclo ./Core/Src/Drivers/myGPIO.d ./Core/Src/Drivers/myGPIO.o ./Core/Src/Drivers/myGPIO.su ./Core/Src/Drivers/myNRF.cyclo ./Core/Src/Drivers/myNRF.d ./Core/Src/Drivers/myNRF.o ./Core/Src/Drivers/myNRF.su ./Core/Src/Drivers/myVoltageRead.cyclo ./Core/Src/Drivers/myVoltageRead.d ./Core/Src/Drivers/myVoltageRead.o ./Core/Src/Drivers/myVoltageRead.su

.PHONY: clean-Core-2f-Src-2f-Drivers

