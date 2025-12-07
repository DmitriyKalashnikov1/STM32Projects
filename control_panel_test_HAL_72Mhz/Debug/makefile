################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := control_panel_test_HAL_72Mhz
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
control_panel_test_HAL_72Mhz.elf \

MAP_FILES += \
control_panel_test_HAL_72Mhz.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
control_panel_test_HAL_72Mhz.list \

OBJCOPY_HEX += \
control_panel_test_HAL_72Mhz.hex \


# All Target
all: main-build

# Main-build Target
main-build: control_panel_test_HAL_72Mhz.elf secondary-outputs

# Tool invocations
control_panel_test_HAL_72Mhz.elf control_panel_test_HAL_72Mhz.map: $(OBJS) $(USER_OBJS) F:\Creations\C++_programs\STM32Projects\control_panel_test_HAL_72Mhz\STM32F103CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "control_panel_test_HAL_72Mhz.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"F:\Creations\C++_programs\STM32Projects\control_panel_test_HAL_72Mhz\STM32F103CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="control_panel_test_HAL_72Mhz.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

control_panel_test_HAL_72Mhz.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "control_panel_test_HAL_72Mhz.list"
	@echo 'Finished building: $@'
	@echo ' '

control_panel_test_HAL_72Mhz.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "control_panel_test_HAL_72Mhz.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) control_panel_test_HAL_72Mhz.elf control_panel_test_HAL_72Mhz.hex control_panel_test_HAL_72Mhz.list control_panel_test_HAL_72Mhz.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
