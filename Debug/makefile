################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include ST_API/src/subdir.mk
-include Drivers/Utils/STM32/src/subdir.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Drivers/S2LP_Middleware/STM32/src/FEM/subdir.mk
-include Drivers/S2LP_Middleware/STM32/src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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


BUILD_ARTIFACT_NAME := HT32SX-Generic-Push-Button
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
HT32SX-Generic-Push-Button.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
HT32SX-Generic-Push-Button.list \

OBJCOPY_BIN += \
HT32SX-Generic-Push-Button.bin \


# All Target
all: main-build

# Main-build Target
main-build: HT32SX-Generic-Push-Button.elf secondary-outputs

# Tool invocations
HT32SX-Generic-Push-Button.elf: $(OBJS) $(USER_OBJS) C:\Users\rafas\Downloads\HT32SX_EXAMPLES\Smart_Tree\STM32L052T8YX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "HT32SX-Generic-Push-Button.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\rafas\Downloads\HT32SX_EXAMPLES\Smart_Tree\STM32L052T8YX_FLASH.ld" --specs=nosys.specs -Wl,-Map="HT32SX-Generic-Push-Button.map" -Wl,--gc-sections -static -Wl,--start-group -l:ST_RETRIEVER_LIB_CM0+_GCC.a -l:SFX_LIB_MON_CortexM0+.a -Wl,--end-group -L../Sigfox_libs/bin -L../ST_API/bin --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

HT32SX-Generic-Push-Button.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "HT32SX-Generic-Push-Button.list"
	@echo 'Finished building: $@'
	@echo ' '

HT32SX-Generic-Push-Button.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "HT32SX-Generic-Push-Button.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
