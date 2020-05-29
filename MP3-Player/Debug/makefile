################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MP3-Player.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MP3-Player.list \

OBJCOPY_BIN += \
MP3-Player.bin \


# All Target
all: MP3-Player.elf secondary-outputs

# Tool invocations
MP3-Player.elf: $(OBJS) $(USER_OBJS) C:\Users\Łukasz\Desktop\git\MP3-Player\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "MP3-Player.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Łukasz\Desktop\git\MP3-Player\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MP3-Player.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MP3-Player.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MP3-Player.list"
	@echo 'Finished building: $@'
	@echo ' '

MP3-Player.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MP3-Player.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
