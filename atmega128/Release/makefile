################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
atmega128.lss \

FLASH_IMAGE += \
atmega128.hex \

EEPROM_IMAGE += \
atmega128.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: atmega128.elf secondary-outputs

# Tool invocations
atmega128.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,atmega128.map -mmcu=atmega1281 -o "atmega128.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

atmega128.lss: atmega128.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S atmega128.elf  >"atmega128.lss"
	@echo 'Finished building: $@'
	@echo ' '

atmega128.hex: atmega128.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex atmega128.elf  "atmega128.hex"
	@echo 'Finished building: $@'
	@echo ' '

atmega128.eep: atmega128.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex atmega128.elf  "atmega128.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: atmega128.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega1281 atmega128.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) atmega128.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
