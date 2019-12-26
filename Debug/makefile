################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/Src/subdir.mk
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
stm32f407xx_SPI_Driver_Test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm32f407xx_SPI_Driver_Test.list \


# All Target
all: stm32f407xx_SPI_Driver_Test.elf secondary-outputs

# Tool invocations
stm32f407xx_SPI_Driver_Test.elf: $(OBJS) $(USER_OBJS) C:\Users\erenc\OneDrive\Desktop\STM32F4_git\STM32F407xx_spi_driver\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "stm32f407xx_SPI_Driver_Test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\erenc\OneDrive\Desktop\STM32F4_git\STM32F407xx_spi_driver\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm32f407xx_SPI_Driver_Test.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm32f407xx_SPI_Driver_Test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm32f407xx_SPI_Driver_Test.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
