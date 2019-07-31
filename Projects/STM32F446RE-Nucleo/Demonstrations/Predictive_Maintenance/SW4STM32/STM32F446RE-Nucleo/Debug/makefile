################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include PredictiveMaintenance/User/Patch/subdir.mk
-include PredictiveMaintenance/User/subdir.mk
-include PredictiveMaintenance/SW4STM32/subdir.mk
-include Middlewares/STM32_MotionSP_Library/subdir.mk
-include Middlewares/MetaDataManager/subdir.mk
-include Middlewares/BlueNRG-MS/Utils/subdir.mk
-include Middlewares/BlueNRG-MS/HCI/hci_tl_patterns/Basic/subdir.mk
-include Middlewares/BlueNRG-MS/HCI/controller/subdir.mk
-include Middlewares/BlueNRG-MS/HCI/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/X-NUCLEO-IKS01A2/subdir.mk
-include Drivers/BSP/X-NUCLEO-CCA02M1/subdir.mk
-include Drivers/BSP/STM32F4xx-Nucleo/subdir.mk
-include Drivers/BSP/Components/lsm6dsl/subdir.mk
-include Drivers/BSP/Components/lsm303agr/subdir.mk
-include Drivers/BSP/Components/lps22hb/subdir.mk
-include Drivers/BSP/Components/ism330dlc/subdir.mk
-include Drivers/BSP/Components/hts221/subdir.mk
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
NUCLEO-F446RE_PredictiveMaintenance.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
NUCLEO-F446RE_PredictiveMaintenance.list \

OBJCOPY_BIN += \
NUCLEO-F446RE_PredictiveMaintenance.bin \


# All Target
all: NUCLEO-F446RE_PredictiveMaintenance.elf secondary-outputs

# Tool invocations
NUCLEO-F446RE_PredictiveMaintenance.elf: $(OBJS) $(USER_OBJS) ../LinkerScript.ld
	arm-none-eabi-gcc -o "NUCLEO-F446RE_PredictiveMaintenance.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../LinkerScript.ld" --specs=nosys.specs -Wl,-Map="FP-IND-PREDMNT1 - PredictiveMaintenance.map" -Wl,--gc-sections -static -L../../../../../../../Drivers/CMSIS/Lib/GCC -L../../../../../../../Middlewares/ST/STM32_Audio/Addons/PDM/Lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

NUCLEO-F446RE_PredictiveMaintenance.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "NUCLEO-F446RE_PredictiveMaintenance.list"
	@echo 'Finished building: $@'
	@echo ' '

NUCLEO-F446RE_PredictiveMaintenance.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "NUCLEO-F446RE_PredictiveMaintenance.bin"
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
