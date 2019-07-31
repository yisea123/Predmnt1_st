################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/STM32CubeIDE/workspace_1.0.2/PREDMNT1_V1.0.0/Middlewares/ST/STM32_MotionSP_Library/Src/MotionSP.c 

OBJS += \
./Middlewares/STM32_MotionSP_Library/MotionSP.o 

C_DEPS += \
./Middlewares/STM32_MotionSP_Library/MotionSP.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/STM32_MotionSP_Library/MotionSP.o: D:/STM32CubeIDE/workspace_1.0.2/PREDMNT1_V1.0.0/Middlewares/ST/STM32_MotionSP_Library/Src/MotionSP.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ '-D__FPU_PRESENT=1U' -DARM_MATH_CM4 -c -I../../../Inc -I../../../Patch -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/BSP/STM32F4xx-Nucleo -I../../../../../../../Drivers/BSP/X-NUCLEO-CCA02M1 -I../../../../../../../Drivers/BSP/X-NUCLEO-IDB0xA1 -I../../../../../../../Drivers/BSP/IKS01A2 -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm6dsl -I../../../../../../../Drivers/BSP/Components/lsm303agr -I../../../../../../../Drivers/BSP/Components/ism330dlc -I../../../../../../../Middlewares/ST/STM32_MetaDataManager -I../../../../../../../Middlewares/ST/BlueNRG-MS/includes -I../../../../../../../Middlewares/ST/BlueNRG-MS/utils -I../../../../../../../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../../../../../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -I../../../../../../../Middlewares/ST/STM32_MotionSP_Library/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/STM32_MotionSP_Library/MotionSP.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

