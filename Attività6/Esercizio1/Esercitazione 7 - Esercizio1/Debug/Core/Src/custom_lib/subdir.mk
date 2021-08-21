################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/custom_lib/ADC_sensor.c \
../Core/Src/custom_lib/avg_lib.c 

OBJS += \
./Core/Src/custom_lib/ADC_sensor.o \
./Core/Src/custom_lib/avg_lib.o 

C_DEPS += \
./Core/Src/custom_lib/ADC_sensor.d \
./Core/Src/custom_lib/avg_lib.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/custom_lib/ADC_sensor.o: ../Core/Src/custom_lib/ADC_sensor.c Core/Src/custom_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/andry/STM32CubeIDE/workspace_1.6.0/Esercitazione 7 - Esercizio1/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/custom_lib/ADC_sensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/custom_lib/avg_lib.o: ../Core/Src/custom_lib/avg_lib.c Core/Src/custom_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/andry/STM32CubeIDE/workspace_1.6.0/Esercitazione 7 - Esercizio1/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/custom_lib/avg_lib.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

