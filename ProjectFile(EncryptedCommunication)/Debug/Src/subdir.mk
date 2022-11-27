################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/OLEDScreenDriver.c \
../Src/cryptosystem.c \
../Src/fonts.c \
../Src/freertos.c \
../Src/keypadDriver.c \
../Src/main.c \
../Src/ssd1306.c \
../Src/stm32l4s5i_iot01.c \
../Src/stm32l4s5i_iot01_hsensor.c \
../Src/stm32l4s5i_iot01_tsensor.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_hal_timebase_tim.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32l4xx.c \
../Src/testFunctionalities.c 

OBJS += \
./Src/OLEDScreenDriver.o \
./Src/cryptosystem.o \
./Src/fonts.o \
./Src/freertos.o \
./Src/keypadDriver.o \
./Src/main.o \
./Src/ssd1306.o \
./Src/stm32l4s5i_iot01.o \
./Src/stm32l4s5i_iot01_hsensor.o \
./Src/stm32l4s5i_iot01_tsensor.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_hal_timebase_tim.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32l4xx.o \
./Src/testFunctionalities.o 

C_DEPS += \
./Src/OLEDScreenDriver.d \
./Src/cryptosystem.d \
./Src/fonts.d \
./Src/freertos.d \
./Src/keypadDriver.d \
./Src/main.d \
./Src/ssd1306.d \
./Src/stm32l4s5i_iot01.d \
./Src/stm32l4s5i_iot01_hsensor.d \
./Src/stm32l4s5i_iot01_tsensor.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_hal_timebase_tim.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32l4xx.d \
./Src/testFunctionalities.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/OLEDScreenDriver.d ./Src/OLEDScreenDriver.o ./Src/OLEDScreenDriver.su ./Src/cryptosystem.d ./Src/cryptosystem.o ./Src/cryptosystem.su ./Src/fonts.d ./Src/fonts.o ./Src/fonts.su ./Src/freertos.d ./Src/freertos.o ./Src/freertos.su ./Src/keypadDriver.d ./Src/keypadDriver.o ./Src/keypadDriver.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/ssd1306.d ./Src/ssd1306.o ./Src/ssd1306.su ./Src/stm32l4s5i_iot01.d ./Src/stm32l4s5i_iot01.o ./Src/stm32l4s5i_iot01.su ./Src/stm32l4s5i_iot01_hsensor.d ./Src/stm32l4s5i_iot01_hsensor.o ./Src/stm32l4s5i_iot01_hsensor.su ./Src/stm32l4s5i_iot01_tsensor.d ./Src/stm32l4s5i_iot01_tsensor.o ./Src/stm32l4s5i_iot01_tsensor.su ./Src/stm32l4xx_hal_msp.d ./Src/stm32l4xx_hal_msp.o ./Src/stm32l4xx_hal_msp.su ./Src/stm32l4xx_hal_timebase_tim.d ./Src/stm32l4xx_hal_timebase_tim.o ./Src/stm32l4xx_hal_timebase_tim.su ./Src/stm32l4xx_it.d ./Src/stm32l4xx_it.o ./Src/stm32l4xx_it.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32l4xx.d ./Src/system_stm32l4xx.o ./Src/system_stm32l4xx.su ./Src/testFunctionalities.d ./Src/testFunctionalities.o ./Src/testFunctionalities.su

.PHONY: clean-Src

