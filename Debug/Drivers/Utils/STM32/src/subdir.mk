################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Utils/STM32/src/SDK_UTILS_Flash.c 

OBJS += \
./Drivers/Utils/STM32/src/SDK_UTILS_Flash.o 

C_DEPS += \
./Drivers/Utils/STM32/src/SDK_UTILS_Flash.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Utils/STM32/src/SDK_UTILS_Flash.o: ../Drivers/Utils/STM32/src/SDK_UTILS_Flash.c Drivers/Utils/STM32/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_STM32L0XX_NUCLEO -DUSE_HAL_DRIVER -DUSE_SYSTICK_DELAY -DFOR_ALL -DSTM32L052xx -DUSE_FLASH -DDEBUG -c -I../ST_API/inc -I../Sigfox_libs/inc -I../Drivers/Utils/STM32/inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/S2LP_Middleware/STM32/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Utils/STM32/src/SDK_UTILS_Flash.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

