################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ST_API/src/mcu_api_stm32.c \
../ST_API/src/nvm_api.c \
../ST_API/src/rf_api.c 

OBJS += \
./ST_API/src/mcu_api_stm32.o \
./ST_API/src/nvm_api.o \
./ST_API/src/rf_api.o 

C_DEPS += \
./ST_API/src/mcu_api_stm32.d \
./ST_API/src/nvm_api.d \
./ST_API/src/rf_api.d 


# Each subdirectory must supply rules for building sources it contributes
ST_API/src/mcu_api_stm32.o: ../ST_API/src/mcu_api_stm32.c ST_API/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_STM32L0XX_NUCLEO -DUSE_HAL_DRIVER -DUSE_SYSTICK_DELAY -DFOR_ALL -DSTM32L052xx -DUSE_FLASH -DDEBUG -c -I../ST_API/inc -I../Sigfox_libs/inc -I../Drivers/Utils/STM32/inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/S2LP_Middleware/STM32/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ST_API/src/mcu_api_stm32.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ST_API/src/nvm_api.o: ../ST_API/src/nvm_api.c ST_API/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_STM32L0XX_NUCLEO -DUSE_HAL_DRIVER -DUSE_SYSTICK_DELAY -DFOR_ALL -DSTM32L052xx -DUSE_FLASH -DDEBUG -c -I../ST_API/inc -I../Sigfox_libs/inc -I../Drivers/Utils/STM32/inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/S2LP_Middleware/STM32/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ST_API/src/nvm_api.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ST_API/src/rf_api.o: ../ST_API/src/rf_api.c ST_API/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_STM32L0XX_NUCLEO -DUSE_HAL_DRIVER -DUSE_SYSTICK_DELAY -DFOR_ALL -DSTM32L052xx -DUSE_FLASH -DDEBUG -c -I../ST_API/inc -I../Sigfox_libs/inc -I../Drivers/Utils/STM32/inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/S2LP_Middleware/STM32/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ST_API/src/rf_api.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

