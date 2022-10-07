################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c \
../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c 

OBJS += \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.o \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.o 

C_DEPS += \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.d \
./Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32L4xx_HAL_Driver/Src/%.o: ../Drivers/STM32L4xx_HAL_Driver/Src/%.c Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4A6xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=4000000 -DEXTERNALSAI1_CLOCK_VALUE=2097000 -DEXTERNALSAI2_CLOCK_VALUE=2097000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

