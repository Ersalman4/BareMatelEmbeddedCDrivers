################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/DS1307.c \
../BSP/lcd.c 

OBJS += \
./BSP/DS1307.o \
./BSP/lcd.o 

C_DEPS += \
./BSP/DS1307.d \
./BSP/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o BSP/%.su BSP/%.cyclo: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"D:/low_level_drivers/stm32f411xx_drivers/BSP" -I"D:/low_level_drivers/stm32f411xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/DS1307.cyclo ./BSP/DS1307.d ./BSP/DS1307.o ./BSP/DS1307.su ./BSP/lcd.cyclo ./BSP/lcd.d ./BSP/lcd.o ./BSP/lcd.su

.PHONY: clean-BSP

