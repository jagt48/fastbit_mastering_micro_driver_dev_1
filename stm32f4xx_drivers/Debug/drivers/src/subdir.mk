################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f407xx_gpio_drv.c \
../drivers/src/stm32f407xx_i2c_drv.c \
../drivers/src/stm32f407xx_spi_drv.c 

OBJS += \
./drivers/src/stm32f407xx_gpio_drv.o \
./drivers/src/stm32f407xx_i2c_drv.o \
./drivers/src/stm32f407xx_spi_drv.o 

C_DEPS += \
./drivers/src/stm32f407xx_gpio_drv.d \
./drivers/src/stm32f407xx_i2c_drv.d \
./drivers/src/stm32f407xx_spi_drv.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/home/joseph/Documents/workspace.fastbit/mastering_micro_driver_dev_1/workspace/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f407xx_gpio_drv.d ./drivers/src/stm32f407xx_gpio_drv.o ./drivers/src/stm32f407xx_i2c_drv.d ./drivers/src/stm32f407xx_i2c_drv.o ./drivers/src/stm32f407xx_spi_drv.d ./drivers/src/stm32f407xx_spi_drv.o

.PHONY: clean-drivers-2f-src

