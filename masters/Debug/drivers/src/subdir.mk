################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f407xx_RCC_driver.c \
../drivers/src/stm32f407xx_gpio_driver.c \
../drivers/src/stm32f407xx_spi_driver.c \
../drivers/src/stm32f4xx_adc.c \
../drivers/src/stm32f4xx_rcc.c 

OBJS += \
./drivers/src/stm32f407xx_RCC_driver.o \
./drivers/src/stm32f407xx_gpio_driver.o \
./drivers/src/stm32f407xx_spi_driver.o \
./drivers/src/stm32f4xx_adc.o \
./drivers/src/stm32f4xx_rcc.o 

C_DEPS += \
./drivers/src/stm32f407xx_RCC_driver.d \
./drivers/src/stm32f407xx_gpio_driver.d \
./drivers/src/stm32f407xx_spi_driver.d \
./drivers/src/stm32f4xx_adc.d \
./drivers/src/stm32f4xx_rcc.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F4DISCOVERY -DDEBUG -I"C:/Users/Martino/workspaceAAA/masters/drivers/inc" -I"C:/Users/Martino/workspaceAAA/masters/drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Martino/workspaceAAA/masters/drivers/CMSIS" -I"C:/Users/Martino/workspaceAAA/masters/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


