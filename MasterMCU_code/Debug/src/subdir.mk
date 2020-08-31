################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/scheduler.c \
../src/tasks.c 

OBJS += \
./src/main.o \
./src/scheduler.o \
./src/tasks.o 

C_DEPS += \
./src/main.d \
./src/scheduler.d \
./src/tasks.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F4DISCOVERY -DDEBUG -I"C:/Users/Martino/workspaceAAA/masters/drivers/inc" -I"C:/Users/Martino/workspaceAAA/masters/drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Martino/workspaceAAA/masters/drivers/CMSIS" -I"C:/Users/Martino/workspaceAAA/masters/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


