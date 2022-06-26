################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LTC6811/Src/LTC6811.cpp \
../LTC6811/Src/LTC681x.cpp 

OBJS += \
./LTC6811/Src/LTC6811.o \
./LTC6811/Src/LTC681x.o 

CPP_DEPS += \
./LTC6811/Src/LTC6811.d \
./LTC6811/Src/LTC681x.d 


# Each subdirectory must supply rules for building sources it contributes
LTC6811/Src/%.o: ../LTC6811/Src/%.cpp LTC6811/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LTC6811-2f-Src

clean-LTC6811-2f-Src:
	-$(RM) ./LTC6811/Src/LTC6811.d ./LTC6811/Src/LTC6811.o ./LTC6811/Src/LTC681x.d ./LTC6811/Src/LTC681x.o

.PHONY: clean-LTC6811-2f-Src

