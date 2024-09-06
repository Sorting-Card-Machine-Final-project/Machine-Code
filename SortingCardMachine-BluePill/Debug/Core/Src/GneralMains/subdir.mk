################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GneralMains/main_PWM.c \
../Core/Src/GneralMains/main_Stepper.c \
../Core/Src/GneralMains/main_pusher_motor.c 

OBJS += \
./Core/Src/GneralMains/main_PWM.o \
./Core/Src/GneralMains/main_Stepper.o \
./Core/Src/GneralMains/main_pusher_motor.o 

C_DEPS += \
./Core/Src/GneralMains/main_PWM.d \
./Core/Src/GneralMains/main_Stepper.d \
./Core/Src/GneralMains/main_pusher_motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/GneralMains/%.o Core/Src/GneralMains/%.su Core/Src/GneralMains/%.cyclo: ../Core/Src/GneralMains/%.c Core/Src/GneralMains/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-GneralMains

clean-Core-2f-Src-2f-GneralMains:
	-$(RM) ./Core/Src/GneralMains/main_PWM.cyclo ./Core/Src/GneralMains/main_PWM.d ./Core/Src/GneralMains/main_PWM.o ./Core/Src/GneralMains/main_PWM.su ./Core/Src/GneralMains/main_Stepper.cyclo ./Core/Src/GneralMains/main_Stepper.d ./Core/Src/GneralMains/main_Stepper.o ./Core/Src/GneralMains/main_Stepper.su ./Core/Src/GneralMains/main_pusher_motor.cyclo ./Core/Src/GneralMains/main_pusher_motor.d ./Core/Src/GneralMains/main_pusher_motor.o ./Core/Src/GneralMains/main_pusher_motor.su

.PHONY: clean-Core-2f-Src-2f-GneralMains

