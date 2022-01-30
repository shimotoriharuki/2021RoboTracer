################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AQM0802.c \
../Core/Src/HAL_SDcard_lib.c \
../Core/Src/ICM_20648.c \
../Core/Src/INA260.c \
../Core/Src/main.c \
../Core/Src/path_following.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

CPP_SRCS += \
../Core/Src/BZ.cpp \
../Core/Src/Encoder.cpp \
../Core/Src/IMU.cpp \
../Core/Src/Joystick.cpp \
../Core/Src/LED.cpp \
../Core/Src/LineSensor.cpp \
../Core/Src/LineTrace.cpp \
../Core/Src/Logger.cpp \
../Core/Src/Motor.cpp \
../Core/Src/Odometry.cpp \
../Core/Src/PathFollowing.cpp \
../Core/Src/PowerSensor.cpp \
../Core/Src/RotarySwitch.cpp \
../Core/Src/SideSensor.cpp \
../Core/Src/SystemIdentification.cpp \
../Core/Src/VelocityCtrl.cpp \
../Core/Src/wrapper.cpp 

C_DEPS += \
./Core/Src/AQM0802.d \
./Core/Src/HAL_SDcard_lib.d \
./Core/Src/ICM_20648.d \
./Core/Src/INA260.d \
./Core/Src/main.d \
./Core/Src/path_following.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/AQM0802.o \
./Core/Src/BZ.o \
./Core/Src/Encoder.o \
./Core/Src/HAL_SDcard_lib.o \
./Core/Src/ICM_20648.o \
./Core/Src/IMU.o \
./Core/Src/INA260.o \
./Core/Src/Joystick.o \
./Core/Src/LED.o \
./Core/Src/LineSensor.o \
./Core/Src/LineTrace.o \
./Core/Src/Logger.o \
./Core/Src/Motor.o \
./Core/Src/Odometry.o \
./Core/Src/PathFollowing.o \
./Core/Src/PowerSensor.o \
./Core/Src/RotarySwitch.o \
./Core/Src/SideSensor.o \
./Core/Src/SystemIdentification.o \
./Core/Src/VelocityCtrl.o \
./Core/Src/main.o \
./Core/Src/path_following.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/wrapper.o 

CPP_DEPS += \
./Core/Src/BZ.d \
./Core/Src/Encoder.d \
./Core/Src/IMU.d \
./Core/Src/Joystick.d \
./Core/Src/LED.d \
./Core/Src/LineSensor.d \
./Core/Src/LineTrace.d \
./Core/Src/Logger.d \
./Core/Src/Motor.d \
./Core/Src/Odometry.d \
./Core/Src/PathFollowing.d \
./Core/Src/PowerSensor.d \
./Core/Src/RotarySwitch.d \
./Core/Src/SideSensor.d \
./Core/Src/SystemIdentification.d \
./Core/Src/VelocityCtrl.d \
./Core/Src/wrapper.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F469xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F469xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

