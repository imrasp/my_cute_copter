################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/autopilot_interface.cpp \
../src/coordinate_converter.cpp \
../src/mavlink_control.cpp \
../src/serial_port.cpp 

OBJS += \
./src/autopilot_interface.o \
./src/coordinate_converter.o \
./src/mavlink_control.o \
./src/serial_port.o 

CPP_DEPS += \
./src/autopilot_interface.d \
./src/coordinate_converter.d \
./src/mavlink_control.d \
./src/serial_port.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/rasp/workspace/My_Drone_UART/include/mavlink/v1.0" -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


