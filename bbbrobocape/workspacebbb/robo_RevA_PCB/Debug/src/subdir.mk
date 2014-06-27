################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BBBGPIO.cpp \
../src/MCP23S17.cpp \
../src/Main.cpp \
../src/PWM.cpp \
../src/ad5293.cpp \
../src/ad9910dds.cpp \
../src/max1300.cpp \
../src/motorcontrol.cpp \
../src/spi.cpp 

CC_SRCS += \
../src/core.cc \
../src/smart_buffer.cc \
../src/socket.cc \
../src/socket_group.cc \
../src/util.cc 

OBJS += \
./src/BBBGPIO.o \
./src/MCP23S17.o \
./src/Main.o \
./src/PWM.o \
./src/ad5293.o \
./src/ad9910dds.o \
./src/core.o \
./src/max1300.o \
./src/motorcontrol.o \
./src/smart_buffer.o \
./src/socket.o \
./src/socket_group.o \
./src/spi.o \
./src/util.o 

CC_DEPS += \
./src/core.d \
./src/smart_buffer.d \
./src/socket.d \
./src/socket_group.d \
./src/util.d 

CPP_DEPS += \
./src/BBBGPIO.d \
./src/MCP23S17.d \
./src/Main.d \
./src/PWM.d \
./src/ad5293.d \
./src/ad9910dds.d \
./src/max1300.d \
./src/motorcontrol.d \
./src/spi.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabi-g++ -I/usr/arm-linux-gnueabi/include/c++/4.7.2 -I/home/chris/workspace/robo/netlink -I/usr/arm-linux-gnueabi/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabi-g++ -I/usr/arm-linux-gnueabi/include/c++/4.7.2 -I/home/chris/workspace/robo/netlink -I/usr/arm-linux-gnueabi/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


