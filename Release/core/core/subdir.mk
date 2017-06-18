################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/CDC.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial0.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial1.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial2.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial3.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/IPAddress.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/PluggableUSB.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/Print.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/Stream.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/Tone.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/USBCore.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/WMath.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/WString.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/abi.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/main.cpp \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/new.cpp 

S_UPPER_SRCS += \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_pulse.S 

C_SRCS += \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/WInterrupts.c \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/hooks.c \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring.c \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_analog.c \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_digital.c \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_pulse.c \
/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_shift.c 

C_DEPS += \
./core/WInterrupts.c.d \
./core/hooks.c.d \
./core/wiring.c.d \
./core/wiring_analog.c.d \
./core/wiring_digital.c.d \
./core/wiring_pulse.c.d \
./core/wiring_shift.c.d 

AR_OBJ += \
./core/CDC.cpp.o \
./core/HardwareSerial.cpp.o \
./core/HardwareSerial0.cpp.o \
./core/HardwareSerial1.cpp.o \
./core/HardwareSerial2.cpp.o \
./core/HardwareSerial3.cpp.o \
./core/IPAddress.cpp.o \
./core/PluggableUSB.cpp.o \
./core/Print.cpp.o \
./core/Stream.cpp.o \
./core/Tone.cpp.o \
./core/USBCore.cpp.o \
./core/WInterrupts.c.o \
./core/WMath.cpp.o \
./core/WString.cpp.o \
./core/abi.cpp.o \
./core/hooks.c.o \
./core/main.cpp.o \
./core/new.cpp.o \
./core/wiring.c.o \
./core/wiring_analog.c.o \
./core/wiring_digital.c.o \
./core/wiring_pulse.S.o \
./core/wiring_pulse.c.o \
./core/wiring_shift.c.o 

S_UPPER_DEPS += \
./core/wiring_pulse.S.d 

CPP_DEPS += \
./core/CDC.cpp.d \
./core/HardwareSerial.cpp.d \
./core/HardwareSerial0.cpp.d \
./core/HardwareSerial1.cpp.d \
./core/HardwareSerial2.cpp.d \
./core/HardwareSerial3.cpp.d \
./core/IPAddress.cpp.d \
./core/PluggableUSB.cpp.d \
./core/Print.cpp.d \
./core/Stream.cpp.d \
./core/Tone.cpp.d \
./core/USBCore.cpp.d \
./core/WMath.cpp.d \
./core/WString.cpp.d \
./core/abi.cpp.d \
./core/main.cpp.d \
./core/new.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
core/CDC.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/CDC.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/HardwareSerial.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/HardwareSerial0.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial0.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/HardwareSerial1.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial1.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/HardwareSerial2.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial2.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/HardwareSerial3.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/HardwareSerial3.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/IPAddress.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/IPAddress.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/PluggableUSB.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/PluggableUSB.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/Print.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/Print.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/Stream.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/Stream.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/Tone.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/Tone.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/USBCore.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/USBCore.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/WInterrupts.c.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/WInterrupts.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/WMath.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/WMath.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/WString.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/WString.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/abi.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/abi.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/hooks.c.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/hooks.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/main.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/main.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/new.cpp.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/new.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-g++" -c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/wiring.c.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/wiring_analog.c.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_analog.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/wiring_digital.c.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_digital.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/wiring_pulse.S.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_pulse.S
	@echo 'Building file: $<'
	@echo 'Starting S compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -x assembler-with-cpp -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/wiring_pulse.c.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_pulse.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '

core/wiring_shift.c.o: /home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino/wiring_shift.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/kaktus/sloeber/arduinoPlugin/tools/arduino/avr-gcc/4.9.2-atmel3.5.3-arduino2/bin/avr-gcc" -c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR   -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/cores/arduino" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/variants/eightanaloginputs" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM" -I"/home/kaktus/sloeber/arduinoPlugin/packages/arduino/hardware/avr/1.6.15/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 "$<" -o "$@"  -Wall
	@echo 'Finished building: $<'
	@echo ' '


