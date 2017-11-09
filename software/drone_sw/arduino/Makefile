# This sample Makefile, explains how you can compile plain AVR C file.
#
# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile



#nano ide 1.6
MCU = atmega328p
BOARD_TAG    = nano
BOARD_SUB    = atmega328p
#ISP_PROG     = usbasp
ARDUINO = /usr/share/arduino
ARDUINO_DIR = $(ARDUINO)
ARDMK_VENDOR = archlinux-arduino
F_CPU = 16000000L
AVRDUDE_ARD_BAUDRATE = 57600
CFLAGS_STD = -std=gnu11 -lstdc++
CXXFLAGS_STD  = -std=c++11 -lstdc++

CXXFLAGS += 		-Wall 			\
			-Wpointer-arith 	\
			-ffunction-sections 	\
			-fdata-sections		\
			-Wno-unused-variable 	\
			-Wunused-variable	\
			-Wl,--gc-sections	\
			-Wno-deprecated 	\
			-Wno-deprecated-declarations \


OBJDIR = build/


LOCAL_C_SRCS += $(wildcard source/*.c)
LOCAL_C_SRCS += $(wildcard libs/*.c)
LOCAL_C_SRCS += $(wildcard libs/*/*.c)
LOCAL_CPP_SRCS += $(wildcard source/*.cpp)
LOCAL_CPP_SRCS += $(wildcard libs/*.cpp)
LOCAL_CPP_SRCS += $(wildcard libs/*/*.cpp)

USER_LIB_PATH = libs

include /usr/share/arduino/Arduino.mk
