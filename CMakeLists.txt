##################################################################################
# "THE ANY BEVERAGE-WARE LICENSE" (Revision 42 - based on beer-ware
# license):
# <dev@layer128.net> wrote this file. As long as you retain this notice
# you can do whatever you want with this stuff. If we meet some day, and
# you think this stuff is worth it, you can buy me a be(ve)er(age) in
# return. (I don't like beer much.)
#
# Matthias Kleemann
##################################################################################

cmake_minimum_required(VERSION 2.8)

### TOOLCHAIN SETUP AREA #################################################
# Set any variables used in the toolchain prior project() call. In that
# case they are already set and used.
##########################################################################

##################################################################################
# tools to be used for programming the AVR
##################################################################################
set(AVR_UPLOADTOOL avrdude)
set(AVR_PROGRAMMER arduino)
set(AVR_UPLOADTOOL_PORT /dev/ttyACM0)
set(AVR_UPLOADTOOL_OPTIONS "-C/home/ggreen/apps/arduino-1.8.15/hardware/tools/avr/etc/avrdude.conf")
# AVR and fuses needs to be set
set(AVR_MCU atmega328p)
set(AVRDUDE_MCU m328p)
set(AVR_USE_E_FUSE on)
set(AVR_E_FUSE 0xfc)
set(AVR_H_FUSE 0xd9)
set(AVR_L_FUSE 0x27)

### END TOOLCHAIN SETUP AREA #############################################

##########################################################################
# name your project
##########################################################################
project(canbus-shield)

##################################################################################
# status messages
##################################################################################
message(STATUS "Current uploadtool is: ${AVR_UPLOADTOOL}")
message(STATUS "Current programmer is: ${AVR_PROGRAMMER}")
message(STATUS "Current upload port is: ${AVR_UPLOADTOOL_PORT}")
message(STATUS "Current uploadtool options are: ${AVR_UPLOADTOOL_OPTIONS}")
message(STATUS "Current MCU is set to: ${AVR_MCU}")
message(STATUS "Current Programmer MCU is set to: ${AVRDUDE_MCU}")
message(STATUS "Current use E fuse is set to: ${AVR_USE_E_FUSE}")
message(STATUS "Current E_FUSE is set to: ${AVR_E_FUSE}")
message(STATUS "Current H_FUSE is set to: ${AVR_H_FUSE}")
message(STATUS "Current L_FUSE is set to: ${AVR_L_FUSE}")

##################################################################################
# set build type, if not already set at cmake command line
##################################################################################
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

##################################################################################
# needs to be defined for AVR toolchain
##################################################################################
set(MCU_SPEED "16000000UL")

##################################################################################
# some cmake cross-compile necessities
##################################################################################
if(DEFINED ENV{AVR_FIND_ROOT_PATH})
    set(CMAKE_FIND_ROOT_PATH $ENV{AVR_FIND_ROOT_PATH})
else(DEFINED ENV{AVR_FIND_ROOT_PATH})
    if(EXISTS "/opt/local/avr")
      set(CMAKE_FIND_ROOT_PATH "/opt/local/avr")
    elseif(EXISTS "/usr/avr")
      set(CMAKE_FIND_ROOT_PATH "/usr/avr")
    elseif(EXISTS "/usr/lib/avr")
      set(CMAKE_FIND_ROOT_PATH "/usr/lib/avr")
    elseif(EXISTS "/usr/local/CrossPack-AVR")
      set(CMAKE_FIND_ROOT_PATH "/usr/local/CrossPack-AVR")
    else(EXISTS "/opt/local/avr")
      message(FATAL_ERROR "Please set AVR_FIND_ROOT_PATH in your environment.")
    endif(EXISTS "/opt/local/avr")
endif(DEFINED ENV{AVR_FIND_ROOT_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# not added automatically, since CMAKE_SYSTEM_NAME is "generic"
set(CMAKE_SYSTEM_INCLUDE_PATH "${CMAKE_FIND_ROOT_PATH}/include")
set(CMAKE_SYSTEM_LIBRARY_PATH "${CMAKE_FIND_ROOT_PATH}/lib")

##################################################################################
# status messages for generating
##################################################################################
message(STATUS "Set CMAKE_FIND_ROOT_PATH to ${CMAKE_FIND_ROOT_PATH}")
message(STATUS "Set CMAKE_SYSTEM_INCLUDE_PATH to ${CMAKE_SYSTEM_INCLUDE_PATH}")
message(STATUS "Set CMAKE_SYSTEM_LIBRARY_PATH to ${CMAKE_SYSTEM_LIBRARY_PATH}")

##################################################################################
# set compiler options for build types
##################################################################################
if(CMAKE_BUILD_TYPE MATCHES Release)
   set(CMAKE_C_FLAGS_RELEASE "-Os")
   set(CMAKE_CXX_FLAGS_RELEASE "-Os")
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
   set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
   set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
endif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)

if(CMAKE_BUILD_TYPE MATCHES Debug)
   set(CMAKE_C_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
   set(CMAKE_CXX_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
endif(CMAKE_BUILD_TYPE MATCHES Debug)

##################################################################################
# compiler options for all build types
##################################################################################
add_definitions("-DF_CPU=${MCU_SPEED}")
add_definitions("-fpack-struct")
add_definitions("-fshort-enums")
add_definitions("-Wall")
#add_definitions("-Werror")
# http://gcc.gnu.org/onlinedocs/gcc-4.8.2/gcc/Alternate-Keywords.html#Alternate-Keywords
# [...]-pedantic and other options cause warnings for many GNU C extensions. You can prevent such warnings within
# one expression by writing __extension__ before the expression. __extension__ has no effect aside from this.[...] 
add_definitions("-pedantic")
add_definitions("-pedantic-errors")
add_definitions("-funsigned-char")
add_definitions("-funsigned-bitfields")
add_definitions("-ffunction-sections")
add_definitions("-c")
add_definitions("-std=gnu99")

##########################################################################
# include search paths
##########################################################################
include_directories(
	"${CMAKE_SOURCE_DIR}"
	"${CMAKE_SOURCE_DIR}/../libs/avr_drivers/canlibrary"
	"${CMAKE_SOURCE_DIR}/../libs/avr_drivers/drivers"
	"${CMAKE_SOURCE_DIR}/../libs/avr_drivers/i2cmaster"
	"${CMAKE_SOURCE_DIR}/../libs/avr_drivers/utils"
#	"${CMAKE_SOURCE_DIR}/../libs/libcanard"
#	"${CMAKE_SOURCE_DIR}/../libs/libcanard/drivers/avr"
#	"${CMAKE_SOURCE_DIR}/../libs/libcanard/drivers/avr/avr-can-lib"
)

##################################################################################
# add AVR executable
##################################################################################
add_avr_executable(
   canbus-shield
   canbus.c
   gpio.c
   gpio.h
   globals.h
   defs.h
)

##################################################################################
# link library to executable
# NOTE: It needs to be the elf target.
##################################################################################

set(CAN_LIB "${CMAKE_AVRLIBS_PATH}/canlibrary/libavrcanlib-${AVR_MCU}.a")
message(STATUS "avrcanlib: ${CAN_LIB}")
set(DRIVER_LIB "${CMAKE_AVRLIBS_PATH}/drivers/libavrdrivers-${AVR_MCU}.a")
message(STATUS "avrdriverslib: ${DRIVER_LIB}")
set(I2CMASTER_LIB "${CMAKE_AVRLIBS_PATH}/i2cmaster/libavri2cmaster-${AVR_MCU}.a")
message(STATUS "avri2cmaster: ${I2CMASTER_LIB}")
set(UTILS_LIB "${CMAKE_AVRLIBS_PATH}/utils/libavrutils-${AVR_MCU}.a")
message(STATUS "avrutilslib: ${UTILS_LIB}")

avr_target_link_libraries(
	canbus-shield
	${CAN_LIB}
	${DRIVER_LIB}
	${I2CMASTER_LIB}
	${UTILS_LIB}
)

