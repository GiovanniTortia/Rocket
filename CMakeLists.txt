cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

set(PICO_SDK_PATH "/home/yogn/pico/pico-sdk")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(razzismo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(razzismo
	razzismo.cpp
)

pico_add_extra_outputs(razzismo)

add_subdirectory(Sensor_lib
)

target_link_libraries(razzismo
	pico_stdlib
	hardware_i2c
	sensors
)

pico_enable_stdio_usb(razzismo 1)
pico_enable_stdio_uart(razzismo 0)
