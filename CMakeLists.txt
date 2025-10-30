# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(mpu6050-solid-control C CXX ASM)

pico_sdk_init()

file(GLOB INCLUDE "include/*.c")
file(GLOB SRC "src/*.c")

add_executable(mpu6050-solid-control src/main.c src/display.c ${INCLUDE})

pico_set_program_name(mpu6050-solid-control "mpu6050-solid-control")
pico_set_program_version(mpu6050-solid-control "0.1")

pico_enable_stdio_uart(mpu6050-solid-control 1)
pico_enable_stdio_usb(mpu6050-solid-control 1)

target_link_libraries(mpu6050-solid-control
        pico_stdlib)

target_include_directories(mpu6050-solid-control PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(mpu6050-solid-control 
        hardware_clocks
        hardware_gpio
        hardware_pwm
        hardware_i2c
        hardware_adc
        pico_multicore
        )
pico_add_extra_outputs(mpu6050-solid-control)