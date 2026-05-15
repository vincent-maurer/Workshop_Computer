cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(pico_sdk_import.cmake)
project(modal C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(modal)

target_sources(modal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/modal.cpp
    ${CMAKE_CURRENT_LIST_DIR}/resources_q15.cpp
    ${CMAKE_CURRENT_LIST_DIR}/samples_flash.cpp
)

# Include the current directory and the ComputerCard library
target_include_directories(modal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/ComputerCard
)

# Essential: prevents reset boot failures
target_compile_definitions(modal PUBLIC PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)
target_compile_definitions(modal PRIVATE PICO_DEFAULT_UART_BAUD_RATE=115200)

# Run all code from RAM to eliminate flash cache jitter
pico_set_binary_type(modal copy_to_ram)

pico_enable_stdio_uart(modal 0)
pico_enable_stdio_usb(modal 0)
pico_add_extra_outputs(modal)

# Compiler warnings: catch accidental float promotion, general warnings
target_compile_options(modal PRIVATE -Wdouble-promotion -Wfloat-conversion -Wall -Wextra)
target_link_options(modal PRIVATE -Wl,--print-memory-usage)

target_link_libraries(modal
    pico_unique_id
    pico_stdlib
    hardware_dma
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_spi
    hardware_timer
    pico_multicore
    hardware_clocks
)
