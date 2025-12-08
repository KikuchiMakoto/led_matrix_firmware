cmake_minimum_required(VERSION 3.13)

# Set Pico SDK path to submodule
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(led_matrix_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(led_matrix_firmware
    src/main.cpp
)

target_link_libraries(led_matrix_firmware
    pico_stdlib
    pico_multicore
    hardware_gpio
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(led_matrix_firmware 1)
pico_enable_stdio_uart(led_matrix_firmware 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(led_matrix_firmware)
