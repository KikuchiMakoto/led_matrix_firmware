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

# Transfer mode selection (0: GPIO, 1: DMA, 2: PIO)
set(TRANSFER_MODE "0" CACHE STRING "Transfer mode: 0=GPIO, 1=DMA, 2=PIO")

add_executable(led_matrix_firmware
    src/main.cpp
)

# Add compile definition for transfer mode
target_compile_definitions(led_matrix_firmware PRIVATE
    TRANSFER_MODE=${TRANSFER_MODE}
)

# Base libraries
target_link_libraries(led_matrix_firmware
    pico_stdlib
    pico_multicore
    hardware_gpio
)

# Mode 1: Add SPI and DMA libraries
if(TRANSFER_MODE EQUAL 1)
    target_link_libraries(led_matrix_firmware
        hardware_spi
        hardware_dma
    )
endif()

# Mode 2: Add PIO library and generate PIO header
if(TRANSFER_MODE EQUAL 2)
    pico_generate_pio_header(led_matrix_firmware ${CMAKE_CURRENT_LIST_DIR}/src/shift_out.pio)
    target_link_libraries(led_matrix_firmware
        hardware_pio
    )
endif()

# Enable USB output, disable UART output
pico_enable_stdio_usb(led_matrix_firmware 1)
pico_enable_stdio_uart(led_matrix_firmware 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(led_matrix_firmware)
