cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_cdc_spi_sender C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040_cdc_spi_sender
    src/main.c
    src/spi_sender.c
)

target_include_directories(rp2040_cdc_spi_sender PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

target_link_libraries(rp2040_cdc_spi_sender
    pico_stdlib
    hardware_uart
    hardware_gpio
)

# Use pico-sdk's built-in USB CDC serial (stdio over USB)
pico_enable_stdio_usb(rp2040_cdc_spi_sender 1)
pico_enable_stdio_uart(rp2040_cdc_spi_sender 0)

pico_add_extra_outputs(rp2040_cdc_spi_sender)
