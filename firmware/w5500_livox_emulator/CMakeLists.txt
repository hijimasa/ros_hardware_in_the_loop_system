cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(w5500_livox_emulator C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(w5500_livox_emulator
    src/main.c
    src/cdc_receiver.c
    src/livox_protocol.c
    src/w5500_spi.c
)

target_include_directories(w5500_livox_emulator PRIVATE
    src/
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

# UART0 (GP0=TX, GP1=RX) for debug output
target_compile_definitions(w5500_livox_emulator PRIVATE
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=0
    PICO_DEFAULT_UART_RX_PIN=1
)

target_link_libraries(w5500_livox_emulator
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_dma
)

# USB CDC for data reception from PC
pico_enable_stdio_usb(w5500_livox_emulator 1)
# UART for debug output
pico_enable_stdio_uart(w5500_livox_emulator 0)

# Generate .uf2 for drag-and-drop flashing
pico_add_extra_outputs(w5500_livox_emulator)
