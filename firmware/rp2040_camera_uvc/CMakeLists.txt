cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_camera_uvc C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040_camera_uvc
    src/main.c
    src/spi_receiver.c
    src/uvc_device.c
    src/usb_descriptors.c
    src/frame_buffer.c
)

target_include_directories(rp2040_camera_uvc PRIVATE
    src/
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

# UART1 (GP4=TX, GP5=RX) for stdio debug — UART0 is reserved for HILS data
target_compile_definitions(rp2040_camera_uvc PRIVATE
    PICO_DEFAULT_UART=1
    PICO_DEFAULT_UART_TX_PIN=4
    PICO_DEFAULT_UART_RX_PIN=5
)

target_link_libraries(rp2040_camera_uvc
    pico_stdlib
    hardware_uart
    hardware_dma
    hardware_gpio
    tinyusb_device
    tinyusb_board
)

# USB port is used exclusively for UVC, not stdio
pico_enable_stdio_usb(rp2040_camera_uvc 0)
# UART1 (GP4=TX, GP5=RX) for debug output — UART0 is used for HILS data
pico_enable_stdio_uart(rp2040_camera_uvc 1)

# Generate .uf2 for drag-and-drop flashing
pico_add_extra_outputs(rp2040_camera_uvc)
