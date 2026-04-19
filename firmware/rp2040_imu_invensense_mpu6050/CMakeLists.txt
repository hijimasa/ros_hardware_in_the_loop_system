cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_imu_invensense_mpu6050 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040_imu_invensense_mpu6050
    src/main.c
)

# Generate PIO header from .pio file (stub, for future use)
pico_generate_pio_header(rp2040_imu_invensense_mpu6050
    ${CMAKE_CURRENT_LIST_DIR}/pio/i2c_slave.pio
)

target_include_directories(rp2040_imu_invensense_mpu6050 PRIVATE
    src/
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

target_link_libraries(rp2040_imu_invensense_mpu6050
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_pio
)

# Use pico-sdk's built-in USB CDC serial (stdio over USB)
pico_enable_stdio_usb(rp2040_imu_invensense_mpu6050 1)
pico_enable_stdio_uart(rp2040_imu_invensense_mpu6050 0)

# Generate .uf2 for drag-and-drop flashing
pico_add_extra_outputs(rp2040_imu_invensense_mpu6050)
