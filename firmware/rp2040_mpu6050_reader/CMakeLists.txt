cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_mpu6050_reader C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040_mpu6050_reader
    src/main.c
)

target_include_directories(rp2040_mpu6050_reader PRIVATE
    src/
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

target_link_libraries(rp2040_mpu6050_reader
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

# USB CDC serial for the observation log (stdio over USB)
pico_enable_stdio_usb(rp2040_mpu6050_reader 1)
pico_enable_stdio_uart(rp2040_mpu6050_reader 0)

pico_add_extra_outputs(rp2040_mpu6050_reader)
