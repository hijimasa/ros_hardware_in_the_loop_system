cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_actuator_servo_pwm C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rp2040_actuator_servo_pwm
    src/main.c
)

# Generate PIO header from .pio file
pico_generate_pio_header(rp2040_actuator_servo_pwm
    ${CMAKE_CURRENT_LIST_DIR}/pio/pwm_servo.pio
)

target_include_directories(rp2040_actuator_servo_pwm PRIVATE
    src/
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

target_link_libraries(rp2040_actuator_servo_pwm
    pico_stdlib
    hardware_pio
    hardware_gpio
    hardware_clocks
)

# Use pico-sdk's built-in USB CDC serial (stdio over USB)
pico_enable_stdio_usb(rp2040_actuator_servo_pwm 1)
pico_enable_stdio_uart(rp2040_actuator_servo_pwm 0)

# Generate .uf2 for drag-and-drop flashing
pico_add_extra_outputs(rp2040_actuator_servo_pwm)
