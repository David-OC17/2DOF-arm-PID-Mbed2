cmake_minimum_required(VERSION 3.12)

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH is not set")
endif()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(2dof_arm_pid_mbed2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)
add_executable(2dof_arm_pid_mbed2
    src/pico_uart_transport.c
    src/common.c
    src/control_law.c
    src/joint_state.c
    src/main.c
)

target_link_libraries(2dof_arm_pid_mbed2
    pico_stdlib
    microros
    hardware_pwm
    hardware_gpio
    hardware_irq
    hardware_clocks
    hardware_spi
)

target_include_directories(2dof_arm_pid_mbed2 PUBLIC
    libmicroros/include
    include/
)

# Optimize binary size
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(2dof_arm_pid_mbed2 1)
pico_enable_stdio_uart(2dof_arm_pid_mbed2 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(2dof_arm_pid_mbed2)
