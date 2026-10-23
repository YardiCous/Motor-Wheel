cmake_minimum_required(VERSION 3.20)

# This project has no host or alternate-board build.
if(DEFINED PICO_BOARD AND NOT PICO_BOARD STREQUAL "pico_w")
    message(FATAL_ERROR "Motor-Wheel supports only PICO_BOARD=pico_w (RP2040).")
endif()
set(PICO_BOARD pico_w CACHE STRING "Required board" FORCE)
if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Set PICO_SDK_PATH to a Pico SDK checkout with its submodules initialized.")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
project(motor_wheel C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
pico_sdk_init()
if(NOT PICO_PLATFORM STREQUAL "rp2040")
    message(FATAL_ERROR "Only the RP2040 Pico W is supported.")
endif()

include(cmake/micro_t.cmake)
add_executable(motor_wheel
    src/packed.c src/controller.c src/encoder.c
    firmware/main.c firmware/tasks.c firmware/board.c firmware/motion.c
    firmware/line.c firmware/imu.c firmware/obstacle.c firmware/navigation.c
    firmware/network.c firmware/state.c
)
target_include_directories(motor_wheel PRIVATE include firmware)
target_link_libraries(motor_wheel PRIVATE micro_t pico_stdlib hardware_adc hardware_i2c
    hardware_pwm hardware_watchdog pico_cyw43_arch_lwip_poll pico_lwip_mqtt m)
target_compile_options(motor_wheel PRIVATE -Wall -Wextra -Werror)
target_compile_definitions(motor_wheel PRIVATE PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=0 PICO_STDIO_DEADLOCK_TIMEOUT_MS=0)
pico_enable_stdio_usb(motor_wheel 1)
pico_enable_stdio_uart(motor_wheel 0)
pico_add_extra_outputs(motor_wheel)

option(ROBOT_ON_TARGET_TESTS "Run core tests at boot with motors off" OFF)
if(ROBOT_ON_TARGET_TESTS)
    target_sources(motor_wheel PRIVATE tests/test_robot.c tests/test_kernel.c)
    target_compile_definitions(motor_wheel PRIVATE ROBOT_ON_TARGET_TESTS=1)
    set_source_files_properties(tests/test_robot.c tests/test_kernel.c PROPERTIES COMPILE_OPTIONS "-UNDEBUG")
endif()
option(ROBOT_MOTOR_TESTS "Run the opt-in physical motor sequence before starting tasks" OFF)
if(ROBOT_MOTOR_TESTS)
    target_sources(motor_wheel PRIVATE tests/test_motion.c)
    target_compile_definitions(motor_wheel PRIVATE ROBOT_MOTOR_TESTS=1)
    set_source_files_properties(tests/test_motion.c PROPERTIES COMPILE_OPTIONS "-UNDEBUG")
endif()
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format clang-format-20 clang-format-19)
if(CLANG_FORMAT_EXECUTABLE)
    file(GLOB FORMAT_SOURCES CONFIGURE_DEPENDS include/*.h src/*.c firmware/*.c firmware/*.h
        firmware/kernel_config/*.h tests/*.c)
    add_custom_target(format COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${FORMAT_SOURCES})
    add_custom_target(format-check COMMAND ${CLANG_FORMAT_EXECUTABLE} --dry-run --Werror ${FORMAT_SOURCES})
endif()
