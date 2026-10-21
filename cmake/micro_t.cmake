# Native kernel from the course-linked Pico port. SDK owns reset/boot2/vectors;
# this target owns tasks, scheduling and SysTick. Core 1 stays parked.
set(MICRO_T_ROOT "${CMAKE_CURRENT_LIST_DIR}/../third_party/micro_t")
file(GLOB MICRO_T_COMMON CONFIGURE_DEPENDS
    "${MICRO_T_ROOT}/kernel/tkernel/*.c"
    "${MICRO_T_ROOT}/kernel/tstdlib/*.c"
    "${MICRO_T_ROOT}/lib/libtk/*.c")
add_library(micro_t STATIC ${MICRO_T_COMMON}
    "${MICRO_T_ROOT}/kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c"
    "${MICRO_T_ROOT}/kernel/sysdepend/cpu/core/armv6m/dispatch.S"
    "${MICRO_T_ROOT}/kernel/sysdepend/pico_rp2040/power_save.c"
    "${MICRO_T_ROOT}/lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c"
    "${MICRO_T_ROOT}/lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c"
    "${CMAKE_CURRENT_LIST_DIR}/../firmware/kernel_sdk.c"
    "${CMAKE_CURRENT_LIST_DIR}/../firmware/kernel_core.c")
target_include_directories(micro_t PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/../firmware/kernel_config"
    "${MICRO_T_ROOT}/include")
target_include_directories(micro_t PRIVATE "${MICRO_T_ROOT}/kernel/knlinc")
target_compile_definitions(micro_t PUBLIC _PICO_RP2040_ CNF_SMP=0)
target_link_libraries(micro_t PUBLIC pico_stdlib hardware_exception hardware_irq hardware_clocks)
# Do not mix the BSP reset handler/vector table or board pin setup with SDK startup.
