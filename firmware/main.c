#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "robot.h"
#ifdef ROBOT_ON_TARGET_TESTS
void robot_core_tests(void);
#endif
#ifdef ROBOT_MOTOR_TESTS
void robot_motor_tests(void);
#endif
int main(void) {
    set_sys_clock_khz(125000, true);
    board_init();
    stdio_init_all();
    network_prepare();
#ifdef ROBOT_ON_TARGET_TESTS
    robot_core_tests();
#endif
#ifdef ROBOT_MOTOR_TESTS
    robot_motor_tests();
#endif
    kernel_start(robot_tasks_entry);
}
