#include "pico/stdlib.h"
#include "robot.h"
#include <stdio.h>

enum {
    MOTOR_TEST_DUTY = 350,
    MOTOR_TEST_STEP_MS = 10,
    MOTOR_TEST_RUN_MS = 1000,
    MOTOR_TEST_PAUSE_MS = 500
};

static bool motor_test_step(const char *name, int left, int right, unsigned duration) {
    printf("Motor test: %s (left=%d, right=%d)\n", name, left, right);
    for (unsigned elapsed = 0; elapsed < duration; elapsed += MOTOR_TEST_STEP_MS) {
        if (board_estop_active()) {
            board_motor_write(0, 0);
            printf("Motor test stopped by E-stop.\n");
            return false;
        }
        board_motor_write(left, right);
        sleep_ms(MOTOR_TEST_STEP_MS);
    }
    board_motor_write(0, 0);
    sleep_ms(MOTOR_TEST_PAUSE_MS);
    return true;
}

static void motor_test_stop(void) {
    printf("Motor test: stop\n");
    board_motor_write(0, 0);
    sleep_ms(MOTOR_TEST_PAUSE_MS);
}

void robot_motor_tests(void) {
    board_motor_write(0, 0);
    printf("Motor test begins in 3 seconds; lift the robot clear of the floor.\n");
    sleep_ms(3000);
    if (!motor_test_step("forward", MOTOR_TEST_DUTY, MOTOR_TEST_DUTY, MOTOR_TEST_RUN_MS))
        return;
    motor_test_stop();
    if (!motor_test_step("backward", -MOTOR_TEST_DUTY, -MOTOR_TEST_DUTY, MOTOR_TEST_RUN_MS))
        return;
    motor_test_stop();
    if (!motor_test_step("turn left", -MOTOR_TEST_DUTY, MOTOR_TEST_DUTY, MOTOR_TEST_RUN_MS))
        return;
    motor_test_stop();
    if (!motor_test_step("turn right", MOTOR_TEST_DUTY, -MOTOR_TEST_DUTY, MOTOR_TEST_RUN_MS))
        return;
    motor_test_stop();
    board_motor_write(0, 0);
    printf("Motor test complete; outputs are stopped.\n");
}
