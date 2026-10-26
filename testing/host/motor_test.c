/** @file motor_test.c
 * @brief Host simulation of the operator motor checks.
 */
#include "motor_test.h"

#include "../../app_program/subsystem2/motion.h"

#include <assert.h>
#include <stdint.h>

#define MOTOR_TEST_TICKS 110u
#define MOTOR_TEST_EDGES 5u
#define MOTOR_TEST_DISTANCE_MM 100u
#define MOTOR_TEST_SPEED 150u

static void run_command(uint32_t command, bool b_turn_left);

/** @brief Exercise forward, reverse, both turn directions and stop. */
void motor_test_run (void)
{
    uint32_t command = 0u;

    assert(motion_forward(&command, MOTOR_TEST_DISTANCE_MM,
                          MOTOR_TEST_SPEED));
    run_command(command, false);
    assert(motion_backward(&command, MOTOR_TEST_DISTANCE_MM,
                           MOTOR_TEST_SPEED));
    run_command(command, false);
    assert(motion_turn_left(&command, MOTOR_TEST_DISTANCE_MM,
                            MOTOR_TEST_SPEED));
    run_command(command, true);
    assert(motion_turn_right(&command, MOTOR_TEST_DISTANCE_MM,
                             MOTOR_TEST_SPEED));
    run_command(command, false);
    assert(motion_stop(&command));
    assert(0u == command);

    motion_t controller = {0};
    int16_t left_pwm     = 1;
    int16_t right_pwm    = 1;
    motion_init(&controller, 0u, 0u);
    assert(motion_forward(&command, 10u, 100u));
    assert(motion_accept(&controller, command));
    assert(motion_stop(&command));
    assert(motion_accept(&controller, command));
    motion_step(&controller, 0u, 0u, 0, false, &left_pwm, &right_pwm);
    assert((0 == left_pwm) && (0 == right_pwm));
}

/** @brief Feed the expected signed encoder direction for one manoeuvre. */
static void run_command (uint32_t command, bool b_turn_left)
{
    motion_t controller = {0};
    uint32_t left       = 0u;
    uint32_t right      = 0u;
    motion_init(&controller, left, right);
    assert(motion_accept(&controller, command));

    bool b_reverse = (0u != ((command >> 23u) & 1u));
    bool b_checked_direction = false;

    for (uint32_t tick = 0u; tick < MOTOR_TEST_TICKS; ++tick)
    {
        int32_t direction = b_reverse ? -1 : 1;
        int32_t left_step = direction * (int32_t)MOTOR_TEST_EDGES;
        int32_t right_step = left_step;

        if ((MOTION_TURN == controller.mode) &&
            (b_turn_left == b_reverse))
        {
            right_step = -right_step;
        }

        left += (uint32_t)left_step;
        right += (uint32_t)right_step;
        int16_t left_pwm  = 0;
        int16_t right_pwm = 0;
        motion_step(&controller, left, right, 0, false, &left_pwm,
                    &right_pwm);
        assert((left_pwm >= -1000) && (left_pwm <= 1000));
        assert((right_pwm >= -1000) && (right_pwm <= 1000));

        if (!b_checked_direction)
        {
            if (MOTION_MOVE == controller.mode)
            {
                assert((b_reverse && (left_pwm < 0) && (right_pwm < 0)) ||
                       ((!b_reverse) && (left_pwm > 0) && (right_pwm > 0)));
            }
            else
            {
                assert((b_turn_left && (left_pwm < 0) && (right_pwm > 0)) ||
                       ((!b_turn_left) && (left_pwm > 0) && (right_pwm < 0)));
            }
            b_checked_direction = true;
        }
    }

    assert(MOTION_IDLE == controller.mode);
}

/*** End of file. ***/

