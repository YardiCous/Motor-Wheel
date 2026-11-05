/** @file motor_test.c
 * @brief Host simulation of the operator motor checks.
 */
#include "motor_test.h"

#include "../../app_program/subsystem2/motion.h"
#include "../../app_program/robot_config.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define MOTOR_TEST_TICKS 110u
#define MOTOR_TEST_EDGES 5u
#define MOTOR_TEST_DISTANCE_MM 100u
#define MOTOR_TEST_SPEED 150u

static void run_command(uint32_t command, bool b_turn_left);
static void run_straight(bool b_reverse, bool b_left_stronger);

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
    run_straight(false, true);
    run_straight(false, false);
    run_straight(true, true);
    run_straight(true, false);
}

/** @brief Simulate unequal motors, encoder quantisation and drive inertia.
 * A stronger motor travels 7 um per duty unit per tick, the other 6 um.
 * Exercise both drift directions and both travel directions over 500 mm.
 */
static void run_straight(bool b_reverse, bool b_left_stronger)
{
    motion_t controller = {0};
    uint32_t command = 0u;
    int32_t left_um = 0;
    int32_t right_um = 0;
    int32_t left_velocity = 0;
    int32_t right_velocity = 0;
    int16_t left_pwm = 0;
    int16_t right_pwm = 0;
    int32_t left_gain = b_left_stronger ? 7 : 6;
    int32_t right_gain = b_left_stronger ? 6 : 7;
    motion_init(&controller, 0u, 0u);
    assert(motion_command(&command, MOTION_MOVE, 500u, 250u, b_reverse));
    assert(motion_accept(&controller, command));

    for (uint32_t tick = 0u; tick < 1000u; ++tick)
    {
        left_velocity = ((3 * left_velocity) + (left_pwm * left_gain)) / 4;
        right_velocity =
            ((3 * right_velocity) + (right_pwm * right_gain)) / 4;
        left_um += left_velocity;
        right_um += right_velocity;
        uint32_t left = (uint32_t)(((int64_t)left_um *
                                    ROBOT_ENCODER_EDGES_TURN) /
                                   ROBOT_WHEEL_CIRCUM_UM);
        uint32_t right = (uint32_t)(((int64_t)right_um *
                                     ROBOT_ENCODER_EDGES_TURN) /
                                    ROBOT_WHEEL_CIRCUM_UM);
        motion_step(&controller, left, right, 0, false,
                    &left_pwm, &right_pwm);
        assert((left_pwm >= -1000) && (left_pwm <= 1000));
        assert((right_pwm >= -1000) && (right_pwm <= 1000));

        if (MOTION_IDLE == controller.mode)
        {
            break;
        }
    }

    int32_t mismatch = left_um - right_um;
    int32_t trim_error =
        mismatch + (((left_um + right_um) * ROBOT_STRAIGHT_TRIM_PERMILLE) /
                    1000);
    assert(controller.b_done);
    assert((0 == left_pwm) && (0 == right_pwm));
    assert((trim_error > -1000) && (trim_error < 1000));
    (void)printf("MOTOR: %s, stronger %s, 500 mm mismatch %d um\n",
                 b_reverse ? "reverse" : "forward",
                 b_left_stronger ? "left" : "right", (int)mismatch);
    assert(motion_stop(&command));
    assert(motion_accept(&controller, command));
    assert(0 == controller.straight_error_edges);
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

