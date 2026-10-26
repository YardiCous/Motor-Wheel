/** @file physical_test.c
 * @brief Explicitly armed encoder-controlled bench sequence.
 */
#include "physical_test.h"

#include "../board_io.h"
#include "../robot_config.h"

#include <tm/tmonitor.h>

#define PHYSICAL_TEST_IDLE 0u
#define PHYSICAL_TEST_FORWARD 1u
#define PHYSICAL_TEST_PAUSE_FORWARD 2u
#define PHYSICAL_TEST_BACKWARD 3u
#define PHYSICAL_TEST_PAUSE_BACKWARD 4u
#define PHYSICAL_TEST_LEFT 5u
#define PHYSICAL_TEST_PAUSE_LEFT 6u
#define PHYSICAL_TEST_RIGHT 7u
#define PHYSICAL_TEST_DONE 8u
#define PHYSICAL_TEST_FAILED 9u
#define PHYSICAL_TEST_PAUSE_MS 500u
#define PHYSICAL_TEST_DISTANCE_MM 500u
#define PHYSICAL_TEST_TURN_MM 120u
#define PHYSICAL_TEST_SPEED 250u
#define PHYSICAL_TEST_START 1u
#define PHYSICAL_TEST_ESTOP 2u
#define PHYSICAL_TEST_RESET 4u

static bool start_command(physical_test_t * p_test, motion_t * p_motion,
                          uint8_t phase, uint32_t command);
static bool stopped(void);
static void report(const char * p_message);
static uint32_t command_forward(void);
static uint32_t command_backward(void);
static uint32_t command_left(void);
static uint32_t command_right(void);
static void pause_step(physical_test_t * p_test, motion_t * p_motion);
static void active_step(physical_test_t * p_test, motion_t * p_motion,
                        int16_t * p_left_pwm, int16_t * p_right_pwm);

/** @brief The test is armed only by a new START-button press. */
void physical_test_init (physical_test_t * p_test)
{
    p_test->phase            = PHYSICAL_TEST_IDLE;
    p_test->previous_buttons = board_io_buttons();
    p_test->pause_until_ms   = 0u;
    p_test->b_reported_ready = false;
    report("PHYSICAL MOTOR TEST: press START with wheels clear");
}

/** @brief Run one 10-ms encoder-controlled test cycle. */
void physical_test_step (physical_test_t * p_test, motion_t * p_motion,
                        int16_t * p_left_pwm, int16_t * p_right_pwm)
{
    uint8_t buttons = board_io_buttons();
    bool    b_start = (0u != (buttons & PHYSICAL_TEST_START)) &&
                      (0u == (p_test->previous_buttons &
                              PHYSICAL_TEST_START));
    bool b_reset = 0u != (buttons & PHYSICAL_TEST_RESET);
    p_test->previous_buttons = buttons;
    *p_left_pwm              = 0;
    *p_right_pwm             = 0;

    if (b_reset)
    {
        board_io_motor(0, 0);
        board_io_clear_fault();
        physical_test_init(p_test);
        motion_init(p_motion, 0u, 0u);
    }
    else if ((PHYSICAL_TEST_IDLE == p_test->phase) && b_start)
    {
        uint32_t left  = 0u;
        uint32_t right = 0u;
        bool b_valid = board_io_encoders(&left, &right);
        motion_init(p_motion, left, right);
        b_valid = b_valid &&
                  (start_command(p_test, p_motion, PHYSICAL_TEST_FORWARD,
                                 command_forward()));
        if (b_valid)
        {
            report("PHYSICAL MOTOR TEST: FORWARD");
        }
    }
    else if ((PHYSICAL_TEST_IDLE == p_test->phase) ||
             (PHYSICAL_TEST_DONE == p_test->phase) ||
             (PHYSICAL_TEST_FAILED == p_test->phase))
    {
        /* Remain stopped until RESET and a new START press. */
    }
    else if (stopped())
    {
        p_test->phase = PHYSICAL_TEST_FAILED;
        report("PHYSICAL MOTOR TEST: STOPPED - E-STOP");
    }
    else if ((PHYSICAL_TEST_PAUSE_FORWARD == p_test->phase) ||
             (PHYSICAL_TEST_PAUSE_BACKWARD == p_test->phase) ||
             (PHYSICAL_TEST_PAUSE_LEFT == p_test->phase))
    {
        pause_step(p_test, p_motion);
    }
    else
    {
        active_step(p_test, p_motion, p_left_pwm, p_right_pwm);
    }
}

static void pause_step (physical_test_t * p_test, motion_t * p_motion)
{
    if (board_io_time_ms() >= p_test->pause_until_ms)
    {
        uint32_t command = 0u;
        uint8_t phase     = PHYSICAL_TEST_IDLE;

        if (PHYSICAL_TEST_PAUSE_FORWARD == p_test->phase)
        {
            command = command_backward();
            phase   = PHYSICAL_TEST_BACKWARD;
        }
        else if (PHYSICAL_TEST_PAUSE_BACKWARD == p_test->phase)
        {
            command = command_left();
            phase   = PHYSICAL_TEST_LEFT;
        }
        else
        {
            command = command_right();
            phase   = PHYSICAL_TEST_RIGHT;
        }

        if (start_command(p_test, p_motion, phase, command))
        {
            report((PHYSICAL_TEST_BACKWARD == phase)
                       ? "PHYSICAL MOTOR TEST: BACKWARD"
                       : (PHYSICAL_TEST_LEFT == phase)
                             ? "PHYSICAL MOTOR TEST: TURN LEFT"
                             : "PHYSICAL MOTOR TEST: TURN RIGHT");
        }
    }
}

static void active_step (physical_test_t * p_test, motion_t * p_motion,
                        int16_t * p_left_pwm, int16_t * p_right_pwm)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;
    bool b_valid = board_io_encoders(&left, &right);
    motion_step(p_motion, left, right, 0, !b_valid, p_left_pwm, p_right_pwm);

    if (MOTION_IDLE == p_motion->mode)
    {
        p_test->pause_until_ms = board_io_time_ms() + PHYSICAL_TEST_PAUSE_MS;
        p_test->phase++;

        if (PHYSICAL_TEST_PAUSE_LEFT < p_test->phase)
        {
            p_test->phase = PHYSICAL_TEST_DONE;
            report("PHYSICAL MOTOR TEST: PASS - all phases complete");
        }
    }
    else if (MOTION_STALL == p_motion->mode)
    {
        p_test->phase = PHYSICAL_TEST_FAILED;
        report("PHYSICAL MOTOR TEST: FAIL - encoder stall");
    }
    else if (MOTION_ESTOP == p_motion->mode)
    {
        p_test->phase = PHYSICAL_TEST_FAILED;
        uint8_t faults = board_io_encoder_faults();
        report((0u != (faults & 1u))
                   ? "PHYSICAL MOTOR TEST: FAIL - left encoder fault"
                   : (0u != (faults & 2u))
                         ? "PHYSICAL MOTOR TEST: FAIL - right encoder fault"
                         : "PHYSICAL MOTOR TEST: FAIL - motion encoder fault");
    }
    else
    {
        /* Continue the active encoder-controlled phase. */
    }
}

/** @brief Build test commands through the public motion API. */
static uint32_t command_forward (void)
{
    uint32_t command = 0u;
    (void)motion_forward(&command, PHYSICAL_TEST_DISTANCE_MM,
                         PHYSICAL_TEST_SPEED);
    return command;
}

static uint32_t command_backward (void)
{
    uint32_t command = 0u;
    (void)motion_backward(&command, PHYSICAL_TEST_DISTANCE_MM,
                          PHYSICAL_TEST_SPEED);
    return command;
}

static uint32_t command_left (void)
{
    uint32_t command = 0u;
    (void)motion_turn_left(&command, PHYSICAL_TEST_TURN_MM,
                           PHYSICAL_TEST_SPEED);
    return command;
}

static uint32_t command_right (void)
{
    uint32_t command = 0u;
    (void)motion_turn_right(&command, PHYSICAL_TEST_TURN_MM,
                            PHYSICAL_TEST_SPEED);
    return command;
}

static bool start_command (physical_test_t * p_test, motion_t * p_motion,
                          uint8_t phase, uint32_t command)
{
    bool b_valid = motion_accept(p_motion, command);
    if (b_valid)
    {
        p_test->phase = phase;
    }
    return b_valid;
}

static bool stopped (void)
{
    return 0u != (board_io_buttons() & PHYSICAL_TEST_ESTOP);
}

static void report (const char * p_message)
{
    tm_printf((UB*)"%s\n", (UB*)p_message);
}

/*** End of file. ***/

