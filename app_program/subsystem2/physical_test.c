/** @file physical_test.c
 * @brief START-armed motor calibration run with encoder reports.
 *
 * Sequence: forward 60 cm, back 60 cm (straightness), then four left
 * quarter turns and four right quarter turns (turn size). Four turns make a
 * full circle, so a small per-turn error shows up four times larger. After
 * every step the wheel edges are printed; the summary explains which
 * robot_config.h value to change.
 */
#include "physical_test.h"

#include "../board_io.h"
#include "../robot_config.h"

#include <tm/tmonitor.h>

#define PHYSICAL_TEST_IDLE 0u
#define PHYSICAL_TEST_ACTIVE 1u
#define PHYSICAL_TEST_PAUSE 2u
#define PHYSICAL_TEST_DONE 3u
#define PHYSICAL_TEST_FAILED 4u
#define PHYSICAL_TEST_STEPS 10u
#define PHYSICAL_TEST_STRAIGHT_STEPS 2u
#define PHYSICAL_TEST_TURNS_EACH 4u
#define PHYSICAL_TEST_PAUSE_MS 1500u
/* 60 cm at 70 mm/s is 8.6 s, inside the 10-s manoeuvre limit. */
#define PHYSICAL_TEST_DISTANCE_MM 600u
#define PHYSICAL_TEST_SPEED 70u
#define PHYSICAL_TEST_START 1u
#define PHYSICAL_TEST_ESTOP 2u
#define PHYSICAL_TEST_RESET 4u

static void     arm(physical_test_t * p_test, motion_t * p_motion);
static bool     start_step(physical_test_t * p_test, motion_t * p_motion);
static uint32_t step_command(uint8_t step);
static void     pause_step(physical_test_t * p_test, motion_t * p_motion);
static void     active_step(physical_test_t * p_test, motion_t * p_motion,
                            int16_t * p_left_pwm, int16_t * p_right_pwm);
static void     report_step(const physical_test_t * p_test);
static void     report_summary(void);
static void     report_fault(void);
static bool     stopped(void);
static void     report(const char * p_message);

/** @brief The test is armed only by a new START-button press. */
void physical_test_init(physical_test_t * p_test)
{
    p_test->phase            = PHYSICAL_TEST_IDLE;
    p_test->step             = 0u;
    p_test->previous_buttons = board_io_buttons();
    p_test->pause_until_ms   = 0u;
    p_test->b_reported_ready = false;
    report("MOTOR CAL: on the floor, press START (GP20). RESET = GP22.");
    tm_printf((UB *)"MOTOR CAL: 1 fwd %u mm, 2 back %u mm, 3-6 left x4, "
                    "7-10 right x4\n",
              (unsigned)PHYSICAL_TEST_DISTANCE_MM,
              (unsigned)PHYSICAL_TEST_DISTANCE_MM);
    tm_printf((UB *)"MOTOR CAL: ROBOT_QUARTER_TURN_MM=%u "
                    "ROBOT_STRAIGHT_TRIM_PERMILLE=%d speed %u mm/s\n",
              (unsigned)ROBOT_QUARTER_TURN_MM,
              (int)ROBOT_STRAIGHT_TRIM_PERMILLE,
              (unsigned)PHYSICAL_TEST_SPEED);
}

/** @brief Run one 10-ms encoder-controlled test cycle. */
void physical_test_step(physical_test_t * p_test, motion_t * p_motion,
                        int16_t * p_left_pwm, int16_t * p_right_pwm)
{
    uint8_t buttons = board_io_buttons();
    bool    b_start = (0u != (buttons & PHYSICAL_TEST_START)) &&
                      (0u == (p_test->previous_buttons & PHYSICAL_TEST_START));
    bool    b_reset = (0u != (buttons & PHYSICAL_TEST_RESET));
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
        arm(p_test, p_motion);
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
        report("MOTOR CAL: STOPPED - E-STOP");
    }
    else if (PHYSICAL_TEST_PAUSE == p_test->phase)
    {
        pause_step(p_test, p_motion);
    }
    else
    {
        active_step(p_test, p_motion, p_left_pwm, p_right_pwm);
    }
}

/** @brief Start the first step from the current encoder counts. */
static void arm(physical_test_t * p_test, motion_t * p_motion)
{
    uint32_t left    = 0u;
    uint32_t right   = 0u;
    bool     b_valid = board_io_encoders(&left, &right);
    motion_init(p_motion, left, right);
    p_test->step = 0u;

    if (!(b_valid && (start_step(p_test, p_motion))))
    {
        p_test->phase = PHYSICAL_TEST_FAILED;
        report("MOTOR CAL: FAIL - encoders not ready");
    }
}

/** @brief Snapshot the encoders and hand the step's command to Motion. */
static bool start_step(physical_test_t * p_test, motion_t * p_motion)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;
    (void)board_io_encoders(&left, &right);
    bool b_valid = motion_accept(p_motion, step_command(p_test->step));

    if (b_valid)
    {
        p_test->phase       = PHYSICAL_TEST_ACTIVE;
        p_test->start_left  = left;
        p_test->start_right = right;
        p_test->start_ms    = board_io_time_ms();
        tm_printf((UB *)"MOTOR CAL: step %u start\n",
                  (unsigned)(p_test->step + 1u));
    }

    return b_valid;
}

/** @brief Steps 0-1 straight, 2-5 left turns, 6-9 right turns. */
static uint32_t step_command(uint8_t step)
{
    uint32_t command = 0u;

    if (0u == step)
    {
        (void)motion_forward(&command, PHYSICAL_TEST_DISTANCE_MM,
                             PHYSICAL_TEST_SPEED);
    }
    else if (1u == step)
    {
        (void)motion_backward(&command, PHYSICAL_TEST_DISTANCE_MM,
                              PHYSICAL_TEST_SPEED);
    }
    else if (step < (PHYSICAL_TEST_STRAIGHT_STEPS + PHYSICAL_TEST_TURNS_EACH))
    {
        (void)motion_turn_left(&command, ROBOT_QUARTER_TURN_MM,
                               PHYSICAL_TEST_SPEED);
    }
    else
    {
        (void)motion_turn_right(&command, ROBOT_QUARTER_TURN_MM,
                                PHYSICAL_TEST_SPEED);
    }

    return command;
}

/** @brief Report once the robot has coasted to rest, then start the next
 * step (or finish).
 */
static void pause_step(physical_test_t * p_test, motion_t * p_motion)
{
    if (board_io_time_ms() >= p_test->pause_until_ms)
    {
        report_step(p_test);
        ++p_test->step;

        if (p_test->step >= PHYSICAL_TEST_STEPS)
        {
            p_test->phase = PHYSICAL_TEST_DONE;
            report_summary();
        }
        else if (!start_step(p_test, p_motion))
        {
            p_test->phase = PHYSICAL_TEST_FAILED;
            report("MOTOR CAL: FAIL - command rejected");
        }
        else
        {
            /* The next step is running. */
        }
    }
}

/** @brief Drive the motors until Motion finishes, stalls or faults. */
static void active_step(physical_test_t * p_test, motion_t * p_motion,
                        int16_t * p_left_pwm, int16_t * p_right_pwm)
{
    uint32_t left    = 0u;
    uint32_t right   = 0u;
    bool     b_valid = board_io_encoders(&left, &right);
    motion_step(p_motion, left, right, 0, !b_valid, p_left_pwm, p_right_pwm);

    if (MOTION_IDLE == p_motion->mode)
    {
        p_test->phase          = PHYSICAL_TEST_PAUSE;
        p_test->pause_until_ms = board_io_time_ms() + PHYSICAL_TEST_PAUSE_MS;
    }
    else if ((MOTION_STALL == p_motion->mode) ||
             (MOTION_ESTOP == p_motion->mode))
    {
        p_test->phase = PHYSICAL_TEST_FAILED;
        report_fault();
    }
    else
    {
        /* Continue the active encoder-controlled step. */
    }
}

/** @brief Wheel edges for the finished step, as distance or as degrees
 * (relative to the calibrated quarter turn), including coasting.
 */
static void report_step(const physical_test_t * p_test)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;
    (void)board_io_encoders(&left, &right);
    int32_t  edges_left  = (int32_t)(left - p_test->start_left);
    int32_t  edges_right = (int32_t)(right - p_test->start_right);
    uint32_t abs_left =
        (edges_left < 0) ? (uint32_t)(-edges_left) : (uint32_t)edges_left;
    uint32_t abs_right =
        (edges_right < 0) ? (uint32_t)(-edges_right) : (uint32_t)edges_right;
    uint64_t travel_um =
        ((uint64_t)(abs_left + abs_right) * ROBOT_WHEEL_CIRCUM_UM) /
        (2u * (uint64_t)ROBOT_ENCODER_EDGES_TURN);
    uint32_t ms = board_io_time_ms() - p_test->start_ms;

    if (p_test->step < PHYSICAL_TEST_STRAIGHT_STEPS)
    {
        tm_printf((UB *)"ENC %s: L=%d R=%d edges = %u mm, L-R %d edges, "
                        "%u ms\n",
                  (UB *)((0u == p_test->step) ? "FWD " : "BACK"),
                  (int)edges_left, (int)edges_right,
                  (unsigned)(travel_um / 1000u),
                  (int)(edges_left - edges_right), (unsigned)ms);
    }
    else
    {
        bool     b_left = (p_test->step < (PHYSICAL_TEST_STRAIGHT_STEPS +
                                           PHYSICAL_TEST_TURNS_EACH));
        uint32_t turn =
            ((uint32_t)(p_test->step - PHYSICAL_TEST_STRAIGHT_STEPS) %
             PHYSICAL_TEST_TURNS_EACH) +
            1u;
        uint64_t tenths =
            (travel_um * 900u) / ((uint64_t)ROBOT_QUARTER_TURN_MM * 1000u);
        tm_printf((UB *)"ENC %s %u: L=%d R=%d edges = %u.%u deg by "
                        "encoders, %u ms\n",
                  (UB *)(b_left ? "LEFT " : "RIGHT"), (unsigned)turn,
                  (int)edges_left, (int)edges_right, (unsigned)(tenths / 10u),
                  (unsigned)(tenths % 10u), (unsigned)ms);
    }
}

/** @brief What to measure and which robot_config.h value to change. */
static void report_summary(void)
{
    report("MOTOR CAL: DONE. Check by eye, then edit robot_config.h:");
    report("TURNS: after 4 left (and 4 right) it should face the start "
           "direction.");
    tm_printf((UB *)"  short by X deg in total -> ROBOT_QUARTER_TURN_MM = "
                    "%u x 360 / (360 - X)\n",
              (unsigned)ROBOT_QUARTER_TURN_MM);
    tm_printf((UB *)"  long by X deg in total  -> ROBOT_QUARTER_TURN_MM = "
                    "%u x 360 / (360 + X)\n",
              (unsigned)ROBOT_QUARTER_TURN_MM);
    report("STRAIGHT: measure sideways drift at the end of the 60 cm "
           "forward run.");
    report("  drifts RIGHT: raise ROBOT_STRAIGHT_TRIM_PERMILLE by 3 per cm");
    report("  drifts LEFT:  lower ROBOT_STRAIGHT_TRIM_PERMILLE by 3 per cm");
    report("  (L-R edges far from 0 means the motors fight the "
           "correction: send the log)");
}

/** @brief Name the failing wheel when the encoders explain the stop. */
static void report_fault(void)
{
    uint8_t faults = board_io_encoder_faults();
    report((0u != (faults & 1u))   ? "MOTOR CAL: FAIL - left encoder fault"
           : (0u != (faults & 2u)) ? "MOTOR CAL: FAIL - right encoder fault"
                                   : "MOTOR CAL: FAIL - stall or encoder");
}

static bool stopped(void)
{
    return (0u != (board_io_buttons() & PHYSICAL_TEST_ESTOP));
}

static void report(const char * p_message)
{
    tm_printf((UB *)"%s\n", (UB *)p_message);
}

/*** End of file. ***/

