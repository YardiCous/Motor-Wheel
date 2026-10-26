/** @file encoder_test.c
 * @brief Motor-off quadrature encoder diagnostic.
 */
#include "encoder_test.h"

#include "../board_io.h"

#include <tm/tmonitor.h>

#define ENCODER_TEST_START 1u
#define ENCODER_TEST_RESET 4u
#define ENCODER_TEST_PERIOD_MS 250u

static void report(const char * p_message);

void encoder_test_init (encoder_test_t * p_test)
{
    p_test->last_report_ms   = board_io_time_ms();
    p_test->last_left        = 0u;
    p_test->last_right       = 0u;
    p_test->previous_buttons = board_io_buttons();
    p_test->b_running        = false;
    board_io_motor(0, 0);
    report("ENCODER TEST: motors disabled; rotate wheels by hand");
    report("ENCODER TEST: press START to begin reporting");
}

void encoder_test_step (encoder_test_t * p_test)
{
    uint8_t buttons = board_io_buttons();
    bool b_start = ((0u != (buttons & ENCODER_TEST_START)) &&
                    (0u == (p_test->previous_buttons &
                            ENCODER_TEST_START)));
    bool b_reset = 0u != (buttons & ENCODER_TEST_RESET);
    uint32_t now = board_io_time_ms();
    p_test->previous_buttons = buttons;

    if (b_reset)
    {
        encoder_test_init(p_test);
        return;
    }

    if (b_start)
    {
        p_test->b_running      = true;
        p_test->last_report_ms = now;
        report("ENCODER TEST: reporting started");
    }

    if ((p_test->b_running) &&
        ((now - p_test->last_report_ms) >= ENCODER_TEST_PERIOD_MS))
    {
        uint32_t left  = 0u;
        uint32_t right = 0u;
        bool b_valid = board_io_encoders(&left, &right);
        int32_t left_delta  = (int32_t)(left - p_test->last_left);
        int32_t right_delta = (int32_t)(right - p_test->last_right);
        tm_printf((UB*)"ENCODER TEST: L=%u dL=%d R=%u dR=%d valid=%d\n",
                  (unsigned)left, (int)left_delta, (unsigned)right,
                  (int)right_delta, b_valid ? 1 : 0);
        p_test->last_left      = left;
        p_test->last_right     = right;
        p_test->last_report_ms = now;
    }
}

static void report (const char * p_message)
{
    tm_printf((UB*)"%s\n", (UB*)p_message);
}

/*** End of file. ***/

