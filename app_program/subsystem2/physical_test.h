/** @file physical_test.h
 * @brief Opt-in, low-speed motor sequence for bench verification.
 */
#ifndef PHYSICAL_TEST_H
#define PHYSICAL_TEST_H

#include "motion.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t  phase;
    uint8_t  previous_buttons;
    uint32_t pause_until_ms;
    bool     b_reported_ready;
} physical_test_t;

/** @brief Initialize the test in a motor-off state. */
void physical_test_init(physical_test_t * p_test);
/** @brief Run one 10-ms test cycle and return the requested PWM values. */
void physical_test_step(physical_test_t * p_test, motion_t * p_motion,
                        int16_t * p_left_pwm, int16_t * p_right_pwm);

#endif /* PHYSICAL_TEST_H */

/*** End of file. ***/

