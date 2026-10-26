/** @file encoder_test.h
 * @brief Motor-off quadrature encoder diagnostic.
 */
#ifndef ENCODER_TEST_H
#define ENCODER_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t last_report_ms;
    uint32_t last_left;
    uint32_t last_right;
    uint8_t previous_buttons;
    uint8_t b_running : 1;
} encoder_test_t;

/** @brief Initialize the motor-off encoder diagnostic. */
void encoder_test_init(encoder_test_t * p_test);
/** @brief Print encoder snapshots while the operator rotates the wheels. */
void encoder_test_step(encoder_test_t * p_test);

#endif /* ENCODER_TEST_H */

/*** End of file. ***/

