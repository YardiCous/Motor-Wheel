/** @file barcode_test.h
 * @brief Motor-off GP28 barcode bench: raw capture, then Code 39 decoding.
 *
 * BARCODE_TEST=1 prints the raw sensor level and every bar and gap it sees.
 * BARCODE_TEST=2 prints the same and then decodes them into a letter.
 */
#ifndef BARCODE_TEST_H
#define BARCODE_TEST_H

#include "code39.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief One scan's bars and gaps, plus the live sensor statistics. */
typedef struct
{
    uint16_t widths_ms[CODE39_MAX_ELEMENTS];
    uint16_t widths_edges[CODE39_MAX_ELEMENTS];
    uint32_t edge_ms;
    uint32_t report_ms;
    uint32_t last_left;
    uint32_t last_right;
    uint32_t scans;
    uint32_t moved_edges;
    uint32_t moving_ms;
    uint32_t ignored;
    uint16_t low;
    uint16_t high;
    uint16_t longest_ms;
    uint8_t  count;
    uint8_t  phase;
    bool     b_black;
    bool     b_scanning;
    bool     b_overflow;
} barcode_test_t;

/** @brief Print the banner; phase 1 is raw capture, 2 adds decoding. */
void barcode_test_init(barcode_test_t * p_test, uint8_t phase);
/** @brief Take one GP28 sample; call every ROBOT_LINE_PERIOD_MS. */
void barcode_test_step(barcode_test_t * p_test);

#endif /* BARCODE_TEST_H */

/*** End of file. ***/

