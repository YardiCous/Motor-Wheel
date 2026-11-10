/** @file scanner.h
 * @brief Reads one Code 39 letter from GP28 while the robot follows the line.
 *
 * Recording starts at the first white-to-black edge seen while the wheels
 * turn. Every bar and gap is measured in encoder edges, so the robot's speed
 * does not matter. The scan ends after a long white quiet zone, or early
 * when the line sensors reach a junction (scanner_flush).
 */
#ifndef SCANNER_H
#define SCANNER_H

#include "code39.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Private to the line task. */
typedef struct
{
    uint16_t widths[CODE39_MAX_ELEMENTS];
    uint32_t last_left;
    uint32_t last_right;
    uint32_t element_edges;
    uint16_t longest;
    uint8_t  count;
    bool     b_black;
    bool     b_scanning;
    bool     b_started;
} scanner_t;

/** @brief Forget any scan; the first step only records the encoders. */
void scanner_init(scanner_t * p_reader);
/** @brief One GP28 sample with the wheel encoder totals. Returns the letter
 * 'A'..'Z' once when a scan decodes to one, otherwise '\0'. With b_enabled
 * false (turning, avoiding) any scan in progress is dropped.
 */
char scanner_step(scanner_t * p_reader, uint16_t raw,
                         uint16_t threshold, uint32_t left, uint32_t right,
                         bool b_enabled);
/** @brief End the scan now (junction reached): its letter or '\0'. */
char scanner_flush(scanner_t * p_reader);

#endif /* SCANNER_H */

/*** End of file. ***/

