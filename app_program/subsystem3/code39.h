/** @file code39.h
 * @brief Code 39 decoder: bar and gap widths in, characters out.
 *
 * Each Code 39 character is nine elements (five black bars and four white
 * gaps, starting with a bar). Exactly three of the nine are wide. Characters
 * are separated by one narrow white gap, and a printed barcode is framed by
 * the '*' start/stop character, so "*A*" is 3 x 9 + 2 = 29 elements.
 *
 * Widths may be in any unit (milliseconds or encoder edges): each character
 * is classified on its own, by picking its three widest elements, so a
 * steady change in speed across the barcode does not matter.
 */
#ifndef CODE39_H
#define CODE39_H

#include <stdbool.h>
#include <stdint.h>

#define CODE39_ELEMENTS 9u
#define CODE39_MAX_TEXT 6u
#define CODE39_MAX_ELEMENTS ((CODE39_MAX_TEXT * 10u) - 1u)

/* Status codes, best first: a lower code means the reading got further. */
#define CODE39_OK 0u
#define CODE39_NOT_LETTER 1u
#define CODE39_UNKNOWN 2u
#define CODE39_UNCLEAR 3u
#define CODE39_BAD_COUNT 4u /* fewer than one character's 9 elements */

/** @brief Everything the bench needs to explain a pass or a failure. */
typedef struct
{
    char     text[CODE39_MAX_TEXT + 1u];
    uint8_t  length;
    uint8_t  status;
    uint8_t  bad_char;
    uint16_t pattern;
    uint16_t widest_narrow;
    uint16_t narrowest_wide;
    bool     b_reversed;
    bool     b_partial;
    uint8_t  skipped;
    char     letter;
} code39_result_t;

/** @brief Decode alternating bar/gap widths that start on a bar. Tries
 * the forward reading first, then the reverse (robot travelling the other
 * way). A count that is not 9, 19, 29... is a cut-short scan: its whole
 * characters are still decoded and b_partial is set. letter is 'A'..'Z'
 * for "*X*", "*X", "X*" or a lone X, otherwise '?'.
 */
void code39_decode(const uint16_t * p_widths, uint8_t count,
                   code39_result_t * p_result);
/** @brief Map a 9-bit wide/narrow pattern (first element = bit 8) to its
 * character, or return '\0' when no Code 39 character uses it.
 */
char code39_char(uint16_t pattern);

#endif /* CODE39_H */

/*** End of file. ***/

