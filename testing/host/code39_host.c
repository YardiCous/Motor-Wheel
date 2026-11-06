/** @file code39_host.c
 * @brief Host test for the Code 39 decoder with synthetic bar widths.
 * Build: gcc -std=c99 -Wall -Wextra -Werror testing/host/code39_host.c
 *        app_program/subsystem3/code39.c -o code39_host && ./code39_host
 */
#include "code39_host.h"

#include "../../app_program/subsystem3/code39.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define HOST_NARROW 10u
#define HOST_WIDE 25u
#define HOST_PATTERNS 512u

static uint16_t pattern_of(char symbol);
static uint8_t  build(const char * p_text, uint16_t * p_widths);
static void check_letter(const uint16_t * p_widths, uint8_t count, char letter,
                         bool b_reversed);

/** @brief Every assertion names one decoder contract. */
int main(void)
{
    uint16_t widths[CODE39_MAX_ELEMENTS]   = {0u};
    uint16_t backward[CODE39_MAX_ELEMENTS] = {0u};
    char     text[4]                       = "*A*";

    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        text[1]       = letter;
        uint8_t count = build(text, widths);
        assert(29u == count);
        check_letter(widths, count, letter, false);

        for (uint8_t index = 0u; index < count; ++index)
        {
            backward[index] = widths[count - 1u - index];
        }

        check_letter(backward, count, letter, true);

        /* Pushing speed drifts from 1x to 1.6x across the barcode. */
        for (uint8_t index = 0u; index < count; ++index)
        {
            uint32_t scaled =
                ((uint32_t)widths[index] * (100u + (2u * index)));
            backward[index] = (uint16_t)(scaled / 100u);
        }

        check_letter(backward, count, letter, false);
    }

    code39_result_t result = {0};
    uint8_t         count  = build("*K*", widths);
    /* Cut short by one element: the whole "*K" still gives K. */
    code39_decode(widths, (uint8_t)(count - 1u), &result);
    assert(CODE39_OK == result.status);
    assert(result.b_partial);
    assert('K' == result.letter);
    /* The 6 Nov robot scan: 25 elements of "*Z*" -> "*Z" -> Z. */
    count = build("*Z*", widths);
    code39_decode(widths, 25u, &result);
    assert(CODE39_OK == result.status);
    assert(0 == strcmp(result.text, "*Z"));
    assert('Z' == result.letter);
    /* One-star barcodes, both ways round. */
    count = build("*Z", widths);
    assert(19u == count);
    check_letter(widths, count, 'Z', false);
    count = build("Z*", widths);
    check_letter(widths, count, 'Z', false);
    code39_decode(widths, 8u, &result);
    assert(CODE39_BAD_COUNT == result.status);

    (void)memset(widths, 0, sizeof(widths));
    for (uint8_t index = 0u; index < 29u; ++index)
    {
        widths[index] = HOST_NARROW;
    }
    code39_decode(widths, 29u, &result);
    assert(CODE39_UNCLEAR == result.status);

    count = build("*1*", widths);
    code39_decode(widths, count, &result);
    assert(CODE39_NOT_LETTER == result.status);
    assert(0 == strcmp(result.text, "*1*"));

    count = build("Q", widths);
    assert(9u == count);
    check_letter(widths, count, 'Q', false);

    (void)puts("PASS: Code 39 A-Z forwards, backwards, speed drift, "
               "partial scans, *Z / Z*, unclear widths, non-letter, "
               "unframed");
    return 0;
}

/** @brief Reverse lookup through the decoder's own table. */
static uint16_t pattern_of(char symbol)
{
    uint16_t found = 0u;

    for (uint16_t pattern = 0u; pattern < HOST_PATTERNS; ++pattern)
    {
        if (symbol == code39_char(pattern))
        {
            found = pattern;
        }
    }

    assert(0u != found);
    return found;
}

/** @brief Nine elements per character plus one narrow separator gap. */
static uint8_t build(const char * p_text, uint16_t * p_widths)
{
    uint8_t count = 0u;

    for (uint8_t index = 0u; '\0' != p_text[index]; ++index)
    {
        uint16_t pattern = pattern_of(p_text[index]);

        if (index > 0u)
        {
            p_widths[count] = HOST_NARROW;
            ++count;
        }

        for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
        {
            uint32_t shift  = 8u - (uint32_t)element;
            bool     b_wide = (0u != (((uint32_t)pattern >> shift) & 1u));
            p_widths[count] = b_wide ? HOST_WIDE : HOST_NARROW;
            ++count;
        }
    }

    return count;
}

/** @brief The decoder must give this letter, read in this direction. */
static void check_letter(const uint16_t * p_widths, uint8_t count, char letter,
                         bool b_reversed)
{
    code39_result_t result = {0};
    code39_decode(p_widths, count, &result);
    assert(CODE39_OK == result.status);
    assert(letter == result.letter);
    assert(b_reversed == result.b_reversed);
}

/*** End of file. ***/

