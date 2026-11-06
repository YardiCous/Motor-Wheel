/** @file code39.c
 * @brief Code 39 decoder: bar and gap widths in, characters out.
 */
#include "code39.h"

#include <string.h>

#define CODE39_SYMBOLS 44u
#define CODE39_WIDE_COUNT 3u
/* The narrowest wide element must be at least 5/4 of the widest narrow one;
 * printed Code 39 uses 2:1 to 3:1, which leaves room for hand-speed wobble.
 */
#define CODE39_RATIO_NUM 5u
#define CODE39_RATIO_DEN 4u

/** @brief One table row: the character and its nine wide/narrow flags. */
typedef struct
{
    char     symbol;
    uint16_t pattern;
} code39_symbol_t;

/* Bit 8 is the first bar; a set bit is a wide element. */
static const code39_symbol_t g_symbols[CODE39_SYMBOLS] = {
    {'1', 0x121u}, {'2', 0x061u}, {'3', 0x160u}, {'4', 0x031u}, {'5', 0x130u},
    {'6', 0x070u}, {'7', 0x025u}, {'8', 0x124u}, {'9', 0x064u}, {'0', 0x034u},
    {'A', 0x109u}, {'B', 0x049u}, {'C', 0x148u}, {'D', 0x019u}, {'E', 0x118u},
    {'F', 0x058u}, {'G', 0x00du}, {'H', 0x10cu}, {'I', 0x04cu}, {'J', 0x01cu},
    {'K', 0x103u}, {'L', 0x043u}, {'M', 0x142u}, {'N', 0x013u}, {'O', 0x112u},
    {'P', 0x052u}, {'Q', 0x007u}, {'R', 0x106u}, {'S', 0x046u}, {'T', 0x016u},
    {'U', 0x181u}, {'V', 0x0c1u}, {'W', 0x1c0u}, {'X', 0x091u}, {'Y', 0x190u},
    {'Z', 0x0d0u}, {'-', 0x085u}, {'.', 0x184u}, {' ', 0x0c4u}, {'*', 0x094u},
    {'$', 0x0a8u}, {'/', 0x0a2u}, {'+', 0x08au}, {'%', 0x02au},
};

static void    decode_direction(const uint16_t * p_widths, uint8_t count,
                                bool b_reversed, code39_result_t * p_result);
static bool    classify(const uint16_t * p_nine, code39_result_t * p_result);
static void    pick_letter(code39_result_t * p_result);
static uint8_t score(const code39_result_t * p_result);

/** @brief Forward first; the reverse reading wins only if it is cleaner. */
void code39_decode(const uint16_t * p_widths, uint8_t count,
                   code39_result_t * p_result)
{
    memset(p_result, 0, sizeof(*p_result));
    p_result->letter = '?';
    /* A cut-short scan still yields its whole characters: any count from 9
     * up decodes floor((count + 1) / 10) of them, from either end. */
    bool b_count_ok = (NULL != p_widths) && (count >= CODE39_ELEMENTS);
    count = (count > CODE39_MAX_ELEMENTS) ? CODE39_MAX_ELEMENTS : count;

    if (!b_count_ok)
    {
        p_result->status = CODE39_BAD_COUNT;
    }
    else
    {
        code39_result_t reverse = {0};
        decode_direction(p_widths, count, false, p_result);
        pick_letter(p_result);

        if (CODE39_OK != p_result->status)
        {
            decode_direction(p_widths, count, true, &reverse);
            pick_letter(&reverse);

            if (score(&reverse) < score(p_result))
            {
                *p_result = reverse;
            }
        }

        p_result->b_partial = (0u != ((count + 1u) % 10u));
    }
}

/** @brief Linear search keeps the table readable against the standard. */
char code39_char(uint16_t pattern)
{
    char symbol = '\0';

    for (uint8_t index = 0u; index < CODE39_SYMBOLS; ++index)
    {
        if (g_symbols[index].pattern == pattern)
        {
            symbol = g_symbols[index].symbol;
        }
    }

    return symbol;
}

/** @brief Ten elements per character: nine data plus one separator gap. */
static void decode_direction(const uint16_t * p_widths, uint8_t count,
                             bool b_reversed, code39_result_t * p_result)
{
    uint8_t chars = (uint8_t)((count + 1u) / 10u);
    memset(p_result, 0, sizeof(*p_result));
    p_result->letter     = '?';
    p_result->b_reversed = b_reversed;
    p_result->status     = CODE39_OK;

    for (uint8_t index = 0u;
         (index < chars) && (CODE39_OK == p_result->status); ++index)
    {
        uint16_t nine[CODE39_ELEMENTS] = {0u};

        for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
        {
            uint8_t position = (uint8_t)((index * 10u) + element);
            nine[element]    = b_reversed ? p_widths[count - 1u - position]
                                          : p_widths[position];
        }

        p_result->bad_char = index;

        if (!classify(nine, p_result))
        {
            p_result->status = CODE39_UNCLEAR;
        }
        else
        {
            char symbol = code39_char(p_result->pattern);

            if ('\0' == symbol)
            {
                p_result->status = CODE39_UNKNOWN;
            }
            else
            {
                p_result->text[index] = symbol;
                p_result->length      = (uint8_t)(index + 1u);
            }
        }
    }
}

/** @brief The three widest elements are wide; they must stand clear. */
static bool classify(const uint16_t * p_nine, code39_result_t * p_result)
{
    uint16_t sorted[CODE39_ELEMENTS] = {0u};
    (void)memcpy(sorted, p_nine, sizeof(sorted));

    /* Insertion sort, ascending: nine elements need no faster method. */
    for (uint8_t index = 1u; index < CODE39_ELEMENTS; ++index)
    {
        uint16_t value    = sorted[index];
        uint8_t  position = index;

        while ((position > 0u) && (sorted[position - 1u] > value))
        {
            sorted[position] = sorted[position - 1u];
            --position;
        }

        sorted[position] = value;
    }

    uint16_t narrow_max     = sorted[CODE39_ELEMENTS - CODE39_WIDE_COUNT - 1u];
    uint16_t wide_min       = sorted[CODE39_ELEMENTS - CODE39_WIDE_COUNT];
    p_result->widest_narrow = narrow_max;
    p_result->narrowest_wide = wide_min;
    p_result->pattern        = 0u;
    bool b_clear             = ((uint32_t)wide_min * CODE39_RATIO_DEN) >=
                               ((uint32_t)narrow_max * CODE39_RATIO_NUM);
    b_clear                  = b_clear && (wide_min > narrow_max);

    for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
    {
        uint16_t bit      = (p_nine[element] >= wide_min) ? 1u : 0u;
        p_result->pattern = (uint16_t)((p_result->pattern << 1u) | bit);
    }

    return b_clear;
}

/** @brief One '*' may frame either end ("*Z*", "*Z", "Z*" or "Z"); what
 * is left must be a single A..Z for the letter.
 */
static void pick_letter(code39_result_t * p_result)
{
    if (CODE39_OK == p_result->status)
    {
        uint8_t first = ('*' == p_result->text[0]) ? 1u : 0u;
        uint8_t last  = p_result->length;
        last          = ((last > first) && ('*' == p_result->text[last - 1u]))
                            ? (uint8_t)(last - 1u)
                            : last;
        char middle   = p_result->text[first];
        bool b_one    = (1u == (uint8_t)(last - first));

        if (b_one && (middle >= 'A') && (middle <= 'Z'))
        {
            p_result->letter = middle;
        }
        else
        {
            p_result->status = CODE39_NOT_LETTER;
        }
    }
}

/** @brief Rank a reading: status first, then how many ends carry '*'. A
 * '*' read backwards is 'P', so stars pick the true direction.
 */
static uint8_t score(const code39_result_t * p_result)
{
    uint8_t length = p_result->length;
    uint8_t stars  = ((length >= 1u) && ('*' == p_result->text[0])) ? 1u : 0u;
    stars +=
        ((length >= 2u) && ('*' == p_result->text[length - 1u])) ? 1u : 0u;
    return (uint8_t)((p_result->status * 3u) + (2u - stars));
}

/*** End of file. ***/

