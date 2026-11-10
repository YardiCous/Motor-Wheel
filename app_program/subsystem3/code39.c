/** @file code39.c
 * @brief Code 39 decoder: bar and gap widths in, characters out.
 */
#include "code39.h"

#include <string.h>

#define CODE39_SYMBOLS 44u
#define CODE39_WIDE_COUNT 3u
#define CODE39_BARS 5u
#define CODE39_GAPS 4u
#define CODE39_WIDE_BARS 2u
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
static bool    split(const uint16_t * p_widths, uint8_t count, uint8_t wide,
                     uint16_t * p_narrow_max, uint16_t * p_wide_min);
static void    pick_letter(code39_result_t * p_result);
static uint8_t score(const code39_result_t * p_result);
static bool    better(const code39_result_t * p_new,
                      const code39_result_t * p_old);

/** @brief Junk before or after the barcode (a hand, the start position, a
 * bar edge while stopped) shifts every character, so try each bar as the
 * first one: forwards dropping leading elements, backwards dropping trailing
 * ones. The cleanest reading wins; ties go to more characters, then to the
 * fewest elements dropped.
 */
void code39_decode(const uint16_t * p_widths, uint8_t count,
                   code39_result_t * p_result)
{
    memset(p_result, 0, sizeof(*p_result));
    p_result->letter = '?';
    p_result->status = CODE39_BAD_COUNT;
    count = (count > CODE39_MAX_ELEMENTS) ? CODE39_MAX_ELEMENTS : count;

    if ((NULL == p_widths) || (count < CODE39_ELEMENTS))
    {
        return;
    }

    bool b_have = false;

    for (uint8_t skip = 0u; ((uint32_t)skip + CODE39_ELEMENTS) <= count;
         skip         = (uint8_t)(skip + 2u))
    {
        for (uint8_t pass = 0u; pass < 2u; ++pass)
        {
            code39_result_t  candidate = {0};
            bool             b_reverse = (1u == pass);
            uint8_t          used      = (uint8_t)(count - skip);
            const uint16_t * p_start =
                b_reverse ? p_widths : (p_widths + skip);
            decode_direction(p_start, used, b_reverse, &candidate);
            pick_letter(&candidate);
            candidate.b_partial = (candidate.b_partial) ||
                                  (0u != ((used + 1u) % 10u)) || (0u != skip);
            candidate.skipped   = skip;

            /* After dropping elements, only a reading that contains the
             * '*' start/stop mark is believable: shifted junk can decode
             * to a lone letter by chance. */
            bool b_star    = (0u != candidate.length) &&
                             (('*' == candidate.text[0]) ||
                              ('*' == candidate.text[candidate.length - 1u]));
            bool b_allowed = (0u == skip) || b_star;

            if (b_allowed && ((!b_have) || (better(&candidate, p_result))))
            {
                *p_result = candidate;
                b_have    = true;
            }
        }
    }
}

/** @brief Lower score first, then more characters; earlier wins ties. */
static bool better(const code39_result_t * p_new,
                   const code39_result_t * p_old)
{
    uint8_t new_score = score(p_new);
    uint8_t old_score = score(p_old);
    return (new_score < old_score) ||
           ((new_score == old_score) && (p_new->length > p_old->length));
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

/** @brief Bars and gaps are judged apart first: the sensor's spot and the
 * threshold make every bar read narrower (or wider) than a gap of the same
 * printed width. Each A..Z and '*' has 2 wide bars of 5 and 1 wide gap of 4.
 * If that fails ('$', '/', '+', '%' have 3 wide gaps), the 3 widest of all
 * nine are wide.
 */
static bool classify(const uint16_t * p_nine, code39_result_t * p_result)
{
    uint16_t bars[CODE39_BARS] = {0u};
    uint16_t gaps[CODE39_GAPS] = {0u};
    uint16_t bar_narrow        = 0u;
    uint16_t bar_wide          = 0u;
    uint16_t gap_narrow        = 0u;
    uint16_t gap_wide          = 0u;
    uint16_t all_narrow        = 0u;
    uint16_t all_wide          = 0u;

    for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
    {
        uint8_t slot = (uint8_t)(element / 2u);

        if (0u == (element & 1u))
        {
            bars[slot] = p_nine[element];
        }
        else
        {
            gaps[slot] = p_nine[element];
        }
    }

    bool b_split =
        split(bars, CODE39_BARS, CODE39_WIDE_BARS, &bar_narrow, &bar_wide);
    b_split =
        (split(gaps, CODE39_GAPS, 1u, &gap_narrow, &gap_wide)) && (b_split);
    uint16_t pattern = 0u;

    for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
    {
        uint16_t wide = (0u == (element & 1u)) ? bar_wide : gap_wide;
        uint16_t bit  = (p_nine[element] >= wide) ? 1u : 0u;
        pattern       = (uint16_t)((pattern << 1u) | bit);
    }

    b_split                 = (b_split) && ('\0' != code39_char(pattern));
    bool b_clear            = split(p_nine, CODE39_ELEMENTS, CODE39_WIDE_COUNT,
                                    &all_narrow, &all_wide);
    p_result->widest_narrow = b_split ? bar_narrow : all_narrow;
    p_result->narrowest_wide = b_split ? bar_wide : all_wide;
    p_result->pattern        = pattern;

    if (!b_split)
    {
        p_result->pattern = 0u;

        for (uint8_t element = 0u; element < CODE39_ELEMENTS; ++element)
        {
            uint16_t bit      = (p_nine[element] >= all_wide) ? 1u : 0u;
            p_result->pattern = (uint16_t)((p_result->pattern << 1u) | bit);
        }
    }

    return b_split || b_clear;
}

/** @brief The `wide` largest of `count` widths are wide; they must be at
 * least 5/4 of the widest narrow one.
 */
static bool split(const uint16_t * p_widths, uint8_t count, uint8_t wide,
                  uint16_t * p_narrow_max, uint16_t * p_wide_min)
{
    uint16_t sorted[CODE39_ELEMENTS] = {0u};
    (void)memcpy(sorted, p_widths, (size_t)count * sizeof(sorted[0]));

    /* Insertion sort, ascending: nine elements need no faster method. */
    for (uint8_t index = 1u; index < count; ++index)
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

    *p_narrow_max = sorted[count - wide - 1u];
    *p_wide_min   = sorted[count - wide];
    bool b_clear  = ((uint32_t)*p_wide_min * CODE39_RATIO_DEN) >=
                    ((uint32_t)*p_narrow_max * CODE39_RATIO_NUM);
    return b_clear && (*p_wide_min > *p_narrow_max);
}

/** @brief One '*' may frame either end ("*Z*", "*Z", "Z*" or "Z"); what
 * is left must be a single A..Z for the letter. "*Z" followed by a misread
 * character also gives Z.
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

    /* "*Z" then a misread or unclear character: the start mark and the
     * letter after it are enough, so keep those two and drop the rest. */
    char second = p_result->text[1];

    if ((CODE39_OK != p_result->status) && (p_result->length >= 2u) &&
        ('*' == p_result->text[0]) && (second >= 'A') && (second <= 'Z'))
    {
        (void)memset(&p_result->text[2], 0, sizeof(p_result->text) - 2u);
        p_result->length    = 2u;
        p_result->status    = CODE39_OK;
        p_result->letter    = second;
        p_result->b_partial = true;
    }
}

/** @brief Rank a reading (lower is better): '*' marks first, then status.
 * A '*' read backwards is 'P', so stars pick the true direction and the
 * true alignment; a framed non-letter still beats an unframed accident.
 */
static uint8_t score(const code39_result_t * p_result)
{
    uint8_t length = p_result->length;
    uint8_t stars  = ((length >= 1u) && ('*' == p_result->text[0])) ? 1u : 0u;
    stars +=
        ((length >= 2u) && ('*' == p_result->text[length - 1u])) ? 1u : 0u;
    return (uint8_t)(((2u - stars) * 8u) + p_result->status);
}

/*** End of file. ***/

