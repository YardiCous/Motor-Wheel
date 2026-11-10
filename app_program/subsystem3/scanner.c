/** @file scanner.c
 * @brief Streaming Code 39 reader for GP28, widths in encoder edges.
 */
#include "scanner.h"

#include <string.h>

#define SCANNER_HYSTERESIS 100u
/* A first bar shorter than this (about 1 mm) is sensor noise. */
#define SCANNER_MIN_BAR_EDGES 12u
/* Black longer than this (about 100 mm) is tape, not a bar: drop the scan. */
#define SCANNER_MAX_BLACK_EDGES 1200u
/* The scan ends after this much white, or 3x the widest element if more. */
#define SCANNER_QUIET_EDGES 300u
#define SCANNER_QUIET_FACTOR 3u
#define SCANNER_WIDTH_LIMIT 65535u

static bool     sample_black(const scanner_t * p_reader, uint16_t raw,
                             uint16_t threshold);
static uint32_t moved(scanner_t * p_reader, uint32_t left, uint32_t right);
static char     scan_step(scanner_t * p_reader, bool b_black);
static void     push_element(scanner_t * p_reader);

/** @brief Encoder totals are taken on the first step. */
void scanner_init(scanner_t * p_reader)
{
    memset(p_reader, 0, sizeof(*p_reader));
}

/** @brief Idle until a bar starts while moving; then time each element. */
char scanner_step(scanner_t * p_reader, uint16_t raw, uint16_t threshold,
                  uint32_t left, uint32_t right, bool b_enabled)
{
    char     letter  = '\0';
    bool     b_black = sample_black(p_reader, raw, threshold);
    uint32_t edges   = moved(p_reader, left, right);

    if (!b_enabled)
    {
        p_reader->b_scanning = false;
    }
    else if (p_reader->b_scanning)
    {
        p_reader->element_edges += edges;
        letter = scan_step(p_reader, b_black);
    }
    else if (b_black && (!p_reader->b_black) && (0u != edges))
    {
        p_reader->b_scanning    = true;
        p_reader->count         = 0u;
        p_reader->longest       = 0u;
        p_reader->element_edges = 0u;
    }
    else
    {
        /* Waiting for the first bar. */
    }

    p_reader->b_black = b_black;
    return letter;
}

/** @brief Whole characters read so far are decoded; the rest is dropped. */
char scanner_flush(scanner_t * p_reader)
{
    char letter = '\0';

    if (p_reader->b_scanning)
    {
        code39_result_t result = {0};
        p_reader->b_scanning   = false;
        code39_decode(p_reader->widths, p_reader->count, &result);

        if (CODE39_OK == result.status)
        {
            letter = result.letter;
        }
    }

    return letter;
}

/** @brief Hysteresis stops a level near the threshold from chattering. */
static bool sample_black(const scanner_t * p_reader, uint16_t raw,
                         uint16_t threshold)
{
    uint32_t on  = (uint32_t)threshold + SCANNER_HYSTERESIS;
    uint32_t off = (threshold > SCANNER_HYSTERESIS)
                       ? ((uint32_t)threshold - SCANNER_HYSTERESIS)
                       : 0u;
    return p_reader->b_black ? ((uint32_t)raw >= off) : ((uint32_t)raw >= on);
}

/** @brief Mean absolute wheel movement since the last step, in edges. */
static uint32_t moved(scanner_t * p_reader, uint32_t left, uint32_t right)
{
    uint32_t edges = 0u;

    if (p_reader->b_started)
    {
        int32_t  delta_left  = (int32_t)(left - p_reader->last_left);
        int32_t  delta_right = (int32_t)(right - p_reader->last_right);
        uint32_t moved_left =
            (delta_left < 0) ? (uint32_t)(-delta_left) : (uint32_t)delta_left;
        uint32_t moved_right = (delta_right < 0) ? (uint32_t)(-delta_right)
                                                 : (uint32_t)delta_right;
        edges                = (moved_left + moved_right) / 2u;
    }

    p_reader->b_started  = true;
    p_reader->last_left  = left;
    p_reader->last_right = right;
    return edges;
}

/** @brief A colour change closes an element; long white ends the scan. */
static char scan_step(scanner_t * p_reader, bool b_black)
{
    char     letter = '\0';
    uint32_t width  = p_reader->element_edges;
    uint32_t quiet  = (uint32_t)p_reader->longest * SCANNER_QUIET_FACTOR;
    quiet = (quiet > SCANNER_QUIET_EDGES) ? quiet : SCANNER_QUIET_EDGES;

    if ((b_black != p_reader->b_black) && (0u == p_reader->count) &&
        (width < SCANNER_MIN_BAR_EDGES))
    {
        /* A blip, not a bar: wait for a real one. */
        p_reader->b_scanning = false;
    }
    else if (b_black != p_reader->b_black)
    {
        push_element(p_reader);

        if (p_reader->count >= CODE39_MAX_ELEMENTS)
        {
            letter = scanner_flush(p_reader);
        }
    }
    else if (b_black && (width > SCANNER_MAX_BLACK_EDGES))
    {
        p_reader->b_scanning = false;
    }
    else if ((!b_black) && (width >= quiet))
    {
        letter = scanner_flush(p_reader);
    }
    else
    {
        /* The current element is still growing. */
    }

    return letter;
}

/** @brief Store the element that just ended. */
static void push_element(scanner_t * p_reader)
{
    uint32_t width = p_reader->element_edges;
    width = (width > SCANNER_WIDTH_LIMIT) ? SCANNER_WIDTH_LIMIT : width;
    p_reader->widths[p_reader->count] = (uint16_t)width;
    ++p_reader->count;
    p_reader->longest       = ((uint16_t)width > p_reader->longest)
                                  ? (uint16_t)width
                                  : p_reader->longest;
    p_reader->element_edges = 0u;
}

/*** End of file. ***/

