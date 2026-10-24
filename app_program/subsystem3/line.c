/** @file line.c
 * @brief GP26/27 track the line; GP28 reads the provisional barcode lane.
 * NOTE: docs/barcode_protocol.md defines the course-format assumption.
 */
#include "line.h"

#include "../data_formats.h"
#include "../robot_config.h"

#include <string.h>

#define LINE_FRAME_TIMEOUT_MS 1000u
#define LINE_CALIBRATION_COUNT 32u
#define LINE_BARCODE_SEGMENTS 6u

static void barcode_step(line_t * p_line, bool b_black, uint16_t distance);
static void barcode_transition(line_t * p_line, uint16_t width);
static bool narrow_bar(uint16_t width);

/** @brief A valid bit distinguishes an undecoded symbol from LEFT. */
void line_init (line_t * p_line)
{
    memset(p_line, 0, sizeof(*p_line));
    p_line->error            = 127u;
    p_line->threshold_left   = ROBOT_IR_THRESHOLD;
    p_line->threshold_centre = ROBOT_IR_THRESHOLD;
    p_line->threshold_right  = ROBOT_IR_THRESHOLD;
}

/** @brief Preserve all 12 ADC bits; reject insufficient contrast. */
bool line_calibrate (line_t * p_line, const uint16_t * p_white,
                    const uint16_t * p_black)
{
    bool     b_valid                    = true;
    uint16_t thresholds[ROBOT_IR_COUNT] = {0u};

    for (uint8_t sensor = 0u; sensor < ROBOT_IR_COUNT; ++sensor)
    {
        bool b_contrast =
            (p_black[sensor] > p_white[sensor]) && (p_black[sensor] <= 4095u);
        b_contrast =
            b_contrast && ((p_black[sensor] - p_white[sensor]) >= 200);
        b_valid            = b_valid && b_contrast;
        thresholds[sensor] = (p_black[sensor] + p_white[sensor]) / 2u;
    }

    if (b_valid)
    {
        p_line->threshold_left   = thresholds[0];
        p_line->threshold_centre = thresholds[1];
        p_line->threshold_right  = thresholds[2];
    }

    return b_valid;
}

/** @brief Barcode and junction suppression is bounded by explicit timers. */
void line_step (line_t * p_line, const uint16_t * p_samples, uint16_t distance,
               bool b_planned_departure)
{
    p_line->b_barcode_valid = false;

    for (uint8_t sensor = 0u; sensor < ROBOT_IR_COUNT; ++sensor)
    {
        (void)data_formats_write(p_line->samples, sensor * 12u, 12u,
                                 p_samples[sensor]);
    }

    bool b_left          = (p_samples[0] >= p_line->threshold_left);
    bool b_centre        = (p_samples[1] >= p_line->threshold_centre);
    bool b_right         = (p_samples[2] >= p_line->threshold_right);
    p_line->b_line_valid = b_left || b_centre;
    int32_t total        = p_samples[0] + p_samples[1];
    int32_t difference   = p_samples[1];
    difference -= p_samples[0];
    int32_t error = (0 == total) ? 0 : ((difference * 127) / total);
    /* Normalized steering is -127..127; its bias fits exactly in 8 bits. */
    p_line->error = (uint32_t)(error + 127);

    if (LINE_CALIBRATING == p_line->state)
    {
        ++p_line->calibration_samples;
        p_line->b_last_black  = b_right;
        p_line->transition_mm = distance;

        if (p_line->calibration_samples >= LINE_CALIBRATION_COUNT)
        {
            p_line->state = LINE_TRACKING;
        }
    }
    else
    {
        barcode_step(p_line, b_right, distance);
    }

    bool b_junction =
        b_left && b_centre && b_right && (LINE_SCAN != p_line->state);

    if (b_junction && (p_line->junction_ms < 300u))
    {
        p_line->junction_ms += ROBOT_LINE_PERIOD_MS;
        p_line->state = LINE_JUNCTION;
    }
    else if (LINE_JUNCTION == p_line->state)
    {
        p_line->state = LINE_TRACKING;
    }
    else
    {
        /* Barcode transitions own their own state changes. */
    }

    if (!b_junction)
    {
        p_line->junction_ms = 0u;
    }

    bool b_suppress = b_planned_departure || (LINE_SCAN == p_line->state) ||
                      (LINE_JUNCTION == p_line->state);

    if ((p_line->b_line_valid) || b_suppress)
    {
        p_line->lost_ms = 0u;
        p_line->b_lost  = false;

        if (LINE_SEARCH == p_line->state)
        {
            p_line->state = LINE_TRACKING;
        }
    }
    else
    {
        uint32_t elapsed = p_line->lost_ms + ROBOT_LINE_PERIOD_MS;
        p_line->lost_ms  = (elapsed > 150u) ? 150u : elapsed;
        p_line->b_lost   = (p_line->lost_ms >= ROBOT_LINE_LOST_MS);

        if (p_line->b_lost)
        {
            p_line->state = LINE_SEARCH;
        }
    }
}

/** @brief Offset binary avoids signed shifts in packed application data. */
int16_t line_error (const line_t * p_line)
{
    /* The field is 0..254, so subtracting its bias fits int16_t. */
    return (int16_t)((int32_t)p_line->error - 127);
}

/** @brief Consume right-sensor edges only after a valid 24..36-mm guard. */
static void barcode_step (line_t * p_line, bool b_black, uint16_t distance)
{
    bool    b_changed = (b_black != p_line->b_last_black);
    int16_t signed_width =
        data_formats_distance_delta(distance, p_line->transition_mm);
    bool b_forward = (signed_width >= 0);
    /* A forward segment fits the unsigned 12-bit distance field. */
    uint16_t width = b_forward ? (uint16_t)signed_width : 0u;

    if (LINE_SCAN == p_line->state)
    {
        p_line->frame_ms += ROBOT_LINE_PERIOD_MS;
    }

    bool b_timeout = (p_line->frame_ms >= LINE_FRAME_TIMEOUT_MS);

    if ((!b_forward) || b_timeout)
    {
        p_line->state = LINE_TRACKING;

        p_line->segment = 0u;
    }
    else if (b_changed)
    {
        barcode_transition(p_line, width);
    }
    else
    {
        /* A steady optical level carries no additional symbol. */
    }

    if (b_changed || (!b_forward))
    {
        p_line->transition_mm = distance;
        p_line->b_last_black  = b_black;
    }
}

/** @brief Odd segments are white gaps; even segments carry black symbols. */
static void barcode_transition (line_t * p_line, uint16_t width)
{
    bool b_black_ended = p_line->b_last_black;

    if (LINE_SCAN != p_line->state)
    {
        bool b_guard = b_black_ended && (width >= 24u) && (width <= 36u);

        if (b_guard && (p_line->b_line_valid))
        {
            p_line->state    = LINE_SCAN;
            p_line->segment  = 1u;
            p_line->symbol   = 0u;
            p_line->frame_ms = 0u;
        }
    }
    else
    {
        uint32_t segment = p_line->segment;
        bool     b_gap   = (0u != (segment & 1u));
        bool     b_wide  = (width >= 14u) && (width <= 22u);
        bool     b_valid =
            b_gap ? narrow_bar(width) : ((narrow_bar(width)) || b_wide);

        if (LINE_BARCODE_SEGMENTS == segment)
        {
            b_valid = (width >= 24u) && (width <= 36u);
        }

        if (!b_valid)
        {
            p_line->state = LINE_TRACKING;
        }
        else if (LINE_BARCODE_SEGMENTS == segment)
        {
            p_line->barcode         = p_line->symbol;
            p_line->b_barcode_valid = true;
            p_line->state           = LINE_TRACKING;
        }
        else
        {
            bool     b_data = (!b_gap) && b_black_ended;
            uint32_t bit    = b_wide ? 1u : 0u;
            p_line->symbol  = b_data ? (((uint32_t)p_line->symbol << 1u) | bit)
                                     : p_line->symbol;
            ++p_line->segment;
        }
    }
}

/** @brief The gap between narrow and wide bands rejects ambiguous widths. */
static bool narrow_bar (uint16_t width)
{
    return (width >= 4u) && (width <= 12u);
}

/*** End of file. ***/

