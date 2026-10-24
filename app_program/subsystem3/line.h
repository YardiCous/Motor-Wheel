/** @file line.h
 * @brief Two-channel line tracking and a distance-framed barcode reader.
 */
#ifndef LINE_H
#define LINE_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_CALIBRATING 0u
#define LINE_TRACKING 1u
#define LINE_SEARCH 2u
#define LINE_JUNCTION 3u
#define LINE_SCAN 4u
#define LINE_DECODE 5u

/** @brief Packed state is private to the 5-ms sensing task. */
typedef struct
{
    uint8_t  samples[5];
    uint32_t state : 3;
    uint32_t error : 8;
    uint32_t barcode : 2;
    bool     b_barcode_valid : 1;
    bool     b_last_black : 1;
    bool     b_line_valid : 1;
    bool     b_lost : 1;
    uint32_t segment : 3;
    uint32_t symbol : 2;
    uint32_t transition_mm : 12;
    uint32_t lost_ms : 8;
    uint32_t frame_ms : 10;
    uint32_t calibration_samples : 6;
    uint32_t junction_ms : 9;
    uint32_t threshold_left : 12;
    uint32_t threshold_centre : 12;
    uint32_t threshold_right : 12;
} line_t;

/** @brief Load provisional calibrated thresholds from robot_config.h. */
void line_init(line_t * p_line);
/** @brief Derive thresholds from measured white and black references. */
bool line_calibrate(line_t * p_line, const uint16_t * p_white,
                    const uint16_t * p_black);
/** @brief Process 12-bit ADC samples and modulo-4096 encoder distance. */
void line_step(line_t * p_line, const uint16_t * p_samples, uint16_t distance,
               bool b_planned_departure);
/** @brief Return steering -127..127, with validity stored separately. */
int16_t line_error(const line_t * p_line);

#endif /* LINE_H */

/*** End of file. ***/

