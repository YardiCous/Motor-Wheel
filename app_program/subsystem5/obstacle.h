/** @file obstacle.h
 * @brief Stopped servo scanning and body-assisted width measurement.
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

#define OBSTACLE_IDLE 0u
#define OBSTACLE_COARSE 1u
#define OBSTACLE_FINE 2u
#define OBSTACLE_PLAN 3u
#define OBSTACLE_BYPASS 4u
#define OBSTACLE_RECOVER 5u
#define OBSTACLE_ABORT 6u
#define OBSTACLE_BODY_SCAN 7u
#define OBSTACLE_SCAN_POINTS 21u
#define OBSTACLE_BODY_POINTS 13u
#define OBSTACLE_REASON_NONE 0u
#define OBSTACLE_REASON_WIDER_SCAN 1u
#define OBSTACLE_REASON_NO_FRONT 2u
#define OBSTACLE_REASON_BLOCKED 3u
#define OBSTACLE_REASON_INVALID 4u
#define OBSTACLE_REASON_DEADLINE 5u
#define OBSTACLE_REASON_NO_EDGES 6u
/* The stopped scan accepts a front face up to 10 cm beyond the trigger
 * distance: the robot may stop or the reading may wobble a little past it. */
#define OBSTACLE_FRONT_MAX_MM (ROBOT_OBSTACLE_MM + 100u)
/* Closer than this during a scan is the robot itself, not the course. */
#define OBSTACLE_SELF_CM 10u


/** @brief The servo angle is represented in three-degree units. */
typedef struct
{
    uint32_t state : 3;
    uint32_t angle : 6;
    uint32_t best_angle : 6;
    uint32_t best_cm : 9;
    uint32_t range_cm : 9;
    uint32_t coarse_index : 5;
    uint32_t retries : 2;
    bool     b_left : 1;
    bool     b_valid : 1;
    bool     b_ready : 1;
    uint32_t scan_ms : 13;
    uint16_t scan_cm[OBSTACLE_SCAN_POINTS];
    uint16_t width_mm;
    uint16_t side_mm;
    uint16_t left_cm;
    uint16_t right_cm;
    uint8_t reason;
} obstacle_t;

/** @brief Reset the recoverable abort state and face the sensor forward. */
void obstacle_init(obstacle_t * p_obstacle);
/** @brief Begin a stationary scan; reject overlapping requests. */
bool obstacle_scan(obstacle_t * p_obstacle);
/** @brief Consume a 3-degree sweep sample and estimate both obstacle edges.
 */
void obstacle_sample(obstacle_t * p_obstacle, uint16_t range_cm, bool b_valid);
/** @brief Enforce the configured whole-scan deadline. */
void obstacle_tick(obstacle_t * p_obstacle, uint16_t elapsed_ms);
/** @brief Return an outlier-resistant median without adding readings. */
uint16_t obstacle_median(uint16_t first, uint16_t second, uint16_t third);

/** @brief Combine zero to three echoes; no echo reports the maximum range. */
uint16_t obstacle_range(const uint16_t * p_ranges, uint8_t echoes);

/** @brief Estimate width from body headings -90..90 in 15-degree steps.
 * Returns zero when either edge or the front face cannot be established.
 */
uint16_t obstacle_body_width(const uint16_t * p_ranges);
/** @brief Sideways travel that puts the robot clear of an edge this far
 * from its centre: edge + turning radius + the side margin.
 */
uint16_t obstacle_side_mm(uint32_t edge_mm);
/** @brief True when lateral free space fits the robot after side travel. */
bool obstacle_room(uint32_t lateral_mm, uint32_t side_mm);
/** @brief Describe the scan rejection or wider-scan request. */
const char * obstacle_reason_name(uint8_t reason);

#endif /* OBSTACLE_H */

/*** End of file. ***/

