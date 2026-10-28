/** @file obstacle.h
 * @brief Coarse/fine scanning and bounded obstacle-recovery state.
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

/** @brief Minimum clear distance (cm) a side needs before a bypass is planned.
 * Keep it above the box reading at the stop (about 40 cm), or a reading of
 * the box itself can pass as clear. */
#define OBSTACLE_CLEAR_CM 40u

/** @brief The servo angle is represented in three-degree units. */
typedef struct
{
    uint32_t state : 3;
    uint32_t angle : 6;
    uint32_t best_angle : 6;
    uint32_t best_cm : 9;
    uint32_t range_cm : 9;
    uint32_t coarse_index : 3;
    uint32_t fine_index : 2;
    uint32_t retries : 2;
    bool     b_left : 1;
    bool     b_valid : 1;
    bool     b_ready : 1;
    uint32_t scan_ms : 13;
} obstacle_t;

/** @brief Reset the recoverable abort state and face the sensor forward. */
void obstacle_init(obstacle_t * p_obstacle);
/** @brief Begin a stationary scan; reject overlapping requests. */
bool obstacle_scan(obstacle_t * p_obstacle);
/** @brief Consume one median-of-three range after servo settling. */
void obstacle_sample(obstacle_t * p_obstacle, uint16_t range_cm, bool b_valid);
/** @brief Enforce a five-second whole-scan deadline. */
void obstacle_tick(obstacle_t * p_obstacle, uint16_t elapsed_ms);
/** @brief Return an outlier-resistant median without adding readings. */
uint16_t obstacle_median(uint16_t first, uint16_t second, uint16_t third);

#endif /* OBSTACLE_H */

/*** End of file. ***/

