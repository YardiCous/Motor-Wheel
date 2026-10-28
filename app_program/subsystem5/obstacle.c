/** @file obstacle.c
 * @brief No servo waits or echo busy loops are present in this state machine.
 */
#include "obstacle.h"

#include "../robot_config.h"

#include <string.h>

#define OBSTACLE_COARSE_POINTS 5u
#define OBSTACLE_FINE_POINTS 3u

/* Servo angles are in 3-degree units, with 30 facing straight ahead.
 * Coarse points are 2 units (6 degrees) apart, so they reach 12 degrees each
 * way, and the fine pass adds one more unit: never beyond 15 degrees. */
#define OBSTACLE_CENTRE 30u
#define OBSTACLE_STEP 2u
#define OBSTACLE_MAX_UNITS 5u

_Static_assert(((OBSTACLE_STEP * 2u) + 1u) <= OBSTACLE_MAX_UNITS,
               "servo sweep would exceed 15 degrees each way");

static const uint8_t g_coarse_angles[OBSTACLE_COARSE_POINTS] = {
    OBSTACLE_CENTRE - (OBSTACLE_STEP * 2u), OBSTACLE_CENTRE - OBSTACLE_STEP,
    OBSTACLE_CENTRE, OBSTACLE_CENTRE + OBSTACLE_STEP,
    OBSTACLE_CENTRE + (OBSTACLE_STEP * 2u)};

/** @brief The centre servo index is 30, corresponding to 90 degrees. */
void obstacle_init(obstacle_t * p_obstacle)
{
    memset(p_obstacle, 0, sizeof(*p_obstacle));
    p_obstacle->angle    = 30u;
    p_obstacle->range_cm = ROBOT_RANGE_MAX_CM;
}

/** @brief Start only after Navigation has stopped the motor controller. */
bool obstacle_scan(obstacle_t * p_obstacle)
{
    bool b_idle = (OBSTACLE_IDLE == p_obstacle->state);

    if (b_idle)
    {
        obstacle_init(p_obstacle);
        p_obstacle->state = OBSTACLE_COARSE;
        p_obstacle->angle = g_coarse_angles[0];
    }

    return b_idle;
}

/** @brief Missing echoes are invalid measurements, never clear space. */
void obstacle_sample(obstacle_t * p_obstacle, uint16_t range_cm, bool b_valid)
{
    b_valid = b_valid && (range_cm >= 2u) && (range_cm <= ROBOT_RANGE_MAX_CM);
    p_obstacle->b_valid  = b_valid;
    p_obstacle->range_cm = b_valid ? range_cm : 0u;
    bool b_scanning      = (OBSTACLE_COARSE == p_obstacle->state) ||
                           (OBSTACLE_FINE == p_obstacle->state);

    if (b_scanning && (!b_valid))
    {
        ++p_obstacle->retries;

        if (p_obstacle->retries >= 3u)
        {
            p_obstacle->state = OBSTACLE_ABORT;
            p_obstacle->angle = 30u;
        }
    }
    else if (OBSTACLE_COARSE == p_obstacle->state)
    {
        bool b_side = (30u != p_obstacle->angle);

        if (b_side && (range_cm > p_obstacle->best_cm))
        {
            p_obstacle->best_cm    = range_cm;
            p_obstacle->best_angle = p_obstacle->angle;
        }

        ++p_obstacle->coarse_index;

        if (p_obstacle->coarse_index < OBSTACLE_COARSE_POINTS)
        {
            p_obstacle->angle = g_coarse_angles[p_obstacle->coarse_index];
        }
        else
        {
            p_obstacle->state   = OBSTACLE_FINE;
            p_obstacle->best_cm = ROBOT_RANGE_MAX_CM;
            p_obstacle->angle   = p_obstacle->best_angle - 1u;
        }
    }
    else if (OBSTACLE_FINE == p_obstacle->state)
    {
        p_obstacle->best_cm =
            (range_cm < p_obstacle->best_cm) ? range_cm : p_obstacle->best_cm;
        ++p_obstacle->fine_index;

        if (p_obstacle->fine_index < OBSTACLE_FINE_POINTS)
        {
            p_obstacle->angle =
                p_obstacle->best_angle + p_obstacle->fine_index - 1u;
        }
        else
        {
            p_obstacle->b_left  = (p_obstacle->best_angle < 30u);
            p_obstacle->b_ready = (p_obstacle->best_cm >= OBSTACLE_CLEAR_CM);
            p_obstacle->state =
                p_obstacle->b_ready ? OBSTACLE_PLAN : OBSTACLE_ABORT;
            p_obstacle->angle = 30u;
        }
    }
    else
    {
        /* Forward monitoring does not alter a scan plan. */
    }
}

/** @brief Deadline saturation also prevents the 13-bit timer wrapping. */
void obstacle_tick(obstacle_t * p_obstacle, uint16_t elapsed_ms)
{
    bool b_scanning = (OBSTACLE_COARSE == p_obstacle->state) ||
                      (OBSTACLE_FINE == p_obstacle->state);

    if (b_scanning)
    {
        uint32_t elapsed = p_obstacle->scan_ms + elapsed_ms;
        p_obstacle->scan_ms =
            (elapsed > ROBOT_SCAN_MS) ? ROBOT_SCAN_MS : elapsed;

        if (p_obstacle->scan_ms >= ROBOT_SCAN_MS)
        {
            p_obstacle->state = OBSTACLE_ABORT;
            p_obstacle->angle = 30u;
        }
    }
}

/** @brief Sorting comparisons keep the complete 0..400-cm precision. */
uint16_t obstacle_median(uint16_t first, uint16_t second, uint16_t third)
{
    uint16_t temporary = 0u;

    if (first > second)
    {
        temporary = first;
        first     = second;
        second    = temporary;
    }

    if (second > third)
    {
        second = third;
    }

    if (first > second)
    {
        second = first;
    }

    return second;
}

/*** End of file. ***/

