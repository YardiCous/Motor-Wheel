/** @file obstacle.c
 * @brief No servo waits or echo busy loops are present in this state machine.
 */
#include "obstacle.h"

#include "../robot_config.h"

#include <string.h>

#define OBSTACLE_CENTRE 30u
#define OBSTACLE_FIRST_ANGLE 20u
#define OBSTACLE_MIDDLE_INDEX 10u
#define OBSTACLE_FACE_TOLERANCE_MM 50u

/* tan and cos for 0, 3, ... 30 degrees, scaled by 1000. */
static const uint16_t g_tangent[11] = {
    0u, 52u, 105u, 158u, 213u, 268u, 325u, 384u, 445u, 510u, 578u};
static const uint16_t g_cosine[11] = {
    1000u, 999u, 995u, 988u, 978u, 966u, 951u, 934u, 914u, 891u, 866u};

static void finish_scan(obstacle_t * p_obstacle);
static uint16_t edge_offset(const obstacle_t * p_obstacle, bool b_left);

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
        p_obstacle->angle = OBSTACLE_FIRST_ANGLE;
    }

    return b_idle;
}

/** @brief Invalid batches retry the current angle and never advance it. */
void obstacle_sample(obstacle_t * p_obstacle, uint16_t range_cm, bool b_valid)
{
    b_valid = b_valid && (range_cm >= 2u) && (range_cm <= ROBOT_RANGE_MAX_CM);
    p_obstacle->b_valid = b_valid;
    p_obstacle->range_cm = b_valid ? range_cm : 0u;
    bool b_scanning = (OBSTACLE_COARSE == p_obstacle->state);

    if (b_scanning && (!b_valid))
    {
        ++p_obstacle->retries;

        if (p_obstacle->retries >= 3u)
        {
            p_obstacle->state = OBSTACLE_ABORT;
            p_obstacle->reason = OBSTACLE_REASON_INVALID;
            p_obstacle->angle = OBSTACLE_CENTRE;
        }
    }
    else if (b_scanning)
    {
        p_obstacle->retries = 0u;
        p_obstacle->scan_cm[p_obstacle->coarse_index] = range_cm;
        ++p_obstacle->coarse_index;

        if (p_obstacle->coarse_index < OBSTACLE_SCAN_POINTS)
        {
            p_obstacle->angle =
                OBSTACLE_FIRST_ANGLE + p_obstacle->coarse_index;
        }
        else
        {
            finish_scan(p_obstacle);
        }
    }
    else
    {
        /* Forward monitoring does not alter a completed width estimate. */
    }
}

/** @brief Locate the first ray beyond each edge of the front-facing object.
 * A ray belongs to the obstacle while its forward projection is close to
 * the centre face. The first clear ray gives a conservative angular bound.
 */
static uint16_t edge_offset(const obstacle_t * p_obstacle, bool b_left)
{
    uint32_t front = p_obstacle->scan_cm[OBSTACLE_MIDDLE_INDEX] * 10u;

    for (uint8_t offset = 1u; offset <= OBSTACLE_MIDDLE_INDEX; ++offset)
    {
        uint8_t index = b_left ? (OBSTACLE_MIDDLE_INDEX - offset)
                              : (OBSTACLE_MIDDLE_INDEX + offset);
        uint32_t range = p_obstacle->scan_cm[index];
        uint32_t depth = (range * 10u * g_cosine[offset]) / 1000u;
        bool b_clear = (ROBOT_RANGE_MAX_CM == range) ||
                       (depth > (front + OBSTACLE_FACE_TOLERANCE_MM));

        if (b_clear)
        {
            return (uint16_t)((front * g_tangent[offset] + 999u) / 1000u);
        }
    }

    return 0u;
}

/** @brief Compare both sides; reject unbounded widths or blocked routes. */
static void finish_scan(obstacle_t * p_obstacle)
{
    uint16_t left = edge_offset(p_obstacle, true);
    uint16_t right = edge_offset(p_obstacle, false);
    uint16_t front = p_obstacle->scan_cm[OBSTACLE_MIDDLE_INDEX];
    p_obstacle->left_cm = p_obstacle->scan_cm[0];
    p_obstacle->right_cm = p_obstacle->scan_cm[OBSTACLE_SCAN_POINTS - 1u];
    p_obstacle->width_mm = left + right;
    uint32_t travel = p_obstacle->width_mm + ROBOT_BYPASS_SIDE_MARGIN_MM;
    bool b_left_clear = (p_obstacle->left_cm * 10u) > travel;
    bool b_right_clear = (p_obstacle->right_cm * 10u) > travel;
    bool b_width = (0u != left) && (0u != right) &&
                   ((front * 10u) <= ROBOT_OBSTACLE_MM);
    p_obstacle->b_ready = b_width && (b_left_clear || b_right_clear);
    p_obstacle->b_left = b_left_clear &&
                        ((!b_right_clear) ||
                         (p_obstacle->left_cm >= p_obstacle->right_cm));
    p_obstacle->best_angle = p_obstacle->b_left ? 20u : 40u;
    p_obstacle->best_cm = p_obstacle->b_left ? p_obstacle->left_cm
                                           : p_obstacle->right_cm;
    if ((front < 2u) || ((front * 10u) > ROBOT_OBSTACLE_MM))
    {
        p_obstacle->state = OBSTACLE_ABORT;
        p_obstacle->reason = OBSTACLE_REASON_NO_FRONT;
    }
    else
    {
        p_obstacle->state = p_obstacle->b_ready ? OBSTACLE_PLAN
                                               : OBSTACLE_BODY_SCAN;
        p_obstacle->reason = p_obstacle->b_ready ? OBSTACLE_REASON_NONE
                                                : OBSTACLE_REASON_WIDER_SCAN;
    }

    p_obstacle->angle = OBSTACLE_CENTRE;
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
            p_obstacle->reason = OBSTACLE_REASON_DEADLINE;
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

/** @brief Median of three echoes, the nearer of two, or the only one.
 * With no echo at all, nothing is within range, so it reports the maximum.
 */
uint16_t obstacle_range (const uint16_t * p_ranges, uint8_t echoes)
{
    uint16_t range = ROBOT_RANGE_MAX_CM;

    if (3u == echoes)
    {
        range = obstacle_median(p_ranges[0], p_ranges[1], p_ranges[2]);
    }
    else if (2u == echoes)
    {
        range = (p_ranges[0] < p_ranges[1]) ? p_ranges[0] : p_ranges[1];
    }
    else if (1u == echoes)
    {
        range = p_ranges[0];
    }
    else
    {
        /* No echo: nothing within the sensor's range. */
    }

    return range;
}
/** @brief Wider rays bound edges the limited servo sweep could not see. */
uint16_t obstacle_body_width (const uint16_t * p_ranges)
{
    static const uint16_t cosine[6] = {1000u, 966u, 866u, 707u, 500u, 259u};
    static const uint16_t tangent[6] = {0u, 268u, 578u, 1000u, 1733u, 3733u};
    uint32_t front = p_ranges[6] * 10u;
    uint32_t edges[2] = {0u, 0u};

    for (uint8_t index = 0u; index < OBSTACLE_BODY_POINTS; ++index)
    {
        if ((p_ranges[index] < 2u) ||
            (p_ranges[index] > ROBOT_RANGE_MAX_CM))
        {
            return 0u;
        }
    }

    if ((p_ranges[6] < 2u) || (front > ROBOT_OBSTACLE_MM))
    {
        return 0u;
    }

    for (uint8_t side = 0u; side < 2u; ++side)
    {
        for (uint8_t offset = 1u; offset < 6u; ++offset)
        {
            uint8_t index = (0u == side) ? (6u - offset) : (6u + offset);
            uint32_t range = p_ranges[index];
            uint32_t depth = (range * 10u * cosine[offset]) / 1000u;
            bool b_clear = (ROBOT_RANGE_MAX_CM == range) ||
                           (depth > (front + OBSTACLE_FACE_TOLERANCE_MM));

            if (b_clear)
            {
                edges[side] = (front * tangent[offset] + 999u) / 1000u;
                break;
            }
        }
    }

    return ((0u == edges[0]) || (0u == edges[1]))
               ? 0u : (uint16_t)(edges[0] + edges[1]);
}

/** @brief Keep stopped-scan failures distinct in the console. */
const char * obstacle_reason_name (uint8_t reason)
{
    static const char * const names[7] = {
        "none", "wider body scan required", "front face not confirmed",
        "both sides blocked", "invalid echo samples", "scan deadline",
        "obstacle edges not found"};
    return (reason < 7u) ? names[reason] : "unknown";
}

/*** End of file. ***/

