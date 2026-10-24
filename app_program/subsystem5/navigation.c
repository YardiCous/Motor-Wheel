/** @file navigation.c
 * @brief Multi-stage moves use the single-owner motion API.
 */
#include "navigation.h"

#include "../data_formats.h"
#include "../robot_config.h"
#include "../subsystem2/motion.h"

#include <string.h>

#define NAVIGATION_BYPASS_STAGES 8u
#define NAVIGATION_QUARTER_ARC_MM ((ROBOT_TRACK_MM * 3142u) / 4000u)

static void issue(navigation_t * p_navigation, uint8_t mode, uint16_t distance,
                  uint16_t speed, bool b_reverse);
static void step_bypass(navigation_t * p_navigation, bool b_done,
                        bool b_line_valid, bool b_scan_ready,
                        bool b_scan_left);
static void step_follow(navigation_t * p_navigation, bool b_line_lost,
                        bool b_on_hump, uint16_t range_cm, bool b_range_valid,
                        bool b_barcode_zone);

/** @brief No network state participates in boot readiness. */
void navigation_init (navigation_t * p_navigation)
{
    memset(p_navigation, 0, sizeof(*p_navigation));
    p_navigation->state = DATA_FORMATS_IDLE;
}

/** @brief STOP is valid in every state and flushes motion at the OS boundary.
 */
bool navigation_supervise (navigation_t * p_navigation, uint8_t command,
                          bool b_safety_latched)
{
    bool b_accepted = false;

    switch (command)
    {
        case DATA_FORMATS_STOP:
            p_navigation->b_stop_requested = true;
            p_navigation->state            = DATA_FORMATS_IDLE;
            p_navigation->b_command_ready  = false;
            p_navigation->b_waiting        = false;
            p_navigation->elapsed_ms       = 0u;
            b_accepted                     = true;
        break;

        case DATA_FORMATS_START:
            b_accepted = (DATA_FORMATS_IDLE == p_navigation->state) &&
                         (!b_safety_latched);

            if (b_accepted)
            {
                p_navigation->state  = DATA_FORMATS_FOLLOW;
                p_navigation->b_slow = false;
                issue(p_navigation, MOTION_FOLLOW, 0u, ROBOT_CRUISE_MM_SECOND,
                      false);
            }
        break;

        case DATA_FORMATS_RESET:
            b_accepted = (DATA_FORMATS_FAULT == p_navigation->state) ||
                         (DATA_FORMATS_IDLE == p_navigation->state);
            p_navigation->b_reset_requested = b_accepted;
        break;

        case DATA_FORMATS_ESTOP:
            p_navigation->b_estop          = true;
            p_navigation->b_stop_requested = true;
            p_navigation->state            = DATA_FORMATS_FAULT;
            b_accepted                     = true;
        break;

        default:
            /* Invalid supervisory commands cannot affect movement. */
        break;
    }

    return b_accepted;
}

/** @brief U-turn is implemented as twice the calibrated 90-degree arc. */
void navigation_barcode (navigation_t * p_navigation, uint8_t barcode)
{
    if ((DATA_FORMATS_FOLLOW == p_navigation->state) && (barcode <= 3u))
    {
        p_navigation->state      = DATA_FORMATS_BARCODE;
        p_navigation->barcode    = barcode;
        p_navigation->elapsed_ms = 0u;
        uint16_t distance        = NAVIGATION_QUARTER_ARC_MM;
        distance =
            (DATA_FORMATS_UTURN == barcode) ? (distance * 2u) : distance;
        uint8_t mode =
            (DATA_FORMATS_STRAIGHT == barcode) ? MOTION_MOVE : MOTION_TURN;
        distance = (DATA_FORMATS_STRAIGHT == barcode) ? 120u : distance;
        bool b_left =
            (DATA_FORMATS_LEFT == barcode) || (DATA_FORMATS_UTURN == barcode);
        issue(p_navigation, mode, distance, ROBOT_TURN_MM_SECOND, b_left);
        p_navigation->b_waiting = true;
    }
}

/** @brief Safety always wins over barcode completion or a pending bypass. */
void navigation_step (navigation_t * p_navigation, uint8_t motion_mode,
                     bool b_done, bool b_line_valid, bool b_line_lost,
                     bool b_on_hump, uint16_t range_cm, bool b_range_valid,
                     bool b_scan_ready, bool b_scan_left, bool b_barcode_zone,
                     bool b_safety_latched)
{
    if (b_safety_latched)
    {
        p_navigation->state            = DATA_FORMATS_FAULT;
        p_navigation->b_stop_requested = true;
    }

    bool b_timed = (DATA_FORMATS_BARCODE == p_navigation->state) ||
                   (DATA_FORMATS_AVOID == p_navigation->state) ||
                   (DATA_FORMATS_SEARCH == p_navigation->state);

    if (b_timed)
    {
        uint32_t elapsed = p_navigation->elapsed_ms + ROBOT_MOTION_PERIOD_MS;
        p_navigation->elapsed_ms = (elapsed > 10000u) ? 10000u : elapsed;
    }

    switch (p_navigation->state)
    {
        case DATA_FORMATS_FOLLOW:
        case DATA_FORMATS_HUMP:
            step_follow(p_navigation, b_line_lost, b_on_hump, range_cm,
                        b_range_valid, b_barcode_zone);
        break;

        case DATA_FORMATS_BARCODE:
            if (b_done && (MOTION_IDLE == motion_mode))
            {
                p_navigation->state = DATA_FORMATS_FOLLOW;
                issue(p_navigation, MOTION_FOLLOW, 0u,
                      ROBOT_CAUTIOUS_MM_SECOND, false);
            }
        break;

        case DATA_FORMATS_AVOID:
            step_bypass(p_navigation, b_done, b_line_valid, b_scan_ready,
                        b_scan_left);
        break;

        case DATA_FORMATS_SEARCH:
            p_navigation->line_confirm =
                b_line_valid ? ((p_navigation->line_confirm < 3u)
                                    ? (p_navigation->line_confirm + 1u)
                                    : 3u)
                             : 0u;

            if (p_navigation->line_confirm >= 3u)
            {
                p_navigation->b_stop_requested = true;
                p_navigation->state            = DATA_FORMATS_FOLLOW;
                issue(p_navigation, MOTION_FOLLOW, 0u,
                      ROBOT_CAUTIOUS_MM_SECOND, false);
            }
            else if (p_navigation->elapsed_ms >= ROBOT_SEARCH_MS)
            {
                p_navigation->b_fault = true;
            }
            else
            {
                /* A bounded encoder-controlled turn searches for the line. */
            }
        break;

        default:
            /* IDLE and FAULT generate no movement. */
        break;
    }

    if (p_navigation->elapsed_ms >= ROBOT_MANOEUVRE_MS)
    {
        p_navigation->b_fault = true;
    }
}

/** @brief Queue overflow is handled by the RTOS boundary as a fatal fault. */
static void issue (navigation_t * p_navigation, uint8_t mode, uint16_t distance,
                  uint16_t speed, bool b_reverse)
{
    p_navigation->b_command_ready = motion_command(
        &p_navigation->command, mode, distance, speed, b_reverse);
}

/** @brief Three straight legs plus turns return beside the original line. */
static void step_bypass (navigation_t * p_navigation, bool b_done,
                        bool b_line_valid, bool b_scan_ready, bool b_scan_left)
{
    if ((0u == p_navigation->stage) && b_scan_ready)
    {
        p_navigation->b_left     = b_scan_left;
        p_navigation->stage      = 1u;
        p_navigation->b_waiting  = false;
        p_navigation->elapsed_ms = 0u;
    }

    if ((p_navigation->b_waiting) && b_done)
    {
        ++p_navigation->stage;
        p_navigation->b_waiting  = false;
        p_navigation->elapsed_ms = 0u;
    }

    uint32_t stage = p_navigation->stage;

    if ((stage > 0u) && (stage < NAVIGATION_BYPASS_STAGES) &&
        (!p_navigation->b_waiting))
    {
        bool b_turn = (0u != (stage & 1u));
        bool b_left = p_navigation->b_left;
        b_left      = ((3u == stage) || (5u == stage)) ? !b_left : b_left;
        uint16_t distance =
            (4u == stage) ? ROBOT_BYPASS_FORWARD_MM : ROBOT_BYPASS_SIDE_MM;
        distance     = b_turn ? NAVIGATION_QUARTER_ARC_MM : distance;
        uint8_t mode = b_turn ? MOTION_TURN : MOTION_MOVE;
        issue(p_navigation, mode, distance, ROBOT_TURN_MM_SECOND,
              b_turn && b_left);
        p_navigation->b_waiting = true;
    }

    if (stage >= NAVIGATION_BYPASS_STAGES)
    {
        p_navigation->state        = DATA_FORMATS_SEARCH;
        p_navigation->elapsed_ms   = 0u;
        p_navigation->line_confirm = b_line_valid ? 1u : 0u;
        issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM,
              ROBOT_TURN_MM_SECOND, true);
    }
}

/** @brief The robot slows before entering an obstacle or hump zone. */
static void step_follow (navigation_t * p_navigation, bool b_line_lost,
                        bool b_on_hump, uint16_t range_cm, bool b_range_valid,
                        bool b_barcode_zone)
{
    bool b_obstacle = b_range_valid && ((range_cm * 10u) <= ROBOT_OBSTACLE_MM);
    bool b_slow = b_on_hump || b_barcode_zone ||
                  (b_range_valid && ((range_cm * 10u) <= ROBOT_WARNING_MM));

    if (b_obstacle && b_on_hump)
    {
        p_navigation->b_estop          = true;
        p_navigation->b_stop_requested = true;
        p_navigation->state            = DATA_FORMATS_FAULT;
    }
    else if (b_obstacle)
    {
        p_navigation->state            = DATA_FORMATS_AVOID;
        p_navigation->b_stop_requested = true;
        p_navigation->b_scan_requested = true;
        p_navigation->stage            = 0u;
        p_navigation->elapsed_ms       = 0u;
        p_navigation->b_waiting        = false;
    }
    else if (b_line_lost)
    {
        p_navigation->state      = DATA_FORMATS_SEARCH;
        p_navigation->elapsed_ms = 0u;
        issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM,
              ROBOT_TURN_MM_SECOND, true);
    }
    else
    {
        p_navigation->state =
            b_on_hump ? DATA_FORMATS_HUMP : DATA_FORMATS_FOLLOW;

        if (b_slow != p_navigation->b_slow)
        {
            uint16_t speed =
                b_slow ? ROBOT_CAUTIOUS_MM_SECOND : ROBOT_CRUISE_MM_SECOND;
            issue(p_navigation, MOTION_FOLLOW, 0u, speed, false);
            p_navigation->b_slow = b_slow;
        }
    }
}

/*** End of file. ***/

