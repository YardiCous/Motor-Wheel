/** @file navigation.c
 * @brief Multi-stage moves use the single-owner motion API.
 */
#include "navigation.h"

#include "../data_formats.h"
#include "../robot_config.h"
#include "../subsystem2/motion.h"
#include "obstacle.h"

#include <string.h>

/* Bypass stages. S is the chosen side, F the original heading, and L the
 * direction back toward the line (opposite of S). The robot stops for
 * NAVIGATION_SETTLE_MS before every turn, move and look. */
#define BYPASS_CHOOSE 0u
#define BYPASS_TURN_OUT 1u    /* F -> S */
#define BYPASS_SIDE_MOVE 2u   /* edge + half robot + margin, facing S */
#define BYPASS_SIDE_FACE 3u   /* S -> F */
#define BYPASS_SIDE_CHECK 4u  /* straight ahead clear? else step out more */
#define BYPASS_SIDE_AGAIN 5u  /* F -> S for one extra sideways step */
#define BYPASS_FWD_MOVE 6u    /* step forward, facing F */
#define BYPASS_FWD_FACE 7u    /* F -> L, to look across at the obstacle */
#define BYPASS_FWD_LOOK 8u    /* still beside? */
#define BYPASS_FWD_RETURN 9u  /* L -> F */
#define BYPASS_BACK_MOVE 10u  /* toward the line, facing L, until tape */
#define BYPASS_BACK_ALIGN 11u /* tape seen: bring the axle onto the line */
#define BYPASS_BACK_TURN 12u  /* L -> F, then follow (or sweep) */
#define NAVIGATION_SETTLE_MS 400u
#define NAVIGATION_STEP_MM                                                    \
    ((uint16_t)((ROBOT_TURN_MM_SECOND * ROBOT_BYPASS_STEP_MS) / 1000u))
#define NAVIGATION_BESIDE_EXTRA_CM 20u
/* A safe step shorter than this means a wall: stop with a fault. */
#define NAVIGATION_MIN_STEP_MM 30u

_Static_assert(ROBOT_BYPASS_STEP_MS <= 8000u,
               "a bypass step must finish inside the 10 s manoeuvre limit");
#define NAVIGATION_QUARTER_ARC_MM ROBOT_QUARTER_TURN_MM
#define NAVIGATION_LOOK_SETTLE_MS 400u
#define NAVIGATION_MAX_LEG_MM ((ROBOT_TURN_MM_SECOND * 8000u) / 1000u)
/* Junction turns: stage 1 drives the axle onto the junction, then turns. */
#define NAVIGATION_JUNCTION_TURN 1u

/* Body scanning is also selected automatically when servo edges are unseen. */
#ifndef ROBOT_BODY_SCAN
#define ROBOT_BODY_SCAN 0u
#endif /* ROBOT_BODY_SCAN */

static void issue(navigation_t * p_navigation, uint8_t mode, uint16_t distance,
                  uint16_t speed, bool b_reverse);
static void step_bypass(navigation_t * p_navigation, bool b_done,
                        bool b_line_valid, bool b_scan_ready, bool b_scan_left,
                        uint16_t range_cm, bool b_range_valid);
static void finish_body_scan(navigation_t * p_navigation);
static void step_look(navigation_t * p_navigation, bool b_done,
                      uint16_t range_cm, bool b_range_valid);
static void bypass_done(navigation_t * p_navigation, bool b_line_valid);
static void bypass_act(navigation_t * p_navigation, uint16_t range_cm,
                       bool b_range_valid);
static void bypass_turn(navigation_t * p_navigation, bool b_out);
static void bypass_move(navigation_t * p_navigation, uint16_t distance);
static void bypass_look(navigation_t * p_navigation, bool b_blocked,
                        uint32_t next_clear);
static void bypass_check(navigation_t * p_navigation, uint16_t seen);
static void bypass_forward(navigation_t * p_navigation, uint16_t range_cm,
                           bool b_range_valid);
static uint16_t safe_step(navigation_t * p_navigation, uint32_t wanted,
                          uint16_t range_cm, bool b_range_valid, bool b_whole);
static void     resume_follow(navigation_t * p_navigation);
static void step_follow(navigation_t * p_navigation, bool b_line_lost,
                        bool b_on_hump, uint16_t range_cm, bool b_range_valid,
                        bool b_barcode_zone);
static void start_search(navigation_t * p_navigation, bool b_line_valid);
static void step_search(navigation_t * p_navigation, bool b_done,
                        bool b_line_valid);

/** @brief No network state participates in boot readiness. */
void navigation_init(navigation_t * p_navigation)
{
    memset(p_navigation, 0, sizeof(*p_navigation));
    p_navigation->state = DATA_FORMATS_IDLE;
    junction_init(&p_navigation->junction);
}

/** @brief STOP is valid in every state and flushes motion at the OS boundary.
 */
bool navigation_supervise(navigation_t * p_navigation, uint8_t command,
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
void navigation_barcode(navigation_t * p_navigation, uint8_t barcode)
{
    if ((DATA_FORMATS_FOLLOW == p_navigation->state) && (barcode <= 3u))
    {
        p_navigation->state      = DATA_FORMATS_BARCODE;
        p_navigation->stage      = 0u;
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

/** @brief Letters are remembered in any state, until a junction uses them.
 */
bool navigation_letter(navigation_t * p_navigation, char letter)
{
    return junction_store(&p_navigation->junction, letter);
}

/** @brief The IR sensors are ROBOT_SENSOR_AHEAD_MM in front of the axle, so
 * a left or right turn first drives that far to turn on the junction itself.
 * A U-turn is twice the 90-degree arc, turned on the spot.
 */
uint8_t navigation_junction(navigation_t * p_navigation)
{
    uint8_t action = JUNCTION_IGNORED;

    if (DATA_FORMATS_FOLLOW == p_navigation->state)
    {
        action = junction_take(&p_navigation->junction);
    }

    if ((JUNCTION_LEFT == action) || (JUNCTION_RIGHT == action))
    {
        p_navigation->state      = DATA_FORMATS_BARCODE;
        p_navigation->stage      = NAVIGATION_JUNCTION_TURN;
        p_navigation->b_left     = (JUNCTION_LEFT == action);
        p_navigation->elapsed_ms = 0u;
        issue(p_navigation, MOTION_MOVE, ROBOT_SENSOR_AHEAD_MM,
              ROBOT_TURN_MM_SECOND, false);
    }
    else if (JUNCTION_UTURN == action)
    {
        p_navigation->state      = DATA_FORMATS_BARCODE;
        p_navigation->stage      = 0u;
        p_navigation->elapsed_ms = 0u;
        issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM * 2u,
              ROBOT_TURN_MM_SECOND, true);
    }
    else
    {
        /* FORWARD: line following already carries straight over. */
    }

    return action;
}

/** @brief Safety always wins over barcode completion or a pending bypass. */
void navigation_step(navigation_t * p_navigation, uint8_t motion_mode,
                     bool b_done, bool b_line_valid, bool b_line_lost,
                     bool b_on_hump, uint16_t range_cm, bool b_range_valid,
                     bool b_scan_ready, bool b_scan_left, bool b_barcode_zone,
                     bool b_safety_latched)
{
    if (b_safety_latched)
    {
        p_navigation->state            = DATA_FORMATS_FAULT;
        p_navigation->b_stop_requested = true;
        p_navigation->b_command_ready  = false;
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
            if (b_done && (MOTION_IDLE == motion_mode) &&
                (NAVIGATION_JUNCTION_TURN == p_navigation->stage))
            {
                /* On the junction: turn left (reverse) or right. */
                p_navigation->stage = 0u;
                issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM,
                      ROBOT_TURN_MM_SECOND, p_navigation->b_left);
            }
            else if (b_done && (MOTION_IDLE == motion_mode))
            {
                p_navigation->state = DATA_FORMATS_FOLLOW;
                issue(p_navigation, MOTION_FOLLOW, 0u,
                      ROBOT_CAUTIOUS_MM_SECOND, false);
            }
            break;

        case DATA_FORMATS_AVOID:
            step_bypass(p_navigation, b_done, b_line_valid, b_scan_ready,
                        b_scan_left, range_cm, b_range_valid);
            break;

        case DATA_FORMATS_SEARCH:
            step_search(p_navigation, b_done, b_line_valid);
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
static void issue(navigation_t * p_navigation, uint8_t mode, uint16_t distance,
                  uint16_t speed, bool b_reverse)
{
    p_navigation->b_command_ready = motion_command(
        &p_navigation->command, mode, distance, speed, b_reverse);
}

/** @brief Body-sweep plan: readings at +/-90 degrees are sideways room. The
 * edges were not seen separately, so the box is taken as centred.
 */
bool navigation_obstacle_plan(navigation_t * p_navigation, uint16_t width_mm,
                              uint16_t left_cm, uint16_t right_cm)
{
    uint16_t side       = obstacle_side_mm(width_mm / 2u);
    bool     b_left_ok  = (left_cm <= ROBOT_RANGE_MAX_CM) &&
                          (obstacle_room(left_cm * 10u, side));
    bool     b_right_ok = (right_cm <= ROBOT_RANGE_MAX_CM) &&
                          (obstacle_room(right_cm * 10u, side));
    bool     b_valid    = (DATA_FORMATS_AVOID == p_navigation->state) &&
                          (BYPASS_CHOOSE == p_navigation->stage) &&
                          (0u != width_mm) && (width_mm <= 4095u) &&
                          (b_left_ok || b_right_ok);

    if (b_valid)
    {
        p_navigation->width_mm = width_mm;
        p_navigation->side_mm  = side;
        p_navigation->left_cm  = left_cm;
        p_navigation->right_cm = right_cm;
        p_navigation->b_left =
            b_left_ok && ((!b_right_ok) || (left_cm >= right_cm));
    }

    return b_valid;
}

/** @brief Servo-scan plan: the scanner already chose the side and distance.
 */
bool navigation_obstacle_route(navigation_t * p_navigation, uint16_t width_mm,
                               uint16_t side_mm, bool b_left)
{
    bool b_valid = (DATA_FORMATS_AVOID == p_navigation->state) &&
                   (BYPASS_CHOOSE == p_navigation->stage) && (0u != side_mm) &&
                   (side_mm <= 4095u) && (width_mm <= 4095u);

    if (b_valid)
    {
        p_navigation->width_mm = width_mm;
        p_navigation->side_mm  = side_mm;
        p_navigation->b_left   = b_left;
    }

    return b_valid;
}

/** @brief Preserve the saved detection range while widening the view. */
bool navigation_body_scan(navigation_t * p_navigation)
{
    bool b_valid = (DATA_FORMATS_AVOID == p_navigation->state) &&
                   (BYPASS_CHOOSE == p_navigation->stage) &&
                   (!p_navigation->b_body_scan);

    if (b_valid)
    {
        p_navigation->b_body_scan  = true;
        p_navigation->look         = 0u;
        p_navigation->body_index   = 0u;
        p_navigation->elapsed_ms   = 0u;
        p_navigation->scan_failure = 0u;
    }

    return b_valid;
}

/** @brief Wait for a plan, then run one stage at a time. On the return leg
 * the first sight of tape stops the robot at once.
 */
static void step_bypass(navigation_t * p_navigation, bool b_done,
                        bool b_line_valid, bool b_scan_ready, bool b_scan_left,
                        uint16_t range_cm, bool b_range_valid)
{
    bool b_chosen = false;
    (void)b_scan_left;

    if ((BYPASS_CHOOSE == p_navigation->stage) &&
        ((p_navigation->b_body_scan) ||
         ((0u != ROBOT_BODY_SCAN) && (0u != p_navigation->width_mm))))
    {
        step_look(p_navigation, b_done, range_cm, b_range_valid);
        b_chosen = (BYPASS_CHOOSE != p_navigation->stage);
    }
    else if ((BYPASS_CHOOSE == p_navigation->stage) && b_scan_ready)
    {
        p_navigation->b_fault = (0u == p_navigation->side_mm);
        p_navigation->stage   = BYPASS_TURN_OUT;
        b_chosen              = !p_navigation->b_fault;
    }
    else if ((BYPASS_BACK_MOVE == p_navigation->stage) &&
             (p_navigation->b_waiting) && b_line_valid)
    {
        p_navigation->b_stop_requested = true;
        p_navigation->b_command_ready  = false;
        p_navigation->b_waiting        = false;
        p_navigation->b_line_seen      = true;
        p_navigation->elapsed_ms       = 0u;
        p_navigation->stage            = BYPASS_BACK_ALIGN;
    }
    else if ((p_navigation->b_waiting) && b_done)
    {
        p_navigation->b_waiting  = false;
        p_navigation->elapsed_ms = 0u;
        bypass_done(p_navigation, b_line_valid);
    }
    else
    {
        /* Waiting for the scan, a command to finish, or a look to settle. */
    }

    if (p_navigation->b_fault)
    {
        p_navigation->b_stop_requested = true;
        return;
    }

    if (b_chosen)
    {
        p_navigation->b_waiting         = false;
        p_navigation->elapsed_ms        = 0u;
        p_navigation->side_remaining_mm = p_navigation->side_mm;
        p_navigation->b_first_forward   = true;
        p_navigation->b_line_seen       = false;
        p_navigation->steps             = 0u;
        p_navigation->b_margin          = false;
    }

    bool b_running = (DATA_FORMATS_AVOID == p_navigation->state) &&
                     (BYPASS_CHOOSE != p_navigation->stage) &&
                     (!p_navigation->b_waiting);

    if (b_running)
    {
        bypass_act(p_navigation, range_cm, b_range_valid);
    }
}

/** @brief Choose the next stage once the current turn or move has finished.
 */
static void bypass_done(navigation_t * p_navigation, bool b_line_valid)
{
    uint32_t moved  = (p_navigation->command >> 2u) & 4095u;
    uint32_t remain = p_navigation->side_remaining_mm;
    remain          = (moved < remain) ? (remain - moved) : 0u;

    switch (p_navigation->stage)
    {
        case BYPASS_TURN_OUT:
        case BYPASS_SIDE_AGAIN:
            p_navigation->stage = BYPASS_SIDE_MOVE;
            break;

        case BYPASS_SIDE_MOVE:
            p_navigation->side_remaining_mm = remain;
            p_navigation->stage =
                (0u == remain) ? BYPASS_SIDE_FACE : BYPASS_SIDE_MOVE;
            break;

        case BYPASS_SIDE_FACE:
            p_navigation->stage = BYPASS_SIDE_CHECK;
            break;

        case BYPASS_FWD_MOVE:
            p_navigation->stage = BYPASS_FWD_FACE;
            break;

        case BYPASS_FWD_FACE:
            p_navigation->stage = BYPASS_FWD_LOOK;
            break;

        case BYPASS_FWD_RETURN:
            p_navigation->stage = BYPASS_FWD_MOVE;
            break;

        case BYPASS_BACK_MOVE:
            p_navigation->side_remaining_mm = remain;
            p_navigation->stage =
                (0u == remain) ? BYPASS_BACK_TURN : BYPASS_BACK_MOVE;
            break;

        case BYPASS_BACK_ALIGN:
            p_navigation->stage = BYPASS_BACK_TURN;
            break;

        default:
            /* BACK_TURN finished: on the line if tape stopped the return
             * leg, otherwise beside it, so sweep for it. */
            if ((p_navigation->b_line_seen) || (0u != ROBOT_NO_LINE))
            {
                resume_follow(p_navigation);
            }
            else
            {
                start_search(p_navigation, b_line_valid);
            }
            break;
    }
}

/** @brief Issue the current stage's turn or move, or evaluate a look. Every
 * action waits until the robot has been still for NAVIGATION_SETTLE_MS.
 */
static void bypass_act(navigation_t * p_navigation, uint16_t range_cm,
                       bool b_range_valid)
{
    /* An invalid reading is treated as the obstacle still being there. */
    uint16_t seen = b_range_valid ? range_cm : 0u;
    uint32_t beside =
        (p_navigation->side_mm / 10u) + NAVIGATION_BESIDE_EXTRA_CM;
    uint16_t step = 0u;

    if (p_navigation->elapsed_ms < NAVIGATION_SETTLE_MS)
    {
        return;
    }

    switch (p_navigation->stage)
    {
        case BYPASS_SIDE_FACE:
        case BYPASS_FWD_FACE:
            bypass_turn(p_navigation, false);
            break;

        case BYPASS_SIDE_MOVE:
        case BYPASS_BACK_MOVE:
            /* Sideways, a wall closer than the whole move is a fault; the
             * return leg may be cut short, since it stops on the tape. */
            step = safe_step(p_navigation, p_navigation->side_remaining_mm,
                             range_cm, b_range_valid,
                             BYPASS_SIDE_MOVE == p_navigation->stage);
            if (0u != step)
            {
                bypass_move(p_navigation, step);
            }
            break;

        case BYPASS_SIDE_CHECK:
            bypass_check(p_navigation, seen);
            break;

        case BYPASS_FWD_MOVE:
            bypass_forward(p_navigation, range_cm, b_range_valid);
            break;

        case BYPASS_FWD_LOOK:
            bypass_look(p_navigation, seen < beside, BYPASS_BACK_MOVE);
            if (BYPASS_BACK_MOVE == p_navigation->stage)
            {
                uint32_t back = (uint32_t)p_navigation->side_mm +
                                ROBOT_BYPASS_BACK_EXTRA_MM;
                p_navigation->side_remaining_mm =
                    (back > 4095u) ? 4095u : back;
            }
            else
            {
                p_navigation->stage = BYPASS_FWD_RETURN;
            }
            break;

        case BYPASS_BACK_ALIGN:
            bypass_move(p_navigation, ROBOT_SENSOR_AHEAD_MM);
            break;

        default:
            /* TURN_OUT, SIDE_AGAIN, FWD_RETURN and BACK_TURN turn toward S.
             */
            bypass_turn(p_navigation, true);
            break;
    }
}

/** @brief Facing F after the side move: the box must no longer be ahead.
 * If it is, the edge estimate was short, so step out further.
 */
static void bypass_check(navigation_t * p_navigation, uint16_t seen)
{
    uint32_t blocked_cm =
        (p_navigation->detected_mm / 10u) + NAVIGATION_BESIDE_EXTRA_CM;

    if (seen >= blocked_cm)
    {
        p_navigation->stage = BYPASS_FWD_MOVE;
    }
    else
    {
        uint32_t side =
            (uint32_t)p_navigation->side_mm + ROBOT_BYPASS_SIDE_EXTRA_MM;
        ++p_navigation->steps;
        p_navigation->b_fault =
            (p_navigation->steps > ROBOT_BYPASS_MAX_STEPS) || (side > 4095u);
        p_navigation->side_mm           = (side > 4095u) ? 4095u : side;
        p_navigation->side_remaining_mm = ROBOT_BYPASS_SIDE_EXTRA_MM;
        p_navigation->stage             = BYPASS_SIDE_AGAIN;
    }
}

/** @brief The first leg carries the robot from the detection point to
 * beside the box; later legs are fixed steps, or one margin move.
 */
static void bypass_forward(navigation_t * p_navigation, uint16_t range_cm,
                           bool b_range_valid)
{
    uint32_t wanted = NAVIGATION_STEP_MM;
    wanted = p_navigation->b_margin ? ROBOT_BYPASS_FWD_MARGIN_MM : wanted;
    wanted = p_navigation->b_first_forward
                 ? ((uint32_t)p_navigation->detected_mm + ROBOT_LENGTH_MM)
                 : wanted;
    uint16_t step =
        safe_step(p_navigation, wanted, range_cm, b_range_valid, false);

    if (0u != step)
    {
        p_navigation->b_first_forward = false;
        bypass_move(p_navigation, step);
    }
}

/** @brief Shorten a move to the free range ahead minus the clearance, and
 * to one leg the manoeuvre timer allows. Too little room is a fault, and so
 * is any shortfall when b_whole demands the full distance.
 */
static uint16_t safe_step(navigation_t * p_navigation, uint32_t wanted,
                          uint16_t range_cm, bool b_range_valid, bool b_whole)
{
    uint32_t room = b_range_valid ? ((uint32_t)range_cm * 10u) : 0u;
    room = (room > ROBOT_CLEARANCE_MM) ? (room - ROBOT_CLEARANCE_MM) : 0u;
    uint32_t step = (wanted < room) ? wanted : room;
    step = (step > NAVIGATION_MAX_LEG_MM) ? NAVIGATION_MAX_LEG_MM : step;

    bool b_short = b_whole && (room < wanted);

    if ((step < NAVIGATION_MIN_STEP_MM) || b_short)
    {
        p_navigation->b_fault          = true;
        p_navigation->b_stop_requested = true;
        step                           = 0u;
    }

    return (uint16_t)step;
}

/** @brief Hand back to line following at the cautious speed. */
static void resume_follow(navigation_t * p_navigation)
{
    p_navigation->state     = DATA_FORMATS_FOLLOW;
    p_navigation->stage     = 0u;
    p_navigation->b_waiting = false;
    p_navigation->b_slow    = true;
    issue(p_navigation, MOTION_FOLLOW, 0u, ROBOT_CAUTIOUS_MM_SECOND, false);
}

/** @brief Quarter turn toward S (out) or away from S (back). A left side
 * means "out" is a left turn, which the motion API encodes as reverse.
 */
static void bypass_turn(navigation_t * p_navigation, bool b_out)
{
    bool b_left = b_out ? p_navigation->b_left : !p_navigation->b_left;
    issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM,
          ROBOT_TURN_MM_SECOND, b_left);
    p_navigation->b_waiting = true;
}

/** @brief Straight move forward along the robot's current heading. */
static void bypass_move(navigation_t * p_navigation, uint16_t distance)
{
    issue(p_navigation, MOTION_MOVE, distance, ROBOT_TURN_MM_SECOND, false);
    p_navigation->b_waiting = true;
}

/** @brief A blocked look always retries, including after a margin move.
 * Two clear looks separated by a margin move permit return to the line.
 */
static void bypass_look(navigation_t * p_navigation, bool b_blocked,
                        uint32_t next_clear)
{
    if (b_blocked)
    {
        p_navigation->b_margin = false;
        ++p_navigation->steps;
        p_navigation->b_fault = (p_navigation->b_fault) ||
                                (p_navigation->steps > ROBOT_BYPASS_MAX_STEPS);
    }
    else if (p_navigation->b_margin)
    {
        p_navigation->b_margin = false;
        p_navigation->steps    = 0u;
        p_navigation->stage    = next_clear;
    }
    else
    {
        p_navigation->b_margin = true;
    }
}

/** @brief The robot slows before entering an obstacle or hump zone. */
static void step_follow(navigation_t * p_navigation, bool b_line_lost,
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
        p_navigation->b_command_ready  = false;
        p_navigation->b_scan_requested = true;
        p_navigation->detected_mm      = range_cm * 10u;
        p_navigation->width_mm         = 0u;
        p_navigation->side_mm          = 0u;
        p_navigation->b_body_scan      = false;
        p_navigation->body_index       = 0u;
        p_navigation->scan_failure     = 0u;
        p_navigation->stage            = 0u;
        p_navigation->elapsed_ms       = 0u;
        p_navigation->b_waiting        = false;
        p_navigation->look             = 0u;
        p_navigation->left_cm          = 0u;
        p_navigation->right_cm         = 0u;
    }
    else if (b_line_lost)
    {
        start_search(p_navigation, false);
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

/** @brief Sweep left a quarter turn first; stage 0 is that left turn. */
static void start_search(navigation_t * p_navigation, bool b_line_valid)
{
    p_navigation->state        = DATA_FORMATS_SEARCH;
    p_navigation->stage        = 0u;
    p_navigation->b_waiting    = true;
    p_navigation->elapsed_ms   = 0u;
    p_navigation->line_confirm = b_line_valid ? 1u : 0u;
    issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM,
          ROBOT_TURN_MM_SECOND, true);
}

/** @brief A finished left turn is followed by a right turn of twice the arc,
 * so the sensors pass back through centre and sweep the right side as well.
 */
static void step_search(navigation_t * p_navigation, bool b_done,
                        bool b_line_valid)
{
    p_navigation->line_confirm = b_line_valid
                                     ? ((p_navigation->line_confirm < 3u)
                                            ? (p_navigation->line_confirm + 1u)
                                            : 3u)
                                     : 0u;

    if (p_navigation->line_confirm >= 3u)
    {
        p_navigation->b_stop_requested = true;
        p_navigation->state            = DATA_FORMATS_FOLLOW;
        p_navigation->stage            = 0u;
        p_navigation->b_waiting        = false;
        issue(p_navigation, MOTION_FOLLOW, 0u, ROBOT_CAUTIOUS_MM_SECOND,
              false);
    }
    else if (p_navigation->elapsed_ms >= ROBOT_SEARCH_MS)
    {
        p_navigation->b_fault = true;
    }
    else if ((p_navigation->b_waiting) && b_done &&
             (0u == p_navigation->stage))
    {
        p_navigation->stage = 1u;
        issue(p_navigation, MOTION_TURN,
              (uint16_t)(NAVIGATION_QUARTER_ARC_MM * 2u), ROBOT_TURN_MM_SECOND,
              false);
    }
    else if ((p_navigation->b_waiting) && b_done)
    {
        /* Both sides have been swept without finding the line. */
        p_navigation->b_waiting = false;
        p_navigation->b_fault   = true;
    }
    else
    {
        /* The current turn is still running. */
    }
}

/** @brief Validate the measured body width before selecting the route. */
static void finish_body_scan(navigation_t * p_navigation)
{
    uint16_t width  = obstacle_body_width(p_navigation->body_ranges);
    bool     b_plan = navigation_obstacle_plan(
        p_navigation, width, p_navigation->body_ranges[0],
        p_navigation->body_ranges[OBSTACLE_BODY_POINTS - 1u]);

    if (!b_plan)
    {
        p_navigation->scan_failure =
            (0u == width) ? OBSTACLE_REASON_NO_EDGES : OBSTACLE_REASON_BLOCKED;
        for (uint8_t index = 0u; index < OBSTACLE_BODY_POINTS; ++index)
        {
            uint16_t range = p_navigation->body_ranges[index];
            if ((range < 2u) || (range > ROBOT_RANGE_MAX_CM))
            {
                p_navigation->scan_failure = OBSTACLE_REASON_INVALID;
            }
        }

        if ((p_navigation->body_ranges[6] * 10u) > OBSTACLE_FRONT_MAX_MM)
        {
            p_navigation->scan_failure = OBSTACLE_REASON_NO_FRONT;
        }
        p_navigation->b_fault = true;
    }
    else
    {
        p_navigation->stage      = BYPASS_TURN_OUT;
        p_navigation->b_waiting  = false;
        p_navigation->elapsed_ms = 0u;
    }
}

/** @brief Body sweep: left 90, sample every 15 degrees to right 90, home.
 * Each cumulative turn target uses the calibrated quarter arc, avoiding
 * rounding drift across the twelve increments. The servo remains centred.
 */
static void step_look(navigation_t * p_navigation, bool b_done,
                      uint16_t range_cm, bool b_range_valid)
{
    bool b_still = (p_navigation->elapsed_ms >= NAVIGATION_LOOK_SETTLE_MS);

    switch (p_navigation->look)
    {
        case 0u:
            issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM,
                  ROBOT_TURN_MM_SECOND, true);
            p_navigation->look       = 1u;
            p_navigation->elapsed_ms = 0u;
            break;

        case 1u:
        case 3u:
        case 5u:
            if (b_done)
            {
                ++p_navigation->look;
                p_navigation->elapsed_ms = 0u;
            }
            break;

        case 2u:
            if (b_still)
            {
                p_navigation->body_ranges[p_navigation->body_index] =
                    b_range_valid ? range_cm : 0u;
                if (p_navigation->body_index < (OBSTACLE_BODY_POINTS - 1u))
                {
                    uint32_t current = (p_navigation->body_index *
                                        NAVIGATION_QUARTER_ARC_MM) /
                                       6u;
                    uint32_t next    = ((p_navigation->body_index + 1u) *
                                        NAVIGATION_QUARTER_ARC_MM) /
                                       6u;
                    issue(p_navigation, MOTION_TURN,
                          (uint16_t)(next - current), ROBOT_TURN_MM_SECOND,
                          false);
                    p_navigation->look = 3u;
                }
                else
                {
                    issue(p_navigation, MOTION_TURN, NAVIGATION_QUARTER_ARC_MM,
                          ROBOT_TURN_MM_SECOND, true);
                    p_navigation->look = 5u;
                }
            }
            break;

        case 4u:
            ++p_navigation->body_index;
            p_navigation->look = 2u;
            break;

        default:
            if (b_still)
            {
                finish_body_scan(p_navigation);
            }
            break;
    }
}

/*** End of file. ***/

