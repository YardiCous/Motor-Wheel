/** @file motion.c
 * @brief Integer PID and encoder motion; assumptions are in robot_config.h.
 */
#include "motion.h"

#include "../data_formats.h"
#include "../robot_config.h"

#include <stddef.h>
#include <string.h>

#define MOTION_POSITION_MOD_UM 4096000
#define MOTION_MAX_EDGES 64u
#define MOTION_DUTY_MAX 1000
#define MOTION_DUTY_MIN 350
#define MOTION_DUTY_STEP 80
#define MOTION_INTEGRAL_MAX 16000

static int32_t limit(int32_t value, int32_t lower, int32_t upper);
static int16_t smooth_pwm(int16_t requested, int32_t target,
                          int16_t previous);
static int16_t control_pwm(int32_t target, int32_t measured,
                           uint32_t * p_integral, uint32_t * p_previous,
                           int16_t previous_pwm);
static int16_t wheel_pid(int32_t target, int32_t measured,
                         uint32_t * p_integral, uint32_t * p_previous);
static void update_distance(motion_t * p_motion, int32_t left, int32_t right);
static bool update_stall(uint32_t * p_elapsed, int32_t target,
                         int32_t measured);

/** @brief Initialization explicitly seeds every offset-binary field. */
void motion_init (motion_t * p_motion, uint32_t left, uint32_t right)
{
    memset(p_motion, 0, sizeof(*p_motion));
    p_motion->last_left      = left;
    p_motion->last_right     = right;
    p_motion->integral_left  = MOTION_INTEGRAL_MAX;
    p_motion->integral_right = MOTION_INTEGRAL_MAX;
    p_motion->prior_left     = 1000u;
    p_motion->prior_right    = 1000u;
    p_motion->pwm_left       = 0;
    p_motion->pwm_right      = 0;
    p_motion->remainder      = ROBOT_ENCODER_EDGES_TURN - 1u;
}

/** @brief Eight unused high bits must remain zero. */
bool motion_command (uint32_t * p_word, uint8_t mode, uint16_t distance,
                    uint16_t speed, bool b_reverse)
{
    bool b_valid = (NULL != p_word) && (mode <= MOTION_TURN) &&
                   (distance <= 4095u) && (speed <= 500u);

    if (b_valid)
    {
        uint32_t reverse = b_reverse ? 1u : 0u;
        *p_word          = mode | ((uint32_t)distance << 2u) |
                           ((uint32_t)speed << 14u) | (reverse << 23u);
    }

    return b_valid;
}

/** @brief Keep common manoeuvre construction out of callers. */
bool motion_forward (uint32_t * p_word, uint16_t distance, uint16_t speed)
{
    return motion_command(p_word, MOTION_MOVE, distance, speed, false);
}

/** @brief Reverse uses the same closed-loop distance accounting as forward. */
bool motion_backward (uint32_t * p_word, uint16_t distance, uint16_t speed)
{
    return motion_command(p_word, MOTION_MOVE, distance, speed, true);
}

/** @brief Positive left-wheel travel and negative right-wheel travel turns
 * the chassis right; invert both signs for a left turn. */
bool motion_turn_left (uint32_t * p_word, uint16_t distance, uint16_t speed)
{
    return motion_command(p_word, MOTION_TURN, distance, speed, true);
}

/** @brief A right turn drives the wheels in opposite directions. */
bool motion_turn_right (uint32_t * p_word, uint16_t distance, uint16_t speed)
{
    return motion_command(p_word, MOTION_TURN, distance, speed, false);
}

/** @brief Zero is the reserved stop command and has no stale payload. */
bool motion_stop (uint32_t * p_word)
{
    bool b_valid = (NULL != p_word);

    if (b_valid)
    {
        *p_word = 0u;
    }

    return b_valid;
}

/** @brief A stopped controller cannot replay a previously queued move. */
bool motion_accept (motion_t * p_motion, uint32_t command)
{
    uint32_t mode  = command & 3u;
    uint32_t speed = (command >> 14u) & 511u;
    bool     b_valid =
        (0u == (command >> 24u)) && (mode <= MOTION_TURN) && (speed <= 500u);
    bool b_available =
        (MOTION_IDLE == p_motion->mode) || (MOTION_FOLLOW == p_motion->mode);

    if (b_valid && (MOTION_IDLE == mode))
    {
        p_motion->mode           = MOTION_IDLE;
        p_motion->speed          = 0u;
        p_motion->integral_left  = MOTION_INTEGRAL_MAX;
        p_motion->integral_right = MOTION_INTEGRAL_MAX;
        p_motion->prior_left     = 1000u;
        p_motion->prior_right    = 1000u;
    }
    else if (b_valid && b_available && (speed > 0u))
    {
        p_motion->mode           = mode;
        p_motion->speed          = speed;
        p_motion->target_mm      = (command >> 2u) & 4095u;
        p_motion->b_reverse      = (command >> 23u) & 1u;
        p_motion->progress_um    = 0u;
        p_motion->elapsed_ms     = 0u;
        p_motion->b_done         = false;
        p_motion->b_stall        = false;
        p_motion->b_timeout      = false;
        p_motion->stall_left_ms  = 0u;
        p_motion->stall_right_ms = 0u;
        p_motion->integral_left  = MOTION_INTEGRAL_MAX;
        p_motion->integral_right = MOTION_INTEGRAL_MAX;
        p_motion->prior_left     = 1000u;
        p_motion->prior_right    = 1000u;
    }
    else
    {
        b_valid = false;
    }

    return b_valid;
}

/** @brief Unsigned subtraction is defined even when a counter wraps. */
int32_t motion_edge_delta (uint32_t newer, uint32_t older, bool * pb_valid)
{
    uint32_t forward  = newer - older;
    uint32_t backward = older - newer;
    int32_t  delta    = 0;
    *pb_valid         = true;

    if (forward <= MOTION_MAX_EDGES)
    {
        /* The edge-rate guard bounds the signed conversion to 64. */
        delta = (int32_t)forward;
    }
    else if (backward <= MOTION_MAX_EDGES)
    {
        /* The same bound makes signed negation representable. */
        delta = -(int32_t)backward;
    }
    else
    {
        *pb_valid = false;
    }

    return delta;
}

/** @brief Check safety before any PID work or motor output. */
void motion_step (motion_t * p_motion, uint32_t left, uint32_t right,
                 int16_t line_error, bool b_estop, int16_t * p_left_pwm,
                 int16_t * p_right_pwm)
{
    bool    b_left_valid  = false;
    bool    b_right_valid = false;
    int32_t left_edges =
        motion_edge_delta(left, p_motion->last_left, &b_left_valid);
    int32_t right_edges =
        motion_edge_delta(right, p_motion->last_right, &b_right_valid);
    *p_left_pwm          = 0;
    *p_right_pwm         = 0;
    p_motion->last_left  = left;
    p_motion->last_right = right;
    p_motion->b_encoder_fault =
        (p_motion->b_encoder_fault) || (((!b_left_valid) || (!b_right_valid)));
    update_distance(p_motion, left_edges, right_edges);

    if (b_estop || (p_motion->b_encoder_fault))
    {
        p_motion->mode = MOTION_ESTOP;
    }

    bool b_active =
        (p_motion->mode >= MOTION_FOLLOW) && (p_motion->mode <= MOTION_TURN);

    if (b_active)
    {
        /* Speed is 0..500 and encoder conversion stays below 1400 mm/s. */
        int32_t target_left    = (int32_t)p_motion->speed;
        int32_t target_right   = target_left;
        int32_t measured_left  = (left_edges * 20420) / 960;
        int32_t measured_right = (right_edges * 20420) / 960;

        if (p_motion->b_reverse)
        {
            target_left  = -target_left;
            target_right = -target_right;
        }

        if (MOTION_TURN == p_motion->mode)
        {
            target_right = -target_left;
        }

        if (MOTION_FOLLOW == p_motion->mode)
        {
            target_left  = limit(target_left + line_error, 0, 500);
            target_right = limit(target_right - line_error, 0, 500);
        }

        uint32_t elapsed_left  = p_motion->stall_left_ms;
        uint32_t elapsed_right = p_motion->stall_right_ms;
        bool     b_stall_left =
            update_stall(&elapsed_left, target_left, left_edges);
        bool b_stall_right =
            update_stall(&elapsed_right, target_right, right_edges);
        p_motion->stall_left_ms  = elapsed_left;
        p_motion->stall_right_ms = elapsed_right;
        p_motion->b_stall        = b_stall_left || b_stall_right;
        if (MOTION_FOLLOW != p_motion->mode)
        {
            uint32_t elapsed = p_motion->elapsed_ms + ROBOT_MOTION_PERIOD_MS;
            p_motion->elapsed_ms = (elapsed > 10000u) ? 10000u : elapsed;
        }
        p_motion->b_timeout     = (MOTION_FOLLOW != p_motion->mode) &&
                                  (p_motion->elapsed_ms >= ROBOT_MANOEUVRE_MS);
        uint32_t integral_left  = p_motion->integral_left;
        uint32_t integral_right = p_motion->integral_right;
        uint32_t prior_left     = p_motion->prior_left;
        uint32_t prior_right    = p_motion->prior_right;
        *p_left_pwm = control_pwm(target_left, measured_left, &integral_left,
                                  &prior_left, p_motion->pwm_left);
        *p_right_pwm = control_pwm(target_right, measured_right,
                                   &integral_right, &prior_right,
                                   p_motion->pwm_right);
        p_motion->pwm_left  = *p_left_pwm;
        p_motion->pwm_right = *p_right_pwm;
        p_motion->integral_left  = integral_left;
        p_motion->integral_right = integral_right;
        p_motion->prior_left     = prior_left;
        p_motion->prior_right    = prior_right;
    }

    bool b_completed =
        (MOTION_MOVE == p_motion->mode) || (MOTION_TURN == p_motion->mode);
    b_completed = b_completed &&
                  (p_motion->progress_um >= (p_motion->target_mm * 1000u));

    if (b_completed || (p_motion->b_stall) || (p_motion->b_timeout))
    {
        p_motion->b_done = b_completed;
        p_motion->mode   = p_motion->b_stall ? MOTION_STALL : MOTION_IDLE;
        *p_left_pwm      = 0;
        *p_right_pwm     = 0;
        p_motion->pwm_left  = 0;
        p_motion->pwm_right = 0;
    }
}

/** @brief The caller supplies actual speed and the common 5-ms clock. */
uint32_t motion_snapshot (const motion_t * p_motion, uint8_t tick,
                         uint16_t speed, bool b_reverse)
{
    uint32_t word = 0u;
    /* Position is maintained below 4096000 micrometres. */
    uint16_t distance = (uint16_t)(p_motion->position_um / 1000u);
    (void)data_formats_motion_pack(&word, distance, speed, b_reverse,
                                   p_motion->mode, tick);
    return word;
}

/** @brief Saturation prevents integral wind-up and unsafe duty values. */
static int32_t limit (int32_t value, int32_t lower, int32_t upper)
{
    int32_t result = value;

    if (value < lower)
    {
        result = lower;
    }

    else if (value > upper)
    {
        result = upper;
    }
    else
    {
        /* No saturation is required. */
    }

    return result;
}

/** @brief Keep a commanded wheel above static friction and ramp changes. */
static int16_t smooth_pwm (int16_t requested, int32_t target,
                           int16_t previous)
{
    int32_t desired = requested;
    int32_t magnitude;
    int32_t prior = previous;
    int32_t delta;

    if (0 == target)
    {
        return 0;
    }

    magnitude = (desired < 0) ? -desired : desired;
    if ((0 == prior) && (magnitude < MOTION_DUTY_MIN) &&
        ((0 == desired) || ((desired > 0) == (target > 0))))
    {
        magnitude = MOTION_DUTY_MIN;
        desired = (target < 0) ? -magnitude : magnitude;
    }
    desired = limit(desired, -MOTION_DUTY_MAX, MOTION_DUTY_MAX);
    if (((prior < 0) && (desired > 0)) ||
        ((prior > 0) && (desired < 0)))
    {
        if (prior > MOTION_DUTY_STEP)
        {
            return (int16_t)(prior - MOTION_DUTY_STEP);
        }
        if (prior < -MOTION_DUTY_STEP)
        {
            return (int16_t)(prior + MOTION_DUTY_STEP);
        }
        return 0;
    }
    delta = desired - prior;
    if (delta > MOTION_DUTY_STEP)
    {
        desired = prior + MOTION_DUTY_STEP;
    }
    else if (delta < -MOTION_DUTY_STEP)
    {
        desired = prior - MOTION_DUTY_STEP;
    }
    else
    {
        /* The requested duty is already within one control step. */
    }

    return (int16_t)desired;
}

/** @brief Apply PID control, static-friction compensation and ramp limiting. */
static int16_t control_pwm (int32_t target, int32_t measured,
                            uint32_t * p_integral, uint32_t * p_previous,
                            int16_t previous_pwm)
{
    int16_t requested = wheel_pid(target, measured, p_integral, p_previous);
    return smooth_pwm(requested, target, previous_pwm);
}

/** @brief Gains are provisional and require the NFR2 bench tuning test. */
static int16_t wheel_pid (int32_t target, int32_t measured,
                         uint32_t * p_integral, uint32_t * p_previous)
{
    int32_t error = limit(target - measured, -1000, 1000);
    /* Offset fields are bounded to 0..32000 and 0..2000. */
    int32_t integral = (int32_t)*p_integral - MOTION_INTEGRAL_MAX;
    int32_t prior    = (int32_t)*p_previous - 1000;
    integral =
        limit(integral + error, -MOTION_INTEGRAL_MAX, MOTION_INTEGRAL_MAX);
    int32_t output = target + error + (integral / 32) + ((error - prior) / 4);
    /* Both biased values are nonnegative and fit their packed widths. */
    *p_integral = (uint32_t)(integral + MOTION_INTEGRAL_MAX);
    *p_previous = (uint32_t)(error + 1000);
    /* The duty clamp fits signed 16-bit storage. */
    return (int16_t)limit(output, -MOTION_DUTY_MAX, MOTION_DUTY_MAX);
}

/** @brief Carry the division residue so sub-millimetre steps accumulate. */
static void update_distance (motion_t * p_motion, int32_t left, int32_t right)
{
    /* Position and residue fit within positive int32_t. */
    int32_t residual  = (int32_t)p_motion->remainder - 959;
    int32_t numerator = ((left + right) * 102100) + residual;
    int32_t travel_um = numerator / 960;
    int32_t position  = (int32_t)p_motion->position_um + travel_um;
    residual          = numerator % 960;

    if (position < 0)
    {
        position += MOTION_POSITION_MOD_UM;
    }
    else if (position >= MOTION_POSITION_MOD_UM)
    {
        position -= MOTION_POSITION_MOD_UM;
    }
    else
    {
        /* The position is already in the modular range. */
    }

    /* Modular position and biased residue are nonnegative and bounded. */
    p_motion->position_um = (uint32_t)position;
    p_motion->remainder   = (uint32_t)(residual + 959);
    int32_t left_abs      = (left < 0) ? -left : left;
    int32_t right_abs     = (right < 0) ? -right : right;
    /* A 64-edge guard bounds this positive increment below 14000. */
    uint32_t progress = (uint32_t)(((left_abs + right_abs) * 102100) / 960);
    /* Total wheel travel increases in either direction, including turns. */
    uint32_t total_numerator =
        ((uint32_t)(left_abs + right_abs) * 102100u) +
        p_motion->total_remainder;
    uint32_t total = p_motion->total_um + (total_numerator / 960u);
    p_motion->total_um = total % MOTION_POSITION_MOD_UM;
    p_motion->total_remainder = total_numerator % 960u;
    progress += p_motion->progress_um;
    p_motion->progress_um = (progress > 4095999u) ? 4095999u : progress;
}

/** @brief A stationary commanded wheel faults after the 200-ms window. */
static bool update_stall (uint32_t * p_elapsed, int32_t target,
                         int32_t measured)
{
    bool b_driven = (target > 80) || (target < -80);

    if (b_driven && (0 == measured))
    {
        *p_elapsed += ROBOT_MOTION_PERIOD_MS;
    }
    else
    {
        *p_elapsed = 0u;
    }

    *p_elapsed = (*p_elapsed > ROBOT_STALL_MS) ? ROBOT_STALL_MS : *p_elapsed;
    return (*p_elapsed >= ROBOT_STALL_MS);
}

/*** End of file. ***/

