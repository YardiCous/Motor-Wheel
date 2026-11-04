/** @file terrain.c
 * @brief Gyro-free hump detection and height estimation (Buddy 4).
 *
 * Sensor fusion (W9 lecture terms): heterogeneous, complementary fusion of
 * the accelerometer (gravity direction = pitch) with the wheel encoders
 * (distance travelled and longitudinal acceleration).
 *
 * The accelerometer's forward axis reads g*sin(pitch) + a, where a is the
 * car's own forward acceleration. Speeding up at 1 m/s^2 would look like a
 * 6-degree climb, so a is estimated from encoder speed and subtracted.
 * Both signals pass through the same low-pass filter first, so neither one
 * lags the other when they are subtracted.
 *
 * See terrain_config.h for every tunable value and its status.
 */
#include "terrain.h"

#include "../data_formats.h"
#include "../robot_config.h"
#include "terrain_config.h"

/* Units: milli-g per g, and milli per unit. */
#define TERRAIN_MG_PER_G 1000
/* 1 g = 9807 mm/s^2. Speed is sampled every ROBOT_IMU_PERIOD_MS, so one
 * mm/s change per 10 ms is 100 mm/s^2 = 10.2 mg. */
#define TERRAIN_GRAVITY_MM_S2 9807
#define TERRAIN_MS_PER_S 1000
/* Height: ds (mm) * sin x 1000 gives micrometres; 10 um = 0.01 mm. */
#define TERRAIN_UM_PER_CMM 10
#define TERRAIN_CMM_PER_MM 100
#define TERRAIN_PEAK_MAX_MM 255
/* Pitch field: 0.5 deg = 50 cdeg per step, zero at 64, 7 bits. */
#define TERRAIN_PITCH_STEP_CDEG 50
#define TERRAIN_PITCH_ZERO 64
#define TERRAIN_PITCH_FIELD_MAX 127
/* Hump event bit layout (TEAM.md): finished 0, peak 1..8, valid 9. */
#define TERRAIN_EVENT_PEAK_SHIFT 1u
#define TERRAIN_EVENT_VALID_SHIFT 9u
#define TERRAIN_EVENT_MASK 0x3ffu
/* Saturation limits for the bit-field counters. */
#define TERRAIN_PHASE_MAX_MS 10000u
#define TERRAIN_STALE_MAX 15u
#define TERRAIN_CONFIRM_MAX_MM 63u
#define TERRAIN_IMPACT_RUN_MAX 3u
/* atan approximation: Rajan et al. (2006), atan(z) ~ (pi/4)z
 * - z(|z|-1)(0.2447 + 0.0663|z|) radians for 0 <= z <= 1, here in 0.01 deg
 * with z in Q12 (4096 = 1.0). */
#define TERRAIN_Q12_ONE 4096
#define TERRAIN_ATAN_45_CDEG 4500
#define TERRAIN_ATAN_A_CDEG 1402
#define TERRAIN_ATAN_B_CDEG 380
#define TERRAIN_90_CDEG 9000
#define TERRAIN_180_CDEG 18000
#define TERRAIN_ISQRT_TOP_BIT (1u << 30u)
/* Yaw rate: (vR - vL) / track in rad/s; 57296 / 1000 deg per rad. */
#define TERRAIN_DEG_PER_RAD_X1000 57296
#define TERRAIN_MILLI 1000
#define TERRAIN_SIN_ONE_SQUARED 1000000
/* Keeps the wheel-difference filter inside its int16_t state. */
#define TERRAIN_TURN_DIFF_LIMIT 2000
/* Raised wheel: track (mm) x sin x 1000 gives um; 10 um = 0.01 mm. */
#define TERRAIN_SIDE_UM_PER_CMM 10
#define TERRAIN_PERCENT 100

static int32_t magnitude(int32_t value);
static int32_t clamp(int32_t value, int32_t lower, int32_t upper);
static void    clear_events(terrain_t * p_terrain);
static void    imu_failure(terrain_t * p_terrain);
static void    map_axes(const terrain_input_t * p_input, int32_t * p_forward,
                        int32_t * p_left, int32_t * p_up);
static int32_t distance_step(terrain_t *             p_terrain,
                             const terrain_input_t * p_input);
static void    update_filters(terrain_t *             p_terrain,
                              const terrain_input_t * p_input, int32_t forward,
                              int32_t left);
static void update_lean(terrain_t * p_terrain, const terrain_input_t * p_input,
                        int32_t left);
static int32_t turning_mg(const terrain_t * p_terrain);
static int32_t filtered_turn_dps(const terrain_t * p_terrain);
static void calibrate(terrain_t * p_terrain, const terrain_input_t * p_input,
                      int32_t forward, int32_t left, int32_t up);
static bool check_impact(terrain_t * p_terrain, int32_t forward, int32_t left);
static void step_hump(terrain_t * p_terrain, int32_t ds_mm);
static void step_pitch(terrain_t * p_terrain, int32_t ds_mm);
static void step_level(terrain_t * p_terrain, int32_t ds_mm);
static void arm_side(terrain_t * p_terrain, int32_t ds_mm);
static int32_t side_height(int32_t lean);
static void    step_side(terrain_t * p_terrain);
static void    add_height(terrain_t * p_terrain, int32_t ds_mm);
static void    finish_hump(terrain_t * p_terrain, bool b_valid);
static void    update_baseline(terrain_t *             p_terrain,
                               const terrain_input_t * p_input);
static void    classify(terrain_t * p_terrain, const terrain_input_t * p_input,
                        int32_t forward);
static void    advance_timers(terrain_t * p_terrain);

/* Compile-time checks (C99 has no _Static_assert under -pedantic, so a
 * negative array size is used instead, as in data_formats.h). */
typedef uint8_t terrain_confirm_fits_t
    [(TERRAIN_CONFIRM_MM <= TERRAIN_CONFIRM_MAX_MM) ? 1 : -1];
typedef uint8_t terrain_side_confirm_fits_t
    [(TERRAIN_SIDE_CONFIRM_MM <= TERRAIN_CONFIRM_MAX_MM) ? 1 : -1];
typedef uint8_t terrain_cal_fits_t[(TERRAIN_CAL_SAMPLES <= 63u) ? 1 : -1];
typedef uint8_t
    terrain_gate_fits_t[(TERRAIN_LANDING_GATE_MS <= 511u) ? 1 : -1];
typedef uint8_t terrain_filter_fits_t
    [((TERRAIN_ACCEL_LIMIT_MG * TERRAIN_FAST_DIVISOR) <= INT16_MAX) ? 1 : -1];

/** @brief Calibration restarts from scratch; the run peak is cleared too. */
void terrain_init(terrain_t * p_terrain)
{
    *p_terrain                = (terrain_t){0};
    p_terrain->state          = TERRAIN_CALIBRATING;
    p_terrain->motion         = TERRAIN_MOTION_UNKNOWN;
    p_terrain->pitch_field    = (uint32_t)TERRAIN_PITCH_ZERO;
    p_terrain->b_request_init = true;
}

/** @brief One step: fault handling, filters, then the statechart. */
void terrain_step(terrain_t * p_terrain, const terrain_input_t * p_input)
{
    clear_events(p_terrain);

    if (!p_input->b_imu_ok)
    {
        /* No pitch this step. The odometer keeps counting, so the next
         * good step's distance covers this gap. */
        imu_failure(p_terrain);
        advance_timers(p_terrain);
        /* Passing the filtered value means "no new vibration evidence". */
        classify(p_terrain, p_input,
                 p_terrain->forward_x8 / TERRAIN_FAST_DIVISOR);
        return;
    }

    p_terrain->failures = 0u;

    if (TERRAIN_SENSOR_FAULT == p_terrain->state)
    {
        /* Recovered. A short I2C outage does not move the chip's offset,
         * so an existing calibration is kept and the car need not stop. */
        p_terrain->state =
            (p_terrain->b_calibrated) ? TERRAIN_LEVEL : TERRAIN_CALIBRATING;
    }

    int32_t forward = 0;
    int32_t left    = 0;
    int32_t up      = 0;
    map_axes(p_input, &forward, &left, &up);
    int32_t ds_mm = distance_step(p_terrain, p_input);
    update_filters(p_terrain, p_input, forward, left);

    if (TERRAIN_CALIBRATING == p_terrain->state)
    {
        calibrate(p_terrain, p_input, forward, left, up);
    }
    else if (check_impact(p_terrain, forward, left))
    {
        /* IMPACT is held so Safety's periodic check also sees it. */
        if (p_terrain->phase_ms >= TERRAIN_IMPACT_HOLD_MS)
        {
            p_terrain->state    = TERRAIN_LEVEL;
            p_terrain->phase_ms = 0u;
        }
    }
    else
    {
        step_hump(p_terrain, ds_mm);
        update_baseline(p_terrain, p_input);
    }

    /* The telemetry pitch is the angle whose sine is sin_milli. */
    int32_t sine   = p_terrain->sin_milli;
    int32_t cosine = (int32_t)terrain_isqrt(
        (uint32_t)(TERRAIN_SIN_ONE_SQUARED - (sine * sine)));
    p_terrain->pitch_field =
        terrain_pitch_field(terrain_atan2_cdeg(sine, cosine));
    advance_timers(p_terrain);
    classify(p_terrain, p_input, forward);
}

/** @brief Safety must not wait forever for an IMU that is absent. */
bool terrain_ready(const terrain_t * p_terrain)
{
    return (p_terrain->b_calibrated) ||
           (TERRAIN_SENSOR_FAULT == p_terrain->state);
}

/** @brief Round half away from zero, then saturate; never wrap. */
uint32_t terrain_pitch_field(int32_t pitch_cdeg)
{
    int32_t half  = TERRAIN_PITCH_STEP_CDEG / 2;
    int32_t steps = (pitch_cdeg >= 0)
                        ? ((pitch_cdeg + half) / TERRAIN_PITCH_STEP_CDEG)
                        : ((pitch_cdeg - half) / TERRAIN_PITCH_STEP_CDEG);
    int32_t field =
        clamp(steps + TERRAIN_PITCH_ZERO, 0, TERRAIN_PITCH_FIELD_MAX);
    return (uint32_t)field;
}

/** @brief Inverse of terrain_pitch_field for telemetry decoding and tests. */
int32_t terrain_pitch_cdeg(uint32_t field)
{
    int32_t steps = (int32_t)(field & (uint32_t)TERRAIN_PITCH_FIELD_MAX);
    return (steps - TERRAIN_PITCH_ZERO) * TERRAIN_PITCH_STEP_CDEG;
}

/** @brief Negative heights report 0; anything above 255 mm reports 255. */
uint32_t terrain_peak_mm(int32_t height_cmm)
{
    int32_t rounded =
        (height_cmm + (TERRAIN_CMM_PER_MM / 2)) / TERRAIN_CMM_PER_MM;
    return (uint32_t)clamp(rounded, 0, TERRAIN_PEAK_MAX_MM);
}

/** @brief Ten information bits in a 16-bit word; bits 10..15 are zero. */
uint16_t terrain_event_pack(bool b_finished, uint32_t peak_mm, bool b_valid)
{
    uint32_t peak  = (peak_mm > (uint32_t)TERRAIN_PEAK_MAX_MM)
                         ? (uint32_t)TERRAIN_PEAK_MAX_MM
                         : peak_mm;
    uint32_t event = (b_finished ? 1u : 0u) |
                     (peak << TERRAIN_EVENT_PEAK_SHIFT) |
                     ((b_valid ? 1u : 0u) << TERRAIN_EVENT_VALID_SHIFT);
    return (uint16_t)event;
}

/** @brief Navigation-side decoder for the same layout. */
bool terrain_event_unpack(uint16_t event, bool * pb_finished,
                          uint32_t * p_peak_mm, bool * pb_valid)
{
    uint32_t word = event;
    *pb_finished  = (0u != (word & 1u));
    *p_peak_mm =
        (word >> TERRAIN_EVENT_PEAK_SHIFT) & (uint32_t)TERRAIN_PEAK_MAX_MM;
    *pb_valid = (0u != ((word >> TERRAIN_EVENT_VALID_SHIFT) & 1u));
    return (0u == (word & ~TERRAIN_EVENT_MASK));
}

/** @brief Reduce to the first octant, approximate, then unfold. */
int32_t terrain_atan2_cdeg(int32_t y, int32_t x)
{
    uint32_t abs_y = (uint32_t)magnitude(y);
    uint32_t abs_x = (uint32_t)magnitude(x);
    int32_t  angle = 0;

    if ((0u != abs_y) || (0u != abs_x))
    {
        bool     b_steep = (abs_y > abs_x);
        uint32_t small   = b_steep ? abs_x : abs_y;
        uint32_t large   = b_steep ? abs_y : abs_x;
        /* z = small / large in Q12, 0..4096. small < 2^19 fits the product. */
        int32_t z = (int32_t)((small * (uint32_t)TERRAIN_Q12_ONE) / large);
        /* z(1 - z) in Q12 is at most 1024. */
        int32_t bend = (z * (TERRAIN_Q12_ONE - z)) / TERRAIN_Q12_ONE;
        int32_t gain = TERRAIN_ATAN_A_CDEG +
                       ((TERRAIN_ATAN_B_CDEG * z) / TERRAIN_Q12_ONE);
        angle = ((TERRAIN_ATAN_45_CDEG * z) / TERRAIN_Q12_ONE) +
                ((bend * gain) / TERRAIN_Q12_ONE);
        angle = b_steep ? (TERRAIN_90_CDEG - angle) : angle;
        angle = (x < 0) ? (TERRAIN_180_CDEG - angle) : angle;
        angle = (y < 0) ? -angle : angle;
    }

    return angle;
}

/** @brief Bit-by-bit method: 16 iterations, no division. */
uint32_t terrain_isqrt(uint32_t value)
{
    uint32_t remainder = value;
    uint32_t root      = 0u;
    uint32_t bit       = TERRAIN_ISQRT_TOP_BIT;

    while (bit > remainder)
    {
        bit >>= 2u;
    }

    while (0u != bit)
    {
        if (remainder >= (root + bit))
        {
            remainder -= root + bit;
            root = (root >> 1u) + bit;
        }
        else
        {
            root >>= 1u;
        }

        bit >>= 2u;
    }

    return root;
}

/** @brief Absolute value; inputs are bounded sensor-scale integers. */
static int32_t magnitude(int32_t value)
{
    return (value < 0) ? -value : value;
}

/** @brief Saturate value to lower..upper. */
static int32_t clamp(int32_t value, int32_t lower, int32_t upper)
{
    int32_t result = (value < lower) ? lower : value;
    return (result > upper) ? upper : result;
}

/** @brief Events last exactly one step. */
static void clear_events(terrain_t * p_terrain)
{
    p_terrain->b_started        = false;
    p_terrain->b_finished       = false;
    p_terrain->b_impact         = false;
    p_terrain->b_motion_changed = false;
    p_terrain->b_request_init   = false;
}

/** @brief Statechart 6.5: 3 consecutive failures enter SENSOR_FAULT. */
static void imu_failure(terrain_t * p_terrain)
{
    if (p_terrain->failures < TERRAIN_FAIL_LIMIT)
    {
        ++p_terrain->failures;
    }

    if (p_terrain->failures < TERRAIN_FAIL_LIMIT)
    {
        /* One or two missed samples: wait. The hump carries on. */
    }
    else if (TERRAIN_SENSOR_FAULT != p_terrain->state)
    {
        if (p_terrain->b_on_hump)
        {
            finish_hump(p_terrain, false);
        }

        p_terrain->b_arming       = false;
        p_terrain->state          = TERRAIN_SENSOR_FAULT;
        p_terrain->phase_ms       = 0u;
        p_terrain->b_request_init = true;
    }
    else if (p_terrain->phase_ms >= TERRAIN_REINIT_PERIOD_MS)
    {
        /* Backoff: re-initialise at most every 100 ms, so a stuck bus
         * cannot use up the task's 10-ms budget every cycle. */
        p_terrain->phase_ms       = 0u;
        p_terrain->b_request_init = true;
    }
    else
    {
        /* Still waiting for the next retry. */
    }
}

/** @brief Apply the mounting table from terrain_config.h. */
static void map_axes(const terrain_input_t * p_input, int32_t * p_forward,
                     int32_t * p_left, int32_t * p_up)
{
    int32_t limit = TERRAIN_ACCEL_LIMIT_MG;
    *p_forward    = TERRAIN_SIGN_FORWARD *
                 clamp(p_input->accel_mg[TERRAIN_AXIS_FORWARD], -limit, limit);
    *p_left = TERRAIN_SIGN_LEFT *
              clamp(p_input->accel_mg[TERRAIN_AXIS_LEFT], -limit, limit);
    *p_up = TERRAIN_SIGN_UP *
            clamp(p_input->accel_mg[TERRAIN_AXIS_UP], -limit, limit);
}

/** @brief Wrap-safe odometer step; negative when reversing.
 * Stale Motion data is skipped and counted. last_distance is not moved, so
 * the next fresh step includes the skipped travel and no distance is lost.
 */
static int32_t distance_step(terrain_t *             p_terrain,
                             const terrain_input_t * p_input)
{
    int32_t ds_mm = 0;

    if (!p_input->b_motion_fresh)
    {
        if (p_terrain->stale_run < TERRAIN_STALE_MAX)
        {
            ++p_terrain->stale_run;
        }

        if ((p_terrain->b_on_hump) &&
            (p_terrain->stale_run > TERRAIN_STALE_LIMIT))
        {
            p_terrain->b_hump_valid = false;
        }
    }
    else
    {
        p_terrain->stale_run = 0u;

        if (p_terrain->b_have_last)
        {
            ds_mm = data_formats_distance_delta(
                (uint16_t)p_input->distance_mm,
                (uint16_t)p_terrain->last_distance);
        }

        p_terrain->last_distance = p_input->distance_mm;
        p_terrain->b_have_last   = true;
    }

    return ds_mm;
}

/** @brief EMA filters, encoder acceleration, and pitch vs the baseline.
 *
 * Each filter state is stored x8 (fixed point) so the EMA step
 * state += x - state / 8 keeps its fractional part. Integer division
 * truncates toward zero, which leaves at most 1 unit of bias.
 * The derivative of the filtered speed equals the filtered acceleration,
 * so acceleration comes from the change in the speed filter's state.
 */
static void update_filters(terrain_t *             p_terrain,
                           const terrain_input_t * p_input, int32_t forward,
                           int32_t left)
{
    int32_t forward_x8 = p_terrain->forward_x8;
    forward_x8 += forward - (forward_x8 / TERRAIN_FAST_DIVISOR);
    p_terrain->forward_x8 = (int16_t)forward_x8;

    if (p_input->b_motion_fresh)
    {
        int32_t speed_x8 = p_terrain->speed_x8;
        int32_t change =
            p_input->speed_mm_s - (speed_x8 / TERRAIN_FAST_DIVISOR);
        p_terrain->speed_x8 = (int16_t)(speed_x8 + change);
        /* change / 8 is the mm/s gained in one 10-ms period, so the
         * acceleration is change / 8 * 100 mm/s^2, and in milli-g:
         * change * 100 * 1000 / (8 * 9807). (The old code's factor was
         * 10x too small, so it removed only a tenth of the effect.) */
        int32_t steps_per_s = TERRAIN_MS_PER_S / (int32_t)ROBOT_IMU_PERIOD_MS;
        int32_t accel_mg    = (change * steps_per_s * TERRAIN_MG_PER_G) /
                           (TERRAIN_FAST_DIVISOR * TERRAIN_GRAVITY_MM_S2);
        p_terrain->accel_kin_mg = (int16_t)accel_mg;
    }

    /* g*sin(pitch) = forward reading - own acceleration - flat baseline. */
    int32_t gravity = (forward_x8 / TERRAIN_FAST_DIVISOR) -
                      p_terrain->accel_kin_mg -
                      (p_terrain->baseline_x256 / TERRAIN_BASELINE_DIVISOR);
    p_terrain->sin_milli =
        (int16_t)clamp(gravity, -TERRAIN_MG_PER_G, TERRAIN_MG_PER_G);
    update_lean(p_terrain, p_input, left);
}

/** @brief Lean = left reading - turning force - flat baseline, filtered twice.
 * Positive means the left side is up. The wheel difference uses the same
 * filter as the left axis, so the two line up when subtracted.
 */
static void update_lean(terrain_t * p_terrain, const terrain_input_t * p_input,
                        int32_t left)
{
    int32_t left_x6 = p_terrain->left_x6;
    left_x6 += left - (left_x6 / TERRAIN_SIDE_FAST_DIVISOR);
    p_terrain->left_x6 = (int16_t)left_x6;

    if (p_input->b_motion_fresh)
    {
        int32_t turn_x6 = p_terrain->turn_x6;
        int32_t difference =
            clamp(p_input->right_mm_s - p_input->left_mm_s,
                  -TERRAIN_TURN_DIFF_LIMIT, TERRAIN_TURN_DIFF_LIMIT);
        turn_x6 += difference - (turn_x6 / TERRAIN_SIDE_FAST_DIVISOR);
        p_terrain->turn_x6 = (int16_t)turn_x6;
    }

    int32_t lean = (left_x6 / TERRAIN_SIDE_FAST_DIVISOR) -
                   turning_mg(p_terrain) -
                   (p_terrain->baseline_left_x256 / TERRAIN_BASELINE_DIVISOR);
    lean         = clamp(lean, -TERRAIN_MG_PER_G, TERRAIN_MG_PER_G);
    int32_t roll = p_terrain->roll_x4;
    roll += lean - (roll / TERRAIN_SIDE_FILTER_DIVISOR);
    p_terrain->roll_x4    = (int16_t)roll;
    p_terrain->roll_milli = (int16_t)(roll / TERRAIN_SIDE_FILTER_DIVISOR);
}

/** @brief Sideways force of turning: v x (vR - vL) / track, in milli-g.
 * Dividing by the track first keeps the product inside int32_t.
 */
static int32_t turning_mg(const terrain_t * p_terrain)
{
    int32_t speed       = p_terrain->speed_x8 / TERRAIN_FAST_DIVISOR;
    int32_t difference  = p_terrain->turn_x6 / TERRAIN_SIDE_FAST_DIVISOR;
    int32_t accel_mm_s2 = (speed * difference) / (int32_t)TERRAIN_TRACK_MM;
    return (accel_mm_s2 * TERRAIN_MG_PER_G) / TERRAIN_GRAVITY_MM_S2;
}

/** @brief Yaw rate from the filtered wheel difference, in deg/s. */
static int32_t filtered_turn_dps(const terrain_t * p_terrain)
{
    int32_t difference = p_terrain->turn_x6 / TERRAIN_SIDE_FAST_DIVISOR;
    return (difference * TERRAIN_DEG_PER_RAD_X1000) /
           ((int32_t)TERRAIN_TRACK_MM * TERRAIN_MILLI);
}

/** @brief Average 32 still samples. Any movement restarts the count. */
static void calibrate(terrain_t * p_terrain, const terrain_input_t * p_input,
                      int32_t forward, int32_t left, int32_t up)
{
    int32_t smooth = p_terrain->forward_x8 / TERRAIN_FAST_DIVISOR;
    bool    b_still =
        (p_input->b_motion_fresh) &&
        (magnitude(p_input->speed_mm_s) <= TERRAIN_STILL_SPEED_MM_S) &&
        (magnitude(forward - smooth) <= TERRAIN_STILL_JITTER_MG);
    /* If "up" is not near 1 g, the mounting table is wrong: never
     * calibrate on it. IMU_TEST prints the raw axes to fix the table. */
    bool b_upright =
        (up >= TERRAIN_GRAVITY_LOW_MG) && (up <= TERRAIN_GRAVITY_HIGH_MG);

    if (b_still && b_upright)
    {
        p_terrain->cal_forward_sum += forward;
        p_terrain->cal_left_sum += left;
        p_terrain->cal_up_sum += up;
        ++p_terrain->cal_samples;
    }
    else
    {
        p_terrain->cal_samples     = 0u;
        p_terrain->cal_forward_sum = 0;
        p_terrain->cal_left_sum    = 0;
        p_terrain->cal_up_sum      = 0;
    }

    if (p_terrain->cal_samples >= TERRAIN_CAL_SAMPLES)
    {
        /* The flat-ground forward reading holds both the chip's zero-g
         * offset (up to +/-60 mg) and any mounting tilt. */
        int32_t baseline =
            p_terrain->cal_forward_sum / (int32_t)TERRAIN_CAL_SAMPLES;
        p_terrain->baseline_x256 = baseline * TERRAIN_BASELINE_DIVISOR;
        p_terrain->baseline_left_x256 =
            (p_terrain->cal_left_sum / (int32_t)TERRAIN_CAL_SAMPLES) *
            TERRAIN_BASELINE_DIVISOR;
        p_terrain->roll_x4      = 0;
        p_terrain->roll_milli   = 0;
        p_terrain->b_calibrated = true;
        p_terrain->state        = TERRAIN_LEVEL;
        p_terrain->phase_ms     = 0u;
    }
}

/** @brief Returns true while in IMPACT. Entering it abandons any hump. */
static bool check_impact(terrain_t * p_terrain, int32_t forward, int32_t left)
{
    /* A wheel dropping off a one-wheel ramp is a landing too. */
    bool b_landing = (p_terrain->gate_ms > 0u) || (p_terrain->b_side) ||
                     (TERRAIN_DESCENDING == p_terrain->state);
    int32_t limit = b_landing ? TERRAIN_LANDING_IMPACT_MG : TERRAIN_IMPACT_MG;
    /* Compare squares: no square root needed. 2 * 4000^2 fits int32_t. */
    int32_t horizontal = (forward * forward) + (left * left);

    if (horizontal > (limit * limit))
    {
        if (p_terrain->impact_run < TERRAIN_IMPACT_RUN_MAX)
        {
            ++p_terrain->impact_run;
        }
    }
    else
    {
        p_terrain->impact_run = 0u;
    }

    if ((p_terrain->impact_run >= TERRAIN_IMPACT_SAMPLES) &&
        (TERRAIN_IMPACT != p_terrain->state))
    {
        if (p_terrain->b_on_hump)
        {
            finish_hump(p_terrain, false);
        }

        p_terrain->b_arming    = false;
        p_terrain->b_wait_flat = true;
        p_terrain->state       = TERRAIN_IMPACT;
        p_terrain->phase_ms    = 0u;
        p_terrain->b_impact    = true;
    }

    return (TERRAIN_IMPACT == p_terrain->state);
}

/** @brief Run whichever hump logic applies, with one 10-s limit for both. */
static void step_hump(terrain_t * p_terrain, int32_t ds_mm)
{
    if (p_terrain->b_side)
    {
        step_side(p_terrain);
    }
    else
    {
        step_pitch(p_terrain, ds_mm);
    }

    if ((p_terrain->b_on_hump) &&
        (p_terrain->hump_ms >= TERRAIN_HUMP_TIMEOUT_MS))
    {
        /* Never came back down within 10 s: report it, marked invalid. */
        finish_hump(p_terrain, false);
    }
}

/** @brief Statechart 6.5 hump phases. sin_milli is pitch vs flat. */
static void step_pitch(terrain_t * p_terrain, int32_t ds_mm)
{
    int32_t sine = p_terrain->sin_milli;

    switch (p_terrain->state)
    {
        case TERRAIN_LEVEL:
            step_level(p_terrain, ds_mm);

            if (TERRAIN_LEVEL == p_terrain->state)
            {
                arm_side(p_terrain, ds_mm);
            }
            break;

        case TERRAIN_CLIMBING:
            add_height(p_terrain, ds_mm);

            if (sine < -TERRAIN_CLIMB_ENTER_MILLI)
            {
                /* Sharp crest: straight from climbing to descending. */
                p_terrain->state    = TERRAIN_DESCENDING;
                p_terrain->phase_ms = 0u;
            }
            else if (sine < TERRAIN_CLIMB_EXIT_MILLI)
            {
                p_terrain->state    = TERRAIN_CRESTING;
                p_terrain->phase_ms = 0u;
            }
            else if (p_terrain->phase_ms >= TERRAIN_CLIMB_TIMEOUT_MS)
            {
                /* Climbing too long without levelling: false trigger. */
                finish_hump(p_terrain, false);
            }
            else
            {
                /* Still climbing. */
            }
            break;

        case TERRAIN_CRESTING:
            add_height(p_terrain, ds_mm);

            if (sine > TERRAIN_CLIMB_ENTER_MILLI)
            {
                p_terrain->state    = TERRAIN_CLIMBING;
                p_terrain->phase_ms = 0u;
            }
            else if (sine < -TERRAIN_CLIMB_ENTER_MILLI)
            {
                p_terrain->state    = TERRAIN_DESCENDING;
                p_terrain->phase_ms = 0u;
            }
            else
            {
                /* On the flat top. */
            }
            break;

        case TERRAIN_DESCENDING:
            add_height(p_terrain, ds_mm);

            if (sine > -TERRAIN_CLIMB_EXIT_MILLI)
            {
                finish_hump(p_terrain, true);
                p_terrain->gate_ms = TERRAIN_LANDING_GATE_MS;
            }
            break;

        default:
            /* CALIBRATING, IMPACT and SENSOR_FAULT are handled elsewhere. */
            break;
    }
}

/** @brief Arming: count height from 2 deg, confirm after 20 mm above 5.
 * Height below the 5-degree threshold is kept, so the start of the ramp is
 * not lost. If the nose drops back under 2 deg first, it was not a hump.
 */
static void step_level(terrain_t * p_terrain, int32_t ds_mm)
{
    int32_t sine = p_terrain->sin_milli;

    if (sine <= TERRAIN_CLIMB_EXIT_MILLI)
    {
        p_terrain->b_arming    = false;
        p_terrain->b_wait_flat = false;
        p_terrain->confirm_mm  = 0u;
        return;
    }

    if (p_terrain->b_wait_flat)
    {
        /* Still tilted after a timeout or impact: one slope must not
         * produce a string of humps. */
        return;
    }

    if (!p_terrain->b_arming)
    {
        p_terrain->b_arming   = true;
        p_terrain->height_cmm = 0;
        p_terrain->peak_cmm   = 0;
        p_terrain->carry_um   = 0;
        p_terrain->confirm_mm = 0u;
    }

    add_height(p_terrain, ds_mm);

    /* A car leaning over 3 deg is not climbing with both wheels: the nose
     * moves around on a one-wheel ramp too, and arm_side() decides that case.
     */
    if ((sine > TERRAIN_CLIMB_ENTER_MILLI) && (ds_mm > 0) &&
        (magnitude(p_terrain->roll_milli) <= TERRAIN_SIDE_ENTER_MILLI))
    {
        uint32_t travel       = p_terrain->confirm_mm + (uint32_t)ds_mm;
        p_terrain->confirm_mm = (travel > TERRAIN_CONFIRM_MAX_MM)
                                    ? TERRAIN_CONFIRM_MAX_MM
                                    : travel;
    }

    if (p_terrain->confirm_mm >= TERRAIN_CONFIRM_MM)
    {
        p_terrain->b_arming     = false;
        p_terrain->state        = TERRAIN_CLIMBING;
        p_terrain->b_on_hump    = true;
        p_terrain->b_hump_valid = true;
        p_terrain->b_started    = true;
        p_terrain->phase_ms     = 0u;
        p_terrain->hump_ms      = 0u;
        p_terrain->side_wheel   = TERRAIN_WHEEL_NONE;
    }
}

/** @brief Confirm a one-wheel hump: lean over the level for a short travel.
 * Only forward travel counts, so lifting the car by hand is not a hump. Travel
 * while upright counts back down instead of clearing the count: on the real
 * ramp, vibration dips the lean for a moment, while on the flat the lean is
 * upright far more often than it is over the level. A lean changing side is
 * rocking, not a ramp, so that restarts the count. The tallest height seen
 * here is kept, so a short ramp's top is not lost before the hump is
 * confirmed.
 */
static void arm_side(terrain_t * p_terrain, int32_t ds_mm)
{
    int32_t lean = magnitude(p_terrain->roll_milli);

    if (lean <= TERRAIN_SIDE_EXIT_MILLI)
    {
        p_terrain->b_side_wait = false;
    }

    if ((p_terrain->b_side_wait) || (p_terrain->b_wait_flat) ||
        (magnitude(filtered_turn_dps(p_terrain)) >= TERRAIN_SIDE_TURN_DPS))
    {
        return;
    }

    if ((ds_mm > 0) &&
        (magnitude(p_terrain->accel_kin_mg) < TERRAIN_SIDE_ACCEL_MG))
    {
        int32_t travel = (int32_t)p_terrain->side_confirm_mm;
        bool    b_left = (p_terrain->roll_milli > 0);

        if (lean > TERRAIN_SIDE_ENTER_MILLI)
        {
            travel = (b_left == p_terrain->b_confirm_left) ? travel : 0;
            p_terrain->b_confirm_left = b_left;
            travel += ds_mm;
        }
        else if (lean <= TERRAIN_SIDE_EXIT_MILLI)
        {
            travel -= ds_mm;
        }
        else
        {
            /* Between the two levels: no evidence either way. */
        }

        p_terrain->side_confirm_mm =
            (uint32_t)clamp(travel, 0, (int32_t)TERRAIN_CONFIRM_MAX_MM);
    }

    int32_t height = side_height(lean);

    if (0u == p_terrain->side_confirm_mm)
    {
        p_terrain->side_arm_peak_cmm = 0;
    }
    else if (height > p_terrain->side_arm_peak_cmm)
    {
        p_terrain->side_arm_peak_cmm = (int16_t)height;
    }
    else
    {
        /* Still below the tallest height seen so far. */
    }

    if (p_terrain->side_confirm_mm >= TERRAIN_SIDE_CONFIRM_MM)
    {
        p_terrain->peak_cmm          = p_terrain->side_arm_peak_cmm;
        p_terrain->side_arm_peak_cmm = 0;
        p_terrain->side_confirm_mm   = 0u;
        p_terrain->b_arming          = false;
        p_terrain->confirm_mm        = 0u;
        p_terrain->b_side            = true;
        p_terrain->side_wheel        = (p_terrain->b_confirm_left)
                                           ? TERRAIN_WHEEL_LEFT
                                           : TERRAIN_WHEEL_RIGHT;
        p_terrain->state             = TERRAIN_CLIMBING;
        p_terrain->b_on_hump         = true;
        p_terrain->b_hump_valid      = true;
        p_terrain->b_started         = true;
        p_terrain->phase_ms          = 0u;
        p_terrain->hump_ms           = 0u;
        p_terrain->height_cmm        = (int16_t)height;
        p_terrain->carry_um          = 0;
    }
}

/** @brief Raised wheel height from the lean: track x sin(lean), in 0.01 mm. */
static int32_t side_height(int32_t lean)
{
    int32_t height =
        ((int32_t)TERRAIN_TRACK_MM * lean) / TERRAIN_SIDE_UM_PER_CMM;
    return clamp(height, 0, INT16_MAX);
}

/** @brief One-wheel hump: follow the lean and keep the highest height.
 * This is the height above the floor the other wheel is on, so it needs no
 * distance and does not depend on how long the ramp took.
 */
static void step_side(terrain_t * p_terrain)
{
    int32_t lean          = magnitude(p_terrain->roll_milli);
    int32_t height        = side_height(lean);
    p_terrain->height_cmm = (int16_t)height;

    if (height > p_terrain->peak_cmm)
    {
        p_terrain->peak_cmm = (int16_t)height;
    }

    /* Vibration dips the lean for a moment mid-ramp, so the hump ends only
     * after the lean has stayed low for a while. */
    bool b_low = (lean < TERRAIN_SIDE_EXIT_MILLI) ||
                 ((height * TERRAIN_PERCENT) <
                  (p_terrain->peak_cmm * TERRAIN_SIDE_END_PERCENT));

    if (!b_low)
    {
        p_terrain->phase_ms = 0u;
    }
    else if (p_terrain->phase_ms >= TERRAIN_SIDE_END_MS)
    {
        finish_hump(p_terrain, true);
        p_terrain->gate_ms = TERRAIN_LANDING_GATE_MS;
    }
    else
    {
        /* Low, but not for long enough yet. */
    }
}

/** @brief h += ds * sin(pitch), at 0.01 mm with a micrometre carry.
 * A 10-degree slope at 0.25 m/s adds about 0.43 mm per step, so a 1-mm
 * accumulator would round every step away (Tribunal 1). Reversing gives a
 * negative ds and lowers the height.
 */
static void add_height(terrain_t * p_terrain, int32_t ds_mm)
{
    int32_t step_um = (ds_mm * p_terrain->sin_milli) + p_terrain->carry_um;
    int32_t height  = p_terrain->height_cmm + (step_um / TERRAIN_UM_PER_CMM);
    /* C99 remainder takes the dividend's sign, so the carry is -9..9. */
    p_terrain->carry_um = (int8_t)(step_um % TERRAIN_UM_PER_CMM);

    if ((height > INT16_MAX) || (height < INT16_MIN))
    {
        /* +/-327 mm: far beyond any practice ramp. Saturate, never wrap. */
        p_terrain->b_hump_valid = false;
        height                  = clamp(height, INT16_MIN, INT16_MAX);
    }

    p_terrain->height_cmm = (int16_t)height;

    if (height > p_terrain->peak_cmm)
    {
        p_terrain->peak_cmm = (int16_t)height;
    }
}

/** @brief End the hump: report its peak and update the run maximum. */
static void finish_hump(terrain_t * p_terrain, bool b_valid)
{
    uint32_t peak_mm = terrain_peak_mm(p_terrain->peak_cmm);
    /* A saturated 255 is a lower bound, not a measurement. */
    bool b_in_range =
        (p_terrain->peak_cmm < ((TERRAIN_PEAK_MAX_MM * TERRAIN_CMM_PER_MM) +
                                (TERRAIN_CMM_PER_MM / 2)));
    p_terrain->b_hump_valid =
        (b_valid) && (p_terrain->b_hump_valid) && (b_in_range);
    p_terrain->hump_peak_mm = peak_mm;

    if ((p_terrain->b_hump_valid) && (peak_mm >= p_terrain->run_peak_mm))
    {
        p_terrain->run_peak_mm  = peak_mm;
        p_terrain->b_peak_valid = true;
    }

    p_terrain->b_finished      = true;
    p_terrain->b_on_hump       = false;
    p_terrain->b_arming        = false;
    p_terrain->b_wait_flat     = true;
    p_terrain->b_side          = false;
    p_terrain->b_side_wait     = true;
    p_terrain->side_confirm_mm = 0u;
    p_terrain->state           = TERRAIN_LEVEL;
    p_terrain->phase_ms        = 0u;
    p_terrain->hump_ms         = 0u;
    p_terrain->height_cmm      = 0;
    p_terrain->carry_um        = 0;
}

/** @brief Re-measure "flat" slowly, only when flat, level and steady.
 * This gives pitch relative to the floor just before the hump (Tribunal 1)
 * and tracks slow drift. It freezes during a hump and while arming.
 */
static void update_baseline(terrain_t *             p_terrain,
                            const terrain_input_t * p_input)
{
    bool b_flat =
        (TERRAIN_LEVEL == p_terrain->state) && (!p_terrain->b_arming) &&
        (p_input->b_motion_fresh) &&
        (magnitude(p_terrain->accel_kin_mg) < TERRAIN_BASELINE_ACCEL_MG) &&
        (magnitude(p_terrain->sin_milli) < TERRAIN_CLIMB_EXIT_MILLI) &&
        (magnitude(p_terrain->roll_milli) < TERRAIN_SIDE_ENTER_MILLI) &&
        (0u == p_terrain->side_confirm_mm);

    if (b_flat)
    {
        int32_t level = (p_terrain->forward_x8 / TERRAIN_FAST_DIVISOR) -
                        p_terrain->accel_kin_mg;
        p_terrain->baseline_x256 +=
            level - (p_terrain->baseline_x256 / TERRAIN_BASELINE_DIVISOR);

        if (magnitude(filtered_turn_dps(p_terrain)) < TERRAIN_SIDE_TURN_DPS)
        {
            int32_t side = (p_terrain->left_x6 / TERRAIN_SIDE_FAST_DIVISOR) -
                           turning_mg(p_terrain);
            p_terrain->baseline_left_x256 +=
                side -
                (p_terrain->baseline_left_x256 / TERRAIN_BASELINE_DIVISOR);
        }
    }
}

/** @brief Pick one motion class, most important first; flag changes. */
static void classify(terrain_t * p_terrain, const terrain_input_t * p_input,
                     int32_t forward)
{
    uint32_t motion = TERRAIN_MOTION_CRUISING;
    int32_t  speed  = p_terrain->speed_x8 / TERRAIN_FAST_DIVISOR;
    int32_t  smooth = p_terrain->forward_x8 / TERRAIN_FAST_DIVISOR;

    if (p_input->b_motion_fresh)
    {
        /* Encoder differential: Ernest's Tribunal position was that it is
         * more reliable than the magnetometer near motors and lab steel. */
        int32_t difference = p_input->right_mm_s - p_input->left_mm_s;
        p_terrain->turn_dps =
            (int16_t)((difference * TERRAIN_DEG_PER_RAD_X1000) /
                      ((int32_t)TERRAIN_TRACK_MM * TERRAIN_MILLI));
    }

    if ((!p_terrain->b_calibrated) ||
        (TERRAIN_SENSOR_FAULT == p_terrain->state))
    {
        motion = TERRAIN_MOTION_UNKNOWN;
    }
    else if (TERRAIN_IMPACT == p_terrain->state)
    {
        motion = TERRAIN_MOTION_IMPACT;
    }
    else if ((TERRAIN_CLIMBING == p_terrain->state) ||
             (TERRAIN_CRESTING == p_terrain->state))
    {
        motion = TERRAIN_MOTION_CLIMBING;
    }
    else if (TERRAIN_DESCENDING == p_terrain->state)
    {
        motion = TERRAIN_MOTION_DESCENDING;
    }
    else if (magnitude(p_terrain->turn_dps) >= TERRAIN_TURN_CLASS_DPS)
    {
        motion = TERRAIN_MOTION_TURNING;
    }
    else if (magnitude(p_terrain->accel_kin_mg) >= TERRAIN_ACCEL_CLASS_MG)
    {
        motion = TERRAIN_MOTION_ACCELERATING;
    }
    else if ((magnitude(speed) <= TERRAIN_STILL_SPEED_MM_S) &&
             (magnitude(forward - smooth) <= TERRAIN_STILL_JITTER_MG))
    {
        motion = TERRAIN_MOTION_STATIONARY;
    }
    else
    {
        /* Moving steadily on the flat. */
    }

    p_terrain->b_motion_changed = (motion != p_terrain->motion);
    p_terrain->motion           = motion;
}

/** @brief 10-ms timers; each saturates below its bit-field limit. */
static void advance_timers(terrain_t * p_terrain)
{
    uint32_t period = ROBOT_IMU_PERIOD_MS;
    uint32_t phase  = p_terrain->phase_ms + period;
    p_terrain->phase_ms =
        (phase > TERRAIN_PHASE_MAX_MS) ? TERRAIN_PHASE_MAX_MS : phase;

    if (p_terrain->b_on_hump)
    {
        uint32_t hump = p_terrain->hump_ms + period;
        p_terrain->hump_ms =
            (hump > TERRAIN_HUMP_TIMEOUT_MS) ? TERRAIN_HUMP_TIMEOUT_MS : hump;
    }

    p_terrain->gate_ms =
        (p_terrain->gate_ms > period) ? (p_terrain->gate_ms - period) : 0u;
}

/*** End of file. ***/

