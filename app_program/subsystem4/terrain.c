/** @file terrain.c
 * @brief Height integrates encoder path times gravity-axis sine.
 * NOTE: LSM303DLHC has no gyro. See docs/design_review.md, section 7.
 */
#include "terrain.h"

#include "../data_formats.h"
#include "../robot_config.h"

#include <string.h>

#define TERRAIN_PITCH_LIMIT 64u
static const uint16_t g_sines[TERRAIN_PITCH_LIMIT + 1u] = {
    0u,   9u,   17u,  26u,  35u,  44u,  52u,  61u,  70u,  78u,  87u,
    96u,  105u, 113u, 122u, 131u, 139u, 148u, 156u, 165u, 174u, 182u,
    191u, 199u, 208u, 216u, 225u, 233u, 242u, 250u, 259u, 267u, 276u,
    284u, 292u, 301u, 309u, 317u, 326u, 334u, 342u, 350u, 358u, 367u,
    375u, 383u, 391u, 399u, 407u, 415u, 423u, 431u, 438u, 446u, 454u,
    462u, 469u, 477u, 485u, 492u, 500u, 508u, 515u, 522u, 530u};

static int32_t  magnitude(int32_t value);
static uint32_t pitch_field(int32_t gravity_x);
static void     update_tracking(terrain_t * p_terrain, int32_t gravity_x,
                                int16_t travel);
static void calibrate(terrain_t * p_terrain, int16_t accel_x, int16_t accel_z,
                      int16_t speed);
static void update_height(terrain_t * p_terrain, int32_t gravity_x,
                          int16_t travel);
static void update_phase(terrain_t * p_terrain, int32_t gravity_x);

/** @brief Biases encode a signed baseline and speed without signed fields. */
void terrain_init (terrain_t * p_terrain)
{
    memset(p_terrain, 0, sizeof(*p_terrain));
    p_terrain->baseline_mg = 500u;
    p_terrain->last_speed  = 512u;
    p_terrain->pitch       = 64u;
    p_terrain->fractional  = 9u;
}

/** @brief Detect impact before compensation, filtering or height work. */
void terrain_step (terrain_t * p_terrain, int16_t accel_x, int16_t accel_y,
                  int16_t accel_z, int16_t speed, uint16_t distance,
                  bool b_fresh)
{
    p_terrain->b_started  = false;
    p_terrain->b_finished = false;
    /* Widen before multiplication, covering every signed 16-bit input. */
    int64_t norm_squared = ((int64_t)accel_x * accel_x) +
                           ((int64_t)accel_y * accel_y) +
                           ((int64_t)accel_z * accel_z);
    p_terrain->b_impact  = b_fresh && (norm_squared > 6250000);

    if (!b_fresh)
    {
        uint32_t failures    = p_terrain->failures + 1u;
        p_terrain->failures  = (failures > 3u) ? 3u : failures;
        p_terrain->b_valid   = false;
        p_terrain->b_on_hump = false;
    }
    else
    {
        p_terrain->failures = 0u;
    }

    if (p_terrain->b_impact)
    {
        p_terrain->state   = TERRAIN_IMPACT;
        p_terrain->b_valid = false;
    }
    else if (p_terrain->failures >= 3u)
    {
        p_terrain->state = TERRAIN_SENSOR_FAULT;
    }
    else if (b_fresh)
    {
        /* Offset speed and baseline fit comfortably within int32_t. */
        int32_t last_speed   = (int32_t)p_terrain->last_speed - 512;
        int32_t acceleration = ((speed - last_speed) * 10000) / 9807;
        int32_t baseline     = (int32_t)p_terrain->baseline_mg - 500;
        int32_t gravity_x    = accel_x - acceleration - baseline;
        int16_t travel =
            data_formats_distance_delta(distance, p_terrain->last_distance);
        if (TERRAIN_SENSOR_FAULT == p_terrain->state)
        {
            terrain_init(p_terrain);
        }

        if (TERRAIN_CALIBRATING == p_terrain->state)
        {
            calibrate(p_terrain, accel_x, accel_z, speed);
        }
        else
        {
            update_tracking(p_terrain, gravity_x, travel);
        }

        p_terrain->pitch         = pitch_field(gravity_x);
        p_terrain->last_speed    = (uint32_t)(speed + 512);
        p_terrain->last_distance = distance;
    }
    else
    {
        /* One or two failed samples cannot supply trustworthy height. */
    }
}

/** @brief Bound the whole-hump timer independently of its climbing phase. */
static void update_tracking (terrain_t * p_terrain, int32_t gravity_x,
                            int16_t travel)
{
    p_terrain->b_valid = !p_terrain->b_overflow;
    update_height(p_terrain, gravity_x, travel);
    update_phase(p_terrain, gravity_x);

    if (p_terrain->b_on_hump)
    {
        p_terrain->hump_ms += ROBOT_IMU_PERIOD_MS;
    }

    if (p_terrain->hump_ms >= 10000u)
    {
        p_terrain->b_valid   = false;
        p_terrain->b_on_hump = false;
        p_terrain->state     = TERRAIN_LEVEL;
        p_terrain->hump_ms   = 0u;
    }
}

/** @brief Stationary calibration requires 32 consecutive flat samples. */
static void calibrate (terrain_t * p_terrain, int16_t accel_x, int16_t accel_z,
                      int16_t speed)
{
    bool b_stationary = (magnitude(speed) <= 10) &&
                        (magnitude(accel_z - 1000) < 100) &&
                        (magnitude(accel_x) <= 500);

    if (b_stationary)
    {
        /* +/-500 mg plus the bias is nonnegative and fits the sum. */
        p_terrain->calibration_sum += (uint32_t)(accel_x + 500);
        ++p_terrain->samples;
    }
    else
    {
        p_terrain->samples         = 0u;
        p_terrain->calibration_sum = 0u;
    }

    if (p_terrain->samples >= ROBOT_IMU_CAL_SAMPLES)
    {
        p_terrain->baseline_mg = p_terrain->calibration_sum / 32u;
        p_terrain->state       = TERRAIN_LEVEL;
        p_terrain->b_valid     = true;
    }
}

/** @brief Nearest fixed-point sine table avoids floating-point dependencies.
 */
static uint32_t pitch_field (int32_t gravity_x)
{
    uint32_t absolute = 0u;
    /* Sensor compensation is bounded well within the int32_t magnitude. */
    absolute       = (uint32_t)magnitude(gravity_x);
    absolute       = (absolute > g_sines[TERRAIN_PITCH_LIMIT])
                         ? g_sines[TERRAIN_PITCH_LIMIT]
                         : absolute;
    uint32_t index = 0u;

    while ((index < TERRAIN_PITCH_LIMIT) && (absolute > g_sines[index]))
    {
        ++index;
    }

    if ((index > 0u) &&
        ((absolute - g_sines[index - 1u]) < (g_sines[index] - absolute)))
    {
        --index;
    }

    index = ((gravity_x >= 0) && (index > 63u)) ? 63u : index;
    return (gravity_x < 0) ? (64u - index) : (64u + index);
}

/** @brief Inputs are bounded signed sensor and speed differences. */
static int32_t magnitude (int32_t value)
{
    return (value < 0) ? -value : value;
}

/** @brief A 0.001-mm carry prevents repetitive sub-centimetre truncation. */
static void update_height (terrain_t * p_terrain, int32_t gravity_x,
                          int16_t travel)
{
    if ((p_terrain->b_on_hump) && (travel > 0))
    {
        /* Stored height is 0..65535 and fractional carry is 0..9. */
        int32_t fine =
            (gravity_x * travel) + (int32_t)p_terrain->fractional - 9;
        int32_t height = (int32_t)p_terrain->height_centi_mm + (fine / 10);
        height         = (height < 0) ? 0 : height;
        p_terrain->b_overflow = (height > 65535);
        height                = (height > 65535) ? 65535 : height;
        /* Saturation and positive remainder bound both unsigned fields. */
        p_terrain->height_centi_mm = (uint32_t)height;
        p_terrain->fractional      = (uint32_t)((fine % 10) + 9);
        uint32_t peak              = p_terrain->peak_centi_mm;
        p_terrain->peak_centi_mm =
            (height > (int32_t)peak) ? (uint32_t)height : peak;
        p_terrain->b_overflow =
            (p_terrain->b_overflow) || ((p_terrain->peak_centi_mm > 25500u));
        uint32_t report    = (p_terrain->peak_centi_mm + 50u) / 100u;
        p_terrain->peak_mm = (report > 255u) ? 255u : report;
        p_terrain->b_valid = !p_terrain->b_overflow;
    }
}

/** @brief Hysteresis avoids repeated starts around the five-degree threshold.
 */
static void update_phase (terrain_t * p_terrain, int32_t gravity_x)
{
    switch (p_terrain->state)
    {
        case TERRAIN_LEVEL:
            if (gravity_x > 87)
            {
                p_terrain->state           = TERRAIN_CLIMBING;
                p_terrain->b_on_hump       = true;
                p_terrain->b_started       = true;
                p_terrain->height_centi_mm = 0u;
                p_terrain->climb_ms        = 0u;
                p_terrain->hump_ms         = 0u;
                p_terrain->fractional      = 9u;
            }
        break;

        case TERRAIN_CLIMBING:
            p_terrain->climb_ms += ROBOT_IMU_PERIOD_MS;

            if (gravity_x < 35)
            {
                p_terrain->state = TERRAIN_CRESTING;
            }
            else if (p_terrain->climb_ms >= 3000u)
            {
                p_terrain->state     = TERRAIN_LEVEL;
                p_terrain->b_on_hump = false;
                p_terrain->b_valid   = false;
            }
            else
            {
                /* The climb has not reached its crest. */
            }
        break;

        case TERRAIN_CRESTING:
            if (gravity_x < -87)
            {
                p_terrain->state = TERRAIN_DESCENDING;
            }
        break;

        case TERRAIN_DESCENDING:
            if (gravity_x > -35)
            {
                p_terrain->state      = TERRAIN_LEVEL;
                p_terrain->b_on_hump  = false;
                p_terrain->b_finished = true;
            }
        break;

        default:
            p_terrain->b_valid = false;
        break;
    }
}

/*** End of file. ***/

