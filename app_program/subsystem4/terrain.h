/** @file terrain.h
 * @brief IMU terrain monitor: pitch, hump height, impact and motion class.
 *
 * Pure logic with no hardware access, so host tests can feed it synthetic
 * samples. Every 10 ms robot_os gathers one terrain_input_t (an IMU sample
 * from drv_lsm303 plus Motion's odometer and speed) and calls
 * terrain_step().
 *
 * Height method (Week 6 design, accepted in the lecturer's feedback):
 * h = sum(ds * sin(pitch)). ds comes from the wheel encoders and sin(pitch)
 * from the direction of gravity seen by the accelerometer. Acceleration is
 * never integrated twice, because that drifts.
 */
#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdbool.h>
#include <stdint.h>

/* Statechart 6.5 states. 0..6 is also the telemetry "terrain" field. */
#define TERRAIN_CALIBRATING 0u
#define TERRAIN_LEVEL 1u
#define TERRAIN_CLIMBING 2u
#define TERRAIN_CRESTING 3u
#define TERRAIN_DESCENDING 4u
#define TERRAIN_IMPACT 5u
#define TERRAIN_SENSOR_FAULT 6u

/* Motion classes (Project Write-up "motion events"), 3 bits. */
#define TERRAIN_MOTION_UNKNOWN 0u
#define TERRAIN_MOTION_STATIONARY 1u
#define TERRAIN_MOTION_CRUISING 2u
#define TERRAIN_MOTION_ACCELERATING 3u
#define TERRAIN_MOTION_TURNING 4u
#define TERRAIN_MOTION_CLIMBING 5u
#define TERRAIN_MOTION_DESCENDING 6u
#define TERRAIN_MOTION_IMPACT 7u

#define TERRAIN_AXES 3u

/** @brief One 10-ms sample, gathered by the caller from IMU and Motion. */
typedef struct
{
    int16_t  accel_mg[TERRAIN_AXES]; /* chip X, Y, Z in milli-g */
    int16_t  speed_mm_s;             /* signed centre speed from Motion */
    int16_t  left_mm_s;              /* signed wheel speeds, for turn rate */
    int16_t  right_mm_s;
    uint32_t distance_mm : 12;   /* Motion odometer, wraps at 4096 mm */
    bool     b_imu_ok : 1;       /* false: the I2C read failed */
    bool     b_motion_fresh : 1; /* false: Motion data older than 20 ms */
} terrain_input_t;

/** @brief Task-local terrain state. Not shared, so no lock is needed.
 *
 * Bounded states, counters and flags are packed into unsigned bit-fields.
 * Signed working values (filters, height) use plain fixed-width integers:
 * BARR-C forbids signed bit-fields, and offset-encoding them would add an
 * add/subtract to every 10-ms step for no RAM saving worth having.
 */
typedef struct
{
    uint32_t state : 3;          /* TERRAIN_CALIBRATING..SENSOR_FAULT */
    uint32_t motion : 3;         /* TERRAIN_MOTION_* */
    uint32_t failures : 2;       /* consecutive I2C failures, 0..3 */
    uint32_t cal_samples : 6;    /* still samples collected, 0..32 */
    uint32_t stale_run : 4;      /* consecutive stale Motion samples, 0..15 */
    uint32_t confirm_mm : 6;     /* travel above 5 deg while arming, 0..63 */
    uint32_t last_distance : 12; /* odometer at the last used sample */
    uint32_t phase_ms : 14;      /* time in the current phase, 0..10000 */
    uint32_t hump_ms : 14;       /* time since the hump started, 0..10000 */
    uint32_t gate_ms : 9;        /* landing-gate time left, 0..300 */
    uint32_t impact_run : 2;     /* samples above impact threshold, 0..3 */
    uint32_t pitch_field : 7;    /* telemetry pitch, 0.5 deg, offset 64 */
    uint32_t hump_peak_mm : 8;   /* peak of the last finished hump */
    uint32_t run_peak_mm : 8;    /* highest valid peak of the run */
    bool     b_calibrated : 1;   /* boot calibration has completed */
    bool     b_arming : 1;       /* nose above 2 deg, hump not confirmed */
    bool     b_wait_flat : 1;    /* hump ended: re-arm only after flat */
    bool     b_on_hump : 1;      /* ON_HUMP: confirmed climb until LEVEL */
    bool     b_hump_valid : 1;   /* current/last hump is trustworthy */
    bool     b_peak_valid : 1;   /* run_peak_mm comes from a valid hump */
    bool     b_have_last : 1;    /* last_distance holds a real sample */
    bool     b_started : 1;      /* event this step: hump confirmed */
    bool     b_finished : 1;     /* event this step: hump ended */
    bool     b_impact : 1;       /* event this step: impact detected */
    bool     b_motion_changed : 1; /* event this step: motion changed */
    bool     b_request_init : 1;   /* caller must (re)initialise the chip */
    int16_t  height_cmm;           /* running height of this hump, 0.01 mm */
    int16_t  peak_cmm;             /* highest height of this hump, 0.01 mm */
    int8_t   carry_um;             /* remainder below 0.01 mm, -9..9 um */
    int16_t  forward_x8;           /* filtered forward accel, mg x 8 */
    int16_t  speed_x8;             /* filtered speed, mm/s x 8 */
    int16_t  accel_kin_mg;         /* filtered encoder acceleration, mg */
    int16_t  sin_milli;            /* pitch vs flat baseline, sin x 1000 */
    int16_t  turn_dps;             /* yaw rate from wheel speeds, deg/s */
    int32_t  baseline_x256;        /* flat-ground forward reading, mg x 256 */
    int32_t  cal_forward_sum;      /* calibration sums, mg */
    int32_t  cal_up_sum;
} terrain_t;

/** @brief Start calibrating and ask the caller to initialise the IMU. */
void terrain_init(terrain_t * p_terrain);
/** @brief Advance the terrain statechart by one 10-ms sample. */
void terrain_step(terrain_t * p_terrain, const terrain_input_t * p_input);
/** @brief True once Safety may stop waiting: calibrated or faulted. */
bool terrain_ready(const terrain_t * p_terrain);
/** @brief Encode pitch in 0.01 deg as 7 bits: 0.5 deg steps, offset 64.
 * Saturates at -32.0 and +31.5 deg; never wraps.
 */
uint32_t terrain_pitch_field(int32_t pitch_cdeg);
/** @brief Decode a 7-bit pitch field back to 0.01 deg. */
int32_t terrain_pitch_cdeg(uint32_t field);
/** @brief Round a height in 0.01 mm to whole mm, saturating at 0..255. */
uint32_t terrain_peak_mm(int32_t height_cmm);
/** @brief Pack a hump event: finished bit 0, peak bits 1-8, valid bit 9. */
uint16_t terrain_event_pack(bool b_finished, uint32_t peak_mm, bool b_valid);
/** @brief Unpack a hump event; false if any reserved bit is set. */
bool terrain_event_unpack(uint16_t event, bool * pb_finished,
                          uint32_t * p_peak_mm, bool * pb_valid);
/** @brief Integer atan2 in 0.01 deg, error under 0.1 deg.
 * @pre |y| and |x| are below 2^19.
 */
int32_t terrain_atan2_cdeg(int32_t y, int32_t x);
/** @brief Integer square root, rounded down. */
uint32_t terrain_isqrt(uint32_t value);

#endif /* TERRAIN_H */

/*** End of file. ***/

