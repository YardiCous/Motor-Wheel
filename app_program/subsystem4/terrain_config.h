/** @file terrain_config.h
 * @brief Tunable terrain constants, kept in one place for bench tuning.
 *
 * PROVISIONAL means a starting value from the design documents or from
 * arithmetic, not a measurement. Replace each one with a value taken from
 * the bench logs (see ~/inf2004/notes/TEST_PLAN_BUDDY4.md) and record the
 * change in the decision log.
 */
#ifndef TERRAIN_CONFIG_H
#define TERRAIN_CONFIG_H

/* Mounting: which chip axis (0 = X, 1 = Y, 2 = Z) points forward, left and
 * up on the car, and whether it reads positive in that direction.
 * PROVISIONAL: set from the static tilt test (bench step 4). */
#define TERRAIN_AXIS_FORWARD 0u
#define TERRAIN_AXIS_LEFT 1u
#define TERRAIN_AXIS_UP 2u
#define TERRAIN_SIGN_FORWARD 1
#define TERRAIN_SIGN_LEFT 1
#define TERRAIN_SIGN_UP 1

/* Readings are clamped to this so the x8 filter state fits an int16_t. */
#define TERRAIN_ACCEL_LIMIT_MG 4000

/* Boot calibration: 32 still samples = 320 ms. The up axis must read close
 * to 1 g, otherwise the axis mapping above is wrong. */
#define TERRAIN_CAL_SAMPLES 32u
#define TERRAIN_STILL_SPEED_MM_S 10
#define TERRAIN_STILL_JITTER_MG 30 /* PROVISIONAL: from 30-s rest log */
#define TERRAIN_GRAVITY_LOW_MG 900
#define TERRAIN_GRAVITY_HIGH_MG 1100

/* Low-pass filters (W9 lecture: exponential moving average). The same
 * divisor is used for the accelerometer and for encoder speed, so the two
 * signals are delayed equally before one is subtracted from the other.
 * 8 at 10 ms gives a time constant of about 80 ms. PROVISIONAL. */
#define TERRAIN_FAST_DIVISOR 8
/* Flat-ground baseline: 256 x 10 ms = about 2.6 s time constant. */
#define TERRAIN_BASELINE_DIVISOR 256
/* Baseline only learns while speed is steady (|accel| below this). */
#define TERRAIN_BASELINE_ACCEL_MG 20

/* Hump thresholds in sin(pitch) x 1000, which equals milli-g along the
 * forward axis. 87 = sin(5 deg) from Week 6; 35 = sin(2 deg) hysteresis.
 * PROVISIONAL: must sit below the measured ramp angle with margin. */
#define TERRAIN_CLIMB_ENTER_MILLI 87
#define TERRAIN_CLIMB_EXIT_MILLI 35
/* Travel above 5 deg needed to confirm a hump (rejects short spikes).
 * Must stay below 64 (6-bit counter). PROVISIONAL. */
#define TERRAIN_CONFIRM_MM 20u
/* Statechart 6.5: CLIMBING longer than 3 s without levelling off is a false
 * trigger. Whole-hump limit ends a hump that never comes back down. */
#define TERRAIN_CLIMB_TIMEOUT_MS 3000u
#define TERRAIN_HUMP_TIMEOUT_MS 10000u
/* Longest run of stale Motion samples a hump tolerates (100 ms). The
 * odometer is cumulative, so short gaps lose no distance. */
#define TERRAIN_STALE_LIMIT 10u

/* Impact: horizontal (forward/left) acceleration magnitude. Landings at the
 * bottom of a hump are mostly vertical, and the threshold is raised during
 * the descent and just after it. PROVISIONAL: from tap and flat-run logs. */
#define TERRAIN_IMPACT_MG 1500
#define TERRAIN_LANDING_IMPACT_MG 2500
#define TERRAIN_LANDING_GATE_MS 300u
/* Samples above threshold before IMPACT. At 100 Hz a real hit may last a
 * single sample, so this starts at 1. Raise it if tap logs show false hits. */
#define TERRAIN_IMPACT_SAMPLES 1u
/* IMPACT is held this long so Safety's 20-ms poll also sees it. */
#define TERRAIN_IMPACT_HOLD_MS 200u

/* Statechart 6.5: SENSOR_FAULT after 3 consecutive I2C failures. While in
 * fault, the chip is re-initialised at most every 100 ms (backoff). */
#define TERRAIN_FAIL_LIMIT 3u
#define TERRAIN_REINIT_PERIOD_MS 100u

/* Motion classification thresholds. PROVISIONAL. */
#define TERRAIN_ACCEL_CLASS_MG 30 /* about 0.3 m/s^2 */
#define TERRAIN_TURN_CLASS_DPS 20

#endif /* TERRAIN_CONFIG_H */

/*** End of file. ***/

