/** @file robot_config.h
 * @brief Measurable limits and hardware assumptions for CSC20.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#if (OBSTACLE_TEST || LINE_TEST)
/* Bench tests: slow enough to watch, and the search gets time to finish. */
#define ROBOT_CRUISE_MM_SECOND 70u
#define ROBOT_CAUTIOUS_MM_SECOND 80u
#define ROBOT_TURN_MM_SECOND 70u
#define ROBOT_SEARCH_MS 5000u
#else
#define ROBOT_CRUISE_MM_SECOND 500u
#define ROBOT_CAUTIOUS_MM_SECOND 250u
#define ROBOT_TURN_MM_SECOND 150u
#define ROBOT_SEARCH_MS 3000u
#endif /* OBSTACLE_TEST || LINE_TEST */
#define ROBOT_WHEEL_CIRCUM_UM 204200u
/* Measured: one wheel turn is 2400 encoder edges. Host tests override this
 * and the track with the reference values their expectations were written
 * for (960 edges, 140 mm), so they test logic, not this robot's numbers. */
#ifndef ROBOT_ENCODER_EDGES_TURN
#define ROBOT_ENCODER_EDGES_TURN 2400u

/* Per-sensor black thresholds; each sensor reads white differently. */
#define ROBOT_IR_THRESHOLD_LEFT 450u
#define ROBOT_IR_THRESHOLD_CENTRE 300u
#define ROBOT_IR_THRESHOLD_RIGHT ROBOT_IR_THRESHOLD
#endif /* ROBOT_ENCODER_EDGES_TURN */
/* Estimated from the turn tests (a 280 setting gave 90 degrees with the old
 * 960 scale, so 280 / 2.5 = 112). Replace with the measured wheel spacing. */
#ifndef ROBOT_TRACK_MM
#define ROBOT_TRACK_MM 112u
#endif /* ROBOT_TRACK_MM */
#define ROBOT_MOTOR_LEFT_SIGN 1
#define ROBOT_MOTOR_RIGHT_SIGN 1
#define ROBOT_ENCODER_LEFT_SIGN 1
#define ROBOT_ENCODER_RIGHT_SIGN 1
#define ROBOT_LINE_PERIOD_MS 5u
#define ROBOT_MOTION_PERIOD_MS 10u
#define ROBOT_IMU_PERIOD_MS 10u
#define ROBOT_SAFETY_PERIOD_MS 20u
#define ROBOT_RANGE_PERIOD_MS 60u
#define ROBOT_TELEMETRY_PERIOD_MS 250u
#define ROBOT_STALL_MS 200u
#define ROBOT_SENSOR_AGE_MS 20u
#define ROBOT_LINE_LOST_MS 150u
#define ROBOT_SCAN_MS 5000u
#define ROBOT_MANOEUVRE_MS 10000u
#define ROBOT_WARNING_MM 600u
#define ROBOT_OBSTACLE_MM 300u
#define ROBOT_CLEARANCE_MM 120u
/* Sensor-guided bypass: move for STEP_MS, stop, check the obstacle, repeat.
 * After the obstacle is clear, one margin move covers the robot's body. */
#define ROBOT_BYPASS_STEP_MS 1000u
#define ROBOT_BYPASS_SIDE_MARGIN_MM 150u
#define ROBOT_BYPASS_FWD_MARGIN_MM 200u
#define ROBOT_BYPASS_MAX_STEPS 15u
#define ROBOT_RANGE_MAX_CM 400u
#define ROBOT_IR_THRESHOLD 450u
/* Line following mode. 1: the tape runs between the left (GP26) and centre
 * (GP27) sensors, so white/white means on the line and goes straight; a sensor
 * that sees black steers toward its side until both are white again; and
 * black/black is a junction, which goes straight for now. 0: the original
 * proportional steering. Host tests use 0, the mode they were written for. */
#ifndef ROBOT_LINE_STRADDLE
#define ROBOT_LINE_STRADDLE 1u
#endif /* ROBOT_LINE_STRADDLE */
/* Steering in mode 1: one wheel speeds up and the other slows by this much. */
#define ROBOT_LINE_STEER_MM_SECOND 70u
#define ROBOT_IMPACT_MG 2500
#define ROBOT_IMU_CAL_SAMPLES 32u
#define ROBOT_START_PIN 20u
#define ROBOT_STOP_PIN 21u
#define ROBOT_RESET_PIN 22u
#define ROBOT_TRACE_PIN 18u
#define ROBOT_BYTE_BITS 8u
#define ROBOT_WHEEL_COUNT 2u
#define ROBOT_IR_COUNT 3u

#endif /* ROBOT_CONFIG_H */

/*** End of file. ***/

