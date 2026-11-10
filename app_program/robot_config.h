/** @file robot_config.h
 * @brief Measurable limits and hardware assumptions for CSC20.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
/* Every build drives at 50 mm/s: line following, slow zones and turns.
 * Host tests override these with the values their expectations were
 * written for (500, 250, 150). */
#ifndef ROBOT_CRUISE_MM_SECOND
#define ROBOT_CRUISE_MM_SECOND 50u
#endif /* ROBOT_CRUISE_MM_SECOND */
#ifndef ROBOT_CAUTIOUS_MM_SECOND
#define ROBOT_CAUTIOUS_MM_SECOND 50u
#endif /* ROBOT_CAUTIOUS_MM_SECOND */
#ifndef ROBOT_TURN_MM_SECOND
#define ROBOT_TURN_MM_SECOND 50u
#endif /* ROBOT_TURN_MM_SECOND */
/* The lost-line search turns 90 degrees left then 180 right: about 5.5 s at
 * 50 mm/s, so it gets 7 s. Slow driving also needs more bypass steps. */
#ifndef ROBOT_SEARCH_MS
#define ROBOT_SEARCH_MS 7000u
#endif /* ROBOT_SEARCH_MS */
#define ROBOT_WHEEL_CIRCUM_UM 204200u
/* Measured: one wheel turn is 2400 encoder edges. Host tests override this
 * and the track with the reference values their expectations were written
 * for (960 edges, 140 mm), so they test logic, not this robot's numbers. */
#ifndef ROBOT_ENCODER_EDGES_TURN
#define ROBOT_ENCODER_EDGES_TURN 2400u
#endif /* ROBOT_ENCODER_EDGES_TURN */

#define ROBOT_REBOOT_HOLD_MS 3000u
#define ROBOT_RANGE_POWER_UP_MS 1500u
#define ROBOT_SERVO_SETTLE_MS 300u
#define ROBOT_BARCODE_DRIVE_MM 400u
#define ROBOT_BARCODE_DRIVE_MM_SECOND 50u
/* Per-sensor black thresholds; each sensor reads white differently. */
#define ROBOT_IR_THRESHOLD_LEFT 450u
#define ROBOT_IR_THRESHOLD_CENTRE 300u
#define ROBOT_IR_THRESHOLD_RIGHT 400u
/* Estimated from the turn tests (a 280 setting gave 90 degrees with the old
 * 960 scale, so 280 / 2.5 = 112). Replace with the measured wheel spacing. */
#ifndef ROBOT_TRACK_MM
#define ROBOT_TRACK_MM 112u
#endif /* ROBOT_TRACK_MM */
/* Calibrated wheel travel for a 90-degree turn: 103 * 90 / 95 = 98 mm. */
#ifndef ROBOT_QUARTER_TURN_MM
#define ROBOT_QUARTER_TURN_MM 92u
#endif /* ROBOT_QUARTER_TURN_MM */
#define ROBOT_MOTOR_LEFT_SIGN 1
#define ROBOT_MOTOR_RIGHT_SIGN 1
/* Straight moves: correct 3 mm/s per mm of wheel-distance mismatch,
 * capped at 40 mm/s and half the commanded speed. */
#define ROBOT_MOTOR_FF_OFFSET 250
#define ROBOT_MOTOR_FF_GAIN 150
#define ROBOT_STRAIGHT_GAIN 3
#define ROBOT_STRAIGHT_CORRECTION_MAX 40
/* Residual right drift: right wheel +0.5%, left -0.5% on straight moves.
 * Positive values bias left; zero requests equal encoder travel. */
#define ROBOT_STRAIGHT_TRIM_PERMILLE 5
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
#define ROBOT_SCAN_MS 8000u
#define ROBOT_MANOEUVRE_MS 10000u
#define ROBOT_WARNING_MM 600u
#define ROBOT_OBSTACLE_MM 250u
#define ROBOT_CLEARANCE_MM 120u
/* Robot footprint. The IR sensors sit SENSOR_AHEAD in front of the wheel
 * axle, which is the point the robot turns about. */
#define ROBOT_WIDTH_MM 200u
#define ROBOT_LENGTH_MM 300u
#define ROBOT_SENSOR_AHEAD_MM 100u
/* Wheel axle to the farthest corner: what the robot sweeps when it turns in
 * place. 180 suits a 200 x 300 mm robot turning about its centre. */
#define ROBOT_TURN_RADIUS_MM 180u
/* Bypass: go sideways by the measured edge on the chosen side + half the
 * robot + SIDE_MARGIN; first forward leg is the detection range + robot
 * length; later legs are STEP_MS long; once a side-look is clear, one
 * FWD_MARGIN move carries the robot's body past before a confirming look. */
#define ROBOT_BYPASS_STEP_MS 2000u
#define ROBOT_BYPASS_SIDE_MARGIN_MM 50u
#define ROBOT_BYPASS_FWD_MARGIN_MM ((ROBOT_WIDTH_MM / 2u) + 50u)
/* Extra sideways step when the box is still ahead after the side move. */
#define ROBOT_BYPASS_SIDE_EXTRA_MM 100u
/* The return leg stops on the tape; this is how far past the expected line
 * it may drive before giving up and sweeping for it. */
#if OBSTACLE_NO_LINE
/* No tape to stop on: return exactly the sideways distance, then carry on
 * along the virtual line. */
#define ROBOT_NO_LINE 1u
#define ROBOT_BYPASS_BACK_EXTRA_MM 0u
#else
#define ROBOT_NO_LINE 0u
#define ROBOT_BYPASS_BACK_EXTRA_MM 150u
#endif /* OBSTACLE_NO_LINE */
#ifndef ROBOT_BYPASS_MAX_STEPS
#define ROBOT_BYPASS_MAX_STEPS 15u
#endif /* ROBOT_BYPASS_MAX_STEPS */
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
#define ROBOT_LINE_STEER_MM_SECOND 50u
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

