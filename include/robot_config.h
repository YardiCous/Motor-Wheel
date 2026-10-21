#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
/* Provisional bounds, not measured/calibrated hardware properties. */
#define LINE_PERIOD_MS 5u
#define MOTION_PERIOD_MS 10u
#define IMU_PERIOD_MS 10u
#define SAFETY_PERIOD_MS 20u
#define RANGE_PERIOD_MS 60u
#define TELEMETRY_PERIOD_MS 250u
#define LINE_MAX_AGE_MS 20u
#define RANGE_MAX_AGE_MS 150u
#define LINE_LOST_MS 150u
#define CONTEXT_TIMEOUT_MS 1000u
#define SEARCH_TIMEOUT_MS 3000u
#define STALL_TIMEOUT_MS 200u
#define OBSTACLE_STOP_MM 250u
#define MAX_SPEED_MM_S 250
#define MOTION_QUEUE_DEPTH 4u
#define COMMAND_QUEUE_DEPTH 4u
#define EVENT_QUEUE_DEPTH 8u
#define TELEMETRY_QUEUE_DEPTH 4u
/* Native micro-T: smaller numbers are higher priority. */
#define PRIORITY_SAFETY 1u
#define PRIORITY_CONTROL 2u
#define PRIORITY_NAVIGATION 3u
#define PRIORITY_OBSTACLE 4u
#define PRIORITY_TELEMETRY 5u
#define PIN_US_TRIG 0u
#define PIN_US_ECHO 1u
#define PIN_IMU_SDA 2u
#define PIN_IMU_SCL 3u
#define PIN_ENCODER_L_A 4u
#define PIN_ENCODER_L_B 5u
#define PIN_MOTOR_L_A 8u
#define PIN_MOTOR_L_B 9u
#define PIN_MOTOR_R_A 10u
#define PIN_MOTOR_R_B 11u
#define PIN_SERVO 12u
#define PIN_ENCODER_R_A 16u
#define PIN_ENCODER_R_B 17u
#define PIN_IR_L 26u
#define PIN_IR_C 27u
#define PIN_IR_R 28u
/* Local START/STOP/RESET pins intentionally unassigned until wiring confirmed. */
/* Wiring proposals for active-low buttons; confirm on your board. */
#define PIN_START 18u
#define PIN_STOP 19u
#define PIN_RESET 20u
#define PIN_ESTOP 21u
#define BUTTON_DEBOUNCE_MS 30u
#define PWM_TOP 6249u
#define WHEEL_DIAMETER_UM 65000u
#define TRACK_WIDTH_UM 140000u
#define ENCODER_COUNTS_PER_REV 960u /* quadrature counts, not A-channel pulses */
#define ENCODER_LEFT_SIGN 1
#define ENCODER_RIGHT_SIGN 1
#define MOTOR_LEFT_SIGN 1
#define MOTOR_RIGHT_SIGN 1
/* Q8 gains: duty permille per mm/s; integral is error*ms. */
#define PID_KP_Q8 768
#define PID_KI_Q8 2
#define PID_KD_Q8 0
#define PID_FEEDFORWARD_Q8 512
#define LINE_STEER_GAIN_Q8 32
#define TURN_SPEED_MM_S 90u
#define SEARCH_SPEED_MM_S 60u
#define BYPASS_SPEED_MM_S 80u
#define MOTION_TIMEOUT_MS 15000u
#define MOTION_MAX_AGE_MS 30u
#define BYPASS_FORWARD_MM 450u
#define BYPASS_SIDE_MM 250u
#define OBSTACLE_CLEARANCE_MM 350u
#define SERVO_SETTLE_MS 250u
#define SCAN_TIMEOUT_MS 8000u
#define SCAN_RETRIES 2u
#define ECHO_TIMEOUT_US 25000u
#define IMU_I2C_TIMEOUT_US 2000u
#define IMU_IMPACT_MG 2500u
#define IMU_CALIBRATION_SAMPLES 100u
/* 0=auto detect MPU6050 or LSM303DLHC, 1=MPU6050, 2=LSM303DLHC (GY-511). */
#define IMU_MODEL 0
#define IMU_PITCH_SIGN 1
#define IR_BLACK_HIGH 1
#define IR_THRESHOLD_LEFT 2000u
#define IR_THRESHOLD_CENTER 2000u
#define IR_THRESHOLD_RIGHT 2000u
#define BARCODE_MIN_ELEMENT_MS 10u
#define BARCODE_MAX_ELEMENT_MS 250u
#define BARCODE_FRAME_TIMEOUT_MS 5000u
#define BARCODE_QUIET_MS 40u
/* Implemented profile: Code 39 *L*, *R*, *S*, *U*, *E*, forward direction. */
#define BARCODE_CODE39 1
/* Set after confirming wiring, counts/rev, gains, IR thresholds and course profile.
 * The default image runs diagnostics/tasks but rejects START. */
#ifndef ROBOT_CALIBRATION_CONFIRMED
#define ROBOT_CALIBRATION_CONFIRMED 0
#endif
#if __has_include("robot_local.h")
#include "robot_local.h"
#endif
_Static_assert(ENCODER_COUNTS_PER_REV > 0, "Encoder counts/rev must be calibrated and nonzero");
_Static_assert(MAX_SPEED_MM_S <= 250, "Speed exceeds packed telemetry range");
_Static_assert(SCAN_RETRIES <= 2, "Retry count exceeds configured policy");
_Static_assert(BYPASS_SIDE_MM > 0 && BYPASS_SIDE_MM <= 8191, "Bypass distance out of range");
_Static_assert(BYPASS_FORWARD_MM > 0 && BYPASS_FORWARD_MM <= 8191, "Bypass distance out of range");
_Static_assert(IMU_CALIBRATION_SAMPLES > 0 && IMU_CALIBRATION_SAMPLES <= 100,
               "Calibration counter range");
#endif
