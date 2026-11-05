# Application API

Generated from public header comments by
`python3 tools/generate_docs.py`.

## app_program/app_main.h

`int usermain(void);`

Retain the kernel-required legacy symbol name and signature.

## app_program/board_io.h

`bool board_io_init(int32_t h_echo_flag);`

Echo capture and GPIO evidence for the most recent ping. */
typedef struct
{
    uint32_t width_us;
    uint32_t rises;
    uint32_t falls;
    uint32_t gpio_control;
    uint32_t gpio_pad;
    uint32_t irq_raw;
    uint32_t irq_enabled;
    uint8_t state;
    uint8_t pins;
} board_io_range_diag_t;

/** @brief Configure safe outputs and install T-Kernel-wrapped ISRs on core 0.

`uint32_t board_io_lock(void);`

Save and disable core-0 interrupts for a short packed-data access.

`void board_io_unlock(uint32_t mask);`

Restore exactly the previous interrupt mask.

`uint32_t board_io_time_us(void);`

Read the free-running modulo-2^32 microsecond timer.

`uint32_t board_io_time_ms(void);`

Read the kernel's modulo-2^32 millisecond clock.

`bool board_io_encoders(uint32_t * p_left, uint32_t * p_right);`

Snapshot both 32-bit modulo encoder counters and edge validity.

`uint8_t board_io_encoder_faults(void);`

Return latched invalid-transition bits: bit 0 left, bit 1 right.

`void board_io_clear_fault(void);`

Clear an encoder diagnostic only while motor drive is inhibited.

`void board_io_motor(int16_t left, int16_t right);`

Only the Motion task calls this after startup.

`bool board_io_line(uint16_t * p_samples);`

Sample all three 12-bit ADC channels with a bounded timeout.

`void board_io_i2c_init(void);`

Reset I2C1 to 400-kHz master mode; no bus traffic.

`bool board_io_i2c_read(uint8_t address, uint8_t reg, uint8_t * p_bytes, uint8_t length);`

Read 1..8 consecutive registers from a 7-bit I2C1 target. The caller sets the chip's auto-increment bit in reg when it needs one.

`bool board_io_i2c_write(uint8_t address, uint8_t reg, uint8_t value);`

Write one register on a 7-bit I2C1 target.

`void board_io_servo(uint8_t angle);`

Set a servo angle 0..60 in three-degree units.

`void board_io_range_reset(void);`

Reinitialise ranging GPIO and capture without resetting encoders.

`void board_io_ping(void);`

Drive a 12-us trigger pulse with both edges in a critical section.

`bool board_io_range(uint16_t * p_cm);`

Read captured range; 1 cm means below range, no echo is invalid.

`void board_io_range_diagnostics(board_io_range_diag_t * p_diag);`

Snapshot echo capture and physical pin configuration atomically.

`uint8_t board_io_buttons(void);`

Read active-low local START, STOP, RESET as three individual bits.

`void board_io_trace(bool b_high);`

Mark each motor-loop start on GP18 for timing measurements.

## app_program/data_formats.h

`uint32_t data_formats_read(const volatile uint8_t * p_data, uint16_t offset, uint8_t width);`

Read an unsigned field from a little-endian bit stream. @pre Width is 1..32 and the caller supplies sufficient buffer space.

`bool data_formats_write(volatile uint8_t * p_data, uint16_t offset, uint8_t width, uint32_t value);`

Validate and write a field without modifying adjacent fields.

`uint32_t data_formats_signed(int32_t value, uint8_t width);`

Encode a signed value as portable unsigned offset binary.

`int32_t data_formats_unsign(uint32_t value, uint8_t width);`

Decode offset binary without signed bit manipulation.

`bool data_formats_motion_pack(uint32_t * p_word, uint16_t distance, uint16_t speed, bool b_reverse, uint8_t mode, uint8_t tick);`

Pack distance12, speed9, reverse1, mode3, timestamp7.

`int16_t data_formats_distance_delta(uint16_t newer, uint16_t older);`

Compute the signed shortest displacement in a modulo-4096 ring. @pre Absolute travel between samples must be less than 2048 mm.

`bool data_formats_command_read(uint8_t payload, uint8_t * p_command, uint8_t * p_sequence);`

Reject reserved command bits and return command2 and sequence4.

`uint8_t data_formats_command_make(uint8_t command, uint8_t sequence);`

Build command2, sequence4 and two zero reserved bits.

`bool data_formats_sequence_new(uint8_t newer, uint8_t older);`

Accept forward modulo-16 sequence increments of 1..7 only.

## app_program/robot_os.h

`void robot_os_run(void);`

Create all objects before starting any of the seven tasks.

`bool robot_os_submit(uint8_t payload);`

Nonblocking binary command bridge, called by the network owner.

`bool robot_os_telemetry(uint8_t * p_packet);`

Retrieve the latest bounded 28-byte telemetry message.

`void robot_os_network(bool b_wifi, bool b_mqtt);`

Report connection bits without inhibiting autonomous control.

`bool robot_os_ack(uint8_t * p_ack);`

Consume one binary acknowledgement for robot/ack.

## app_program/selftest.h

`bool selftest_run(void);`

Check format boundaries and safety latching before task startup.

## app_program/subsystem1/mqtt_utk.h

`void mqtt_utk_start(void);`

Poll connection readiness from the existing WiFi service task.

## app_program/subsystem1/telemetry.h

`void telemetry_begin(telemetry_t * p_receiver, uint32_t total, bool b_command_topic);`

Only a single command byte may be accumulated. */
typedef struct
{
    uint32_t payload : 8;
    uint32_t received : 1;
    bool     b_bad : 1;
    bool     b_complete : 1;
} telemetry_t;

/** @brief Reject unexpected topics and every length other than one byte.

`bool telemetry_fragment(telemetry_t * p_receiver, const uint8_t * p_data, uint16_t length, bool b_last);`

Preserve binary bytes; reject truncation, overrun and extra fragments.

## app_program/subsystem2/encoder_test.h

`void encoder_test_init(encoder_test_t * p_test);`

Initialize the motor-off encoder diagnostic.

`void encoder_test_step(encoder_test_t * p_test);`

Print encoder snapshots while the operator rotates the wheels.

## app_program/subsystem2/motion.h

`void motion_init(motion_t * p_motion, uint32_t left, uint32_t right);`

Task-private packed state; never transmitted as a C object. */
typedef struct
{
    uint32_t last_left;
    uint32_t last_right;
    uint32_t position_um : 22;
    uint32_t mode : 3;
    bool     b_reverse : 1;
    bool     b_done : 1;
    bool     b_stall : 1;
    bool     b_encoder_fault : 1;
    bool     b_timeout : 1;
    uint32_t speed : 9;
    uint32_t target_mm : 12;
    uint32_t progress_um : 22;
    uint32_t elapsed_ms : 14;
    uint32_t stall_left_ms : 8;
    uint32_t stall_right_ms : 8;
    uint32_t integral_left : 15;
    uint32_t integral_right : 15;
    uint32_t prior_left : 11;
    uint32_t prior_right : 11;
    uint32_t remainder : 13;
    int16_t  pwm_left;
    int16_t  pwm_right;
    int16_t  straight_error_edges;
} motion_t;

/** @brief Reset while stopped, anchoring both encoder snapshots.

`bool motion_command(uint32_t * p_word, uint8_t mode, uint16_t distance, uint16_t speed, bool b_reverse);`

Build a 24-bit command with checked physical ranges.

`bool motion_forward(uint32_t * p_word, uint16_t distance, uint16_t speed);`

Build a forward, encoder-controlled straight move command.

`bool motion_backward(uint32_t * p_word, uint16_t distance, uint16_t speed);`

Build a reverse, encoder-controlled straight move command.

`bool motion_turn_left(uint32_t * p_word, uint16_t distance, uint16_t speed);`

Build a left-in-place turn command.

`bool motion_turn_right(uint32_t * p_word, uint16_t distance, uint16_t speed);`

Build a right-in-place turn command.

`bool motion_stop(uint32_t * p_word);`

Build the command that stops the controller immediately.

`bool motion_accept(motion_t * p_motion, uint32_t command);`

STOP overrides all states; manoeuvres preempt FOLLOW only.

`void motion_step(motion_t * p_motion, uint32_t left, uint32_t right, int16_t line_error, bool b_estop, int16_t * p_left_pwm, int16_t * p_right_pwm);`

Run one 10-ms cycle and produce bounded duty values -1000..1000.

`uint32_t motion_snapshot(const motion_t * p_motion, uint8_t tick, uint16_t speed, bool b_reverse);`

Publish one whole 32-bit snapshot, never bit-field writes.

`int32_t motion_edge_delta(uint32_t newer, uint32_t older, bool * pb_valid);`

Return signed modulo-2^32 edge displacement, with an edge guard.

## app_program/subsystem2/physical_test.h

`void physical_test_init(physical_test_t * p_test);`

Initialize the test in a motor-off state.

`void physical_test_step(physical_test_t * p_test, motion_t * p_motion, int16_t * p_left_pwm, int16_t * p_right_pwm);`

Run one 10-ms test cycle and return the requested PWM values.

## app_program/subsystem3/line.h

`void line_init(line_t * p_line);`

Packed state is private to the 5-ms sensing task. */
typedef struct
{
    uint8_t  samples[5];
    uint32_t state : 3;
    uint32_t error : 8;
    uint32_t barcode : 2;
    bool     b_barcode_valid : 1;
    bool     b_last_black : 1;
    bool     b_line_valid : 1;
    bool     b_lost : 1;
    uint32_t segment : 3;
    uint32_t symbol : 2;
    uint32_t transition_mm : 12;
    uint32_t lost_ms : 8;
    uint32_t frame_ms : 10;
    uint32_t calibration_samples : 6;
    uint32_t junction_ms : 9;
    uint32_t threshold_left : 12;
    uint32_t threshold_centre : 12;
    uint32_t threshold_right : 12;
} line_t;

/** @brief Load provisional calibrated thresholds from robot_config.h.

`bool line_calibrate(line_t * p_line, const uint16_t * p_white, const uint16_t * p_black);`

Derive thresholds from measured white and black references.

`void line_step(line_t * p_line, const uint16_t * p_samples, uint16_t distance, bool b_planned_departure);`

Process 12-bit ADC samples and modulo-4096 encoder distance.

`int16_t line_error(const line_t * p_line);`

Return steering -127..127, with validity stored separately.

## app_program/subsystem4/drv_lsm303.h

`bool drv_lsm303_init(drv_lsm303_t * p_imu);`

Which chip was found, and where. */
typedef struct
{
    uint32_t variant : 2;       /* DRV_LSM303_NONE, DLHC or D */
    uint32_t accel_address : 7; /* 7-bit I2C address */
    uint32_t mag_address : 7;
    /* Identity registers as read, for the boot print. Raw chip bytes. */
    uint8_t id[DRV_LSM303_ID_BYTES];
} drv_lsm303_t;

/** @brief Reset I2C1, identify the chip and write its configuration. Each configuration register is read back to prove the write landed. @return false if no supported chip answers or a write did not stick.

`bool drv_lsm303_accel(const drv_lsm303_t * p_imu, int16_t * p_raw, int16_t * p_mg);`

Read one acceleration sample: raw chip counts and milli-g. Both arrays hold chip X, Y, Z. Either pointer may be NULL.

`bool drv_lsm303_mag(const drv_lsm303_t * p_imu, int16_t * p_raw);`

Read raw magnetometer counts in chip X, Y, Z order.

`const char * drv_lsm303_name(const drv_lsm303_t * p_imu);`

Printable chip name for the boot banner.

## app_program/subsystem4/imu_test.h

`void imu_test_init(imu_test_t * p_test);`

Bring-up state; the encoders give distance when pushed by hand. */
typedef struct
{
    drv_lsm303_t imu;
    terrain_t    terrain;
    uint32_t     last_left; /* encoder counts, modulo 2^32 */
    uint32_t     last_right;
    int64_t      edges;    /* left + right edges since start */
    uint32_t     start_ms; /* kernel time at init */
    uint32_t     last_row_ms;
    uint32_t     last_retry_ms;
    uint32_t     last_timing_ms;
    uint32_t     last_status_ms;
    uint32_t     read_us : 16;     /* this cycle's I2C read time */
    uint32_t     step_us : 16;     /* this cycle's terrain_step time */
    uint32_t     max_read_us : 16; /* worst case since the last summary */
    uint32_t     max_step_us : 16;
    uint32_t     last_state : 3; /* terrain state last cycle, for events */
    bool         b_found : 1;    /* a supported chip answered */
    bool         b_reported : 1; /* chip found/lost already printed */
} imu_test_t;

/** @brief Stop the motors, scan I2C1, identify the chip, print the header.

`void imu_test_step(imu_test_t * p_test);`

One 10-ms cycle: sample, run terrain, print at 20 Hz.

## app_program/subsystem4/safety.h

`void safety_init(safety_t * p_safety);`

Each of the twelve fault causes occupies one shared bit. */
typedef struct
{
    uint32_t faults : 12;
    uint32_t active : 12;
    bool     b_latched : 1;
} safety_t;

/** @brief Initialize with motors inhibited until startup is complete.

`void safety_update(safety_t * p_safety, uint16_t active);`

Update current conditions and retain historical fatal causes.

`bool safety_reset(safety_t * p_safety);`

RESET succeeds only when every currently active fatal cause clears.

## app_program/subsystem4/terrain.h

`void terrain_init(terrain_t * p_terrain);`

One 10-ms sample, gathered by the caller from IMU and Motion. */
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

/** @brief Task-local terrain state. Not shared, so no lock is needed.  * Bounded states, counters and flags are packed into unsigned bit-fields. Signed working values (filters, height) use plain fixed-width integers: BARR-C forbids signed bit-fields, and offset-encoding them would add an add/subtract to every 10-ms step for no RAM saving worth having. /
typedef struct
{
    uint32_t state : 3;           /* TERRAIN_CALIBRATING..SENSOR_FAULT */
    uint32_t motion : 3;          /* TERRAIN_MOTION_* */
    uint32_t failures : 2;        /* consecutive I2C failures, 0..3 */
    uint32_t cal_samples : 6;     /* still samples collected, 0..32 */
    uint32_t stale_run : 4;       /* consecutive stale Motion samples, 0..15 */
    uint32_t confirm_mm : 6;      /* travel above 5 deg while arming, 0..63 */
    uint32_t last_distance : 12;  /* odometer at the last used sample */
    uint32_t phase_ms : 14;       /* time in the current phase, 0..10000 */
    uint32_t hump_ms : 14;        /* time since the hump started, 0..10000 */
    uint32_t gate_ms : 9;         /* landing-gate time left, 0..300 */
    uint32_t impact_run : 2;      /* samples above impact threshold, 0..3 */
    uint32_t pitch_field : 7;     /* telemetry pitch, 0.5 deg, offset 64 */
    uint32_t hump_peak_mm : 8;    /* peak of the last finished hump */
    uint32_t run_peak_mm : 8;     /* highest valid peak of the run */
    bool     b_calibrated : 1;    /* boot calibration has completed */
    bool     b_arming : 1;        /* nose above 2 deg, hump not confirmed */
    bool     b_wait_flat : 1;     /* hump ended: re-arm only after flat */
    bool     b_side : 1;          /* current hump is a one-wheel (lean) hump */
    bool     b_side_wait : 1;     /* hump ended: re-arm only after upright */
    bool     b_confirm_left : 1;  /* side of the lean being confirmed */
    uint32_t side_wheel : 2;      /* TERRAIN_WHEEL_*, of current/last hump */
    uint32_t side_confirm_mm : 6; /* travel leaning over the level, 0..63 */
    bool     b_on_hump : 1;       /* ON_HUMP: confirmed climb until LEVEL */
    bool     b_hump_valid : 1;    /* current/last hump is trustworthy */
    bool     b_peak_valid : 1;    /* run_peak_mm comes from a valid hump */
    bool     b_have_last : 1;     /* last_distance holds a real sample */
    bool     b_started : 1;       /* event this step: hump confirmed */
    bool     b_finished : 1;      /* event this step: hump ended */
    bool     b_impact : 1;        /* event this step: impact detected */
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
    int16_t  side_arm_peak_cmm;    /* highest height while confirming */
    int16_t  left_x6;              /* filtered left accel, mg x 6 */
    int16_t  turn_x6;              /* filtered right - left wheel, mm/s x 6 */
    int16_t  roll_x4;              /* second lean filter, sin x 1000 x 4 */
    int16_t  roll_milli;           /* lean vs flat, sin x 1000, + = left up */
    int32_t  baseline_left_x256;   /* flat-ground left reading, mg x 256 */
    int32_t  cal_left_sum;
    int32_t  cal_forward_sum; /* calibration sums, mg */
    int32_t  cal_up_sum;
} terrain_t;

/** @brief Start calibrating and ask the caller to initialise the IMU.

`void terrain_step(terrain_t * p_terrain, const terrain_input_t * p_input);`

Advance the terrain statechart by one 10-ms sample.

`bool terrain_ready(const terrain_t * p_terrain);`

True once Safety may stop waiting: calibrated or faulted.

`uint32_t terrain_pitch_field(int32_t pitch_cdeg);`

Encode pitch in 0.01 deg as 7 bits: 0.5 deg steps, offset 64. Saturates at -32.0 and +31.5 deg; never wraps.

`int32_t terrain_pitch_cdeg(uint32_t field);`

Decode a 7-bit pitch field back to 0.01 deg.

`uint32_t terrain_peak_mm(int32_t height_cmm);`

Round a height in 0.01 mm to whole mm, saturating at 0..255.

`uint16_t terrain_event_pack(bool b_finished, uint32_t peak_mm, bool b_valid);`

Pack a hump event: finished bit 0, peak bits 1-8, valid bit 9.

`bool terrain_event_unpack(uint16_t event, bool * pb_finished, uint32_t * p_peak_mm, bool * pb_valid);`

Unpack a hump event; false if any reserved bit is set.

`int32_t terrain_atan2_cdeg(int32_t y, int32_t x);`

Integer atan2 in 0.01 deg, error under 0.1 deg. @pre |y| and |x| are below 2^19.

`uint32_t terrain_isqrt(uint32_t value);`

Integer square root, rounded down.

## app_program/subsystem5/navigation.h

`void navigation_init(navigation_t * p_navigation);`

Task-private mission state uses eight values exactly. */
typedef struct
{
    uint32_t state : 3;
    uint32_t stage : 4;
    bool     b_waiting : 1;
    bool     b_scan_requested : 1;
    bool     b_reset_requested : 1;
    bool     b_stop_requested : 1;
    bool     b_command_ready : 1;
    bool     b_fault : 1;
    bool     b_left : 1;
    bool     b_slow : 1;
    uint32_t line_confirm : 2;
    bool     b_estop : 1;
    uint32_t barcode : 2;
    uint32_t elapsed_ms : 14;
    uint32_t look : 3;
    uint32_t left_cm : 9;
    uint32_t right_cm : 9;
    uint32_t side_mm : 12;
    uint32_t steps : 5;
    bool     b_margin : 1;
    uint32_t width_mm : 12;
    uint32_t detected_mm : 12;
    bool b_first_forward : 1;
    bool b_body_scan : 1;
    uint32_t body_index : 4;
    uint32_t side_remaining_mm : 12;
    uint16_t body_ranges[OBSTACLE_BODY_POINTS];
    uint8_t scan_failure;
    uint32_t command;
} navigation_t;

/** @brief Reach IDLE locally without waiting for WiFi.

`bool navigation_supervise(navigation_t * p_navigation, uint8_t command, bool b_safety_latched);`

Validate START/STOP/RESET/E-stop against the current state.

`void navigation_barcode(navigation_t * p_navigation, uint8_t barcode);`

Consume one barcode once, pre-empting line following.

`bool navigation_obstacle_plan(navigation_t * p_navigation, uint16_t width_mm, uint16_t left_cm, uint16_t right_cm);`

Accept a bounded width and side clearances for a pending bypass.

`bool navigation_body_scan(navigation_t * p_navigation);`

Request a centred-sensor body sweep for a pending bypass.

`void navigation_step(navigation_t * p_navigation, uint8_t motion_mode, bool b_done, bool b_line_valid, bool b_line_lost, bool b_on_hump, uint16_t range_cm, bool b_range_valid, bool b_scan_ready, bool b_scan_left, bool b_barcode_zone, bool b_safety_latched);`

Run one 10-ms arbitration cycle; output is one packed command.

## app_program/subsystem5/obstacle.h

`void obstacle_init(obstacle_t * p_obstacle);`

The servo angle is represented in three-degree units. */
typedef struct
{
    uint32_t state : 3;
    uint32_t angle : 6;
    uint32_t best_angle : 6;
    uint32_t best_cm : 9;
    uint32_t range_cm : 9;
    uint32_t coarse_index : 5;
    uint32_t retries : 2;
    bool     b_left : 1;
    bool     b_valid : 1;
    bool     b_ready : 1;
    uint32_t scan_ms : 13;
    uint16_t scan_cm[OBSTACLE_SCAN_POINTS];
    uint16_t width_mm;
    uint16_t left_cm;
    uint16_t right_cm;
    uint8_t reason;
} obstacle_t;

/** @brief Reset the recoverable abort state and face the sensor forward.

`bool obstacle_scan(obstacle_t * p_obstacle);`

Begin a stationary scan; reject overlapping requests.

`void obstacle_sample(obstacle_t * p_obstacle, uint16_t range_cm, bool b_valid);`

Consume a 3-degree sweep sample and estimate both obstacle edges.

`void obstacle_tick(obstacle_t * p_obstacle, uint16_t elapsed_ms);`

Enforce the configured whole-scan deadline.

`uint16_t obstacle_median(uint16_t first, uint16_t second, uint16_t third);`

Return an outlier-resistant median without adding readings.

`uint16_t obstacle_range(const uint16_t * p_ranges, uint8_t echoes);`

Combine zero to three echoes; no echo reports the maximum range.

`uint16_t obstacle_body_width(const uint16_t * p_ranges);`

Estimate width from body headings -90..90 in 15-degree steps. Returns zero when either edge or the front face cannot be established.

`const char * obstacle_reason_name(uint8_t reason);`

Describe the scan rejection or wider-scan request.

## testing/host/motor_test.h

`void motor_test_run(void);`

Validate the operator movement sequence without energizing hardware.

## testing/host/terrain_test.h

`void terrain_test_run(void);`

Run every terrain check; an assertion stops at the first failure.

## testing/host/verify_main.h

`int main(void);`

Run boundary, fault-injection and cross-subsystem scenarios.
