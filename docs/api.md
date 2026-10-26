# Application API

Generated from public header comments by
`python3 tools/generate_docs.py`.

## app_program/app_main.h

`int usermain(void);`

Retain the kernel-required legacy symbol name and signature.

## app_program/board_io.h

`bool board_io_init(int32_t h_echo_flag);`

Configure safe outputs and install T-Kernel-wrapped ISRs on core 0.

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

`bool board_io_imu_init(void);`

Initialize the 100-Hz, +/-4-g LSM303DLHC accelerometer.

`bool board_io_imu(int16_t * p_x, int16_t * p_y, int16_t * p_z);`

Read a coherent XYZ sample in milli-g through I2C1.

`void board_io_servo(uint8_t angle);`

Set a servo angle 0..60 in three-degree units.

`void board_io_ping(void);`

Trigger a 10-us pulse using TIMER alarm 3, without busy waiting.

`bool board_io_range(uint16_t * p_cm);`

Retrieve the ISR-captured echo; an absent echo is invalid.

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
    uint32_t remainder : 11;
    int16_t  pwm_left;
    int16_t  pwm_right;
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

Return signed modulo-2^32 edge displacement, with a 64-edge guard.

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

Fine accumulation retains small 100-Hz height increments. */
typedef struct
{
    uint32_t state : 3;
    uint32_t failures : 2;
    uint32_t samples : 6;
    bool     b_valid : 1;
    bool     b_on_hump : 1;
    bool     b_started : 1;
    bool     b_finished : 1;
    bool     b_impact : 1;
    bool     b_overflow : 1;
    uint32_t pitch : 7;
    uint32_t peak_mm : 8;
    uint32_t height_centi_mm : 16;
    uint32_t peak_centi_mm : 16;
    uint32_t baseline_mg : 10;
    uint32_t last_speed : 10;
    uint32_t last_distance : 12;
    uint32_t climb_ms : 12;
    uint32_t hump_ms : 14;
    uint32_t fractional : 5;
    uint32_t calibration_sum : 15;
} terrain_t;

/** @brief Reset calibration and invalidate height until stationary samples.

`void terrain_step(terrain_t * p_terrain, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t speed, uint16_t distance, bool b_fresh);`

Readings are signed milli-g; motion speed is signed mm/s. @param b_fresh False means I2C or motion data exceeded the 20-ms limit.

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
    uint32_t command;
} navigation_t;

/** @brief Reach IDLE locally without waiting for WiFi.

`bool navigation_supervise(navigation_t * p_navigation, uint8_t command, bool b_safety_latched);`

Validate START/STOP/RESET/E-stop against the current state.

`void navigation_barcode(navigation_t * p_navigation, uint8_t barcode);`

Consume one barcode once, pre-empting line following.

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
    uint32_t coarse_index : 3;
    uint32_t fine_index : 2;
    uint32_t retries : 2;
    bool     b_left : 1;
    bool     b_valid : 1;
    bool     b_ready : 1;
    uint32_t scan_ms : 13;
} obstacle_t;

/** @brief Reset the recoverable abort state and face the sensor forward.

`bool obstacle_scan(obstacle_t * p_obstacle);`

Begin a stationary scan; reject overlapping requests.

`void obstacle_sample(obstacle_t * p_obstacle, uint16_t range_cm, bool b_valid);`

Consume one median-of-three range after servo settling.

`void obstacle_tick(obstacle_t * p_obstacle, uint16_t elapsed_ms);`

Enforce a five-second whole-scan deadline.

`uint16_t obstacle_median(uint16_t first, uint16_t second, uint16_t third);`

Return an outlier-resistant median without adding readings.

## testing/host/motor_test.h

`void motor_test_run(void);`

Validate the operator movement sequence without energizing hardware.

## testing/host/verify_main.h

`int main(void);`

Run boundary, fault-injection and cross-subsystem scenarios.

## lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h

`void mqtt_utk_start(void);`

Poll connection readiness from the existing WiFi service task.
