# Application data representation

All transport fields use explicit little-endian masks/shifts. Internal
C bit-fields are task-private or protected under the core-0 IRQ lock;
their compiler-specific ordering is never a network or queue ABI.

Widths below are information widths. Byte boundaries, compiler structure
padding and naturally aligned native cells can leave unused physical bits;
those are not extra application fields. Reserved wire bits are zero.

Arithmetic temporaries keep sufficient native precision before encoding.
Pointers, RTOS IDs, stack pointers, hardware register images and kernel
timestamps retain their platform contracts; they are not narrowed as if
they were bounded robot states. The table's full-width counter/time fields
have the entire unsigned 32-bit modulo range as their valid domain.

## Telemetry wire layout

| Field | Bit offset | Bits | Meaning / valid range |
|---|---:|---:|---|
| vehicle | 0 | 3 | 0..7 state |
| distance_mm | 3 | 12 | 0..4095 mm; wraps; signed delta requires <2048 mm |
| speed_mm_second | 15 | 9 | 0..511; command target is at most 500; sign is separate |
| reverse | 24 | 1 | 0/1, actual centre-speed sign |
| motion_mode | 25 | 3 | 0..6 |
| tick_5ms | 28 | 7 | 0..127, wraps at 640 ms |
| faults | 35 | 12 | 12 independent bits, 0..4095 |
| ir_left | 47 | 12 | 0..4095 ADC counts |
| ir_centre | 59 | 12 | 0..4095 ADC counts |
| ir_right | 71 | 12 | 0..4095 ADC counts |
| terrain | 83 | 3 | 0..6 |
| pitch_offset | 86 | 7 | 0..127 = -32..31.5 degrees |
| peak_mm | 93 | 8 | 0..255 mm; overflow sets RANGE and clears validity |
| range_cm | 101 | 9 | 0 unavailable; 1 below 2 cm; 2..400 cm (400 also no echo) |
| servo_3deg | 110 | 6 | 0..60 = 0..180 degrees; selected profile angle in PLAN, otherwise commanded servo angle |
| barcode_event | 116 | 3 | code2 + known1; last valid barcode, not a queue pulse |
| line_error_offset | 119 | 8 | 0..254 = -127..127; 255 unused |
| line_state | 127 | 3 | 0..5 |
| obstacle_state | 130 | 3 | 0..7 (7 body scan) |
| link | 133 | 2 | WiFi1 | MQTT1 |
| on_hump | 135 | 1 | 0/1, whole-hump status |
| height_valid | 136 | 1 | 0/1; never infer validity from a zero height |
| encoder_left | 137 | 32 | 0..4294967295, unsigned wrapping encoder count |
| encoder_right | 169 | 32 | 0..4294967295, unsigned wrapping encoder count |
| wheel_left_offset | 201 | 10 | 0..1023 = -512..511 mm/s, bias 512 |
| wheel_right_offset | 211 | 10 | 0..1023 = -512..511 mm/s, bias 512 |
| Reserved padding | 221 | 3 | Must be zero; total 28 bytes |

Motion snapshot offset 3 occupies exactly 32 bits: 12+9+1+3+7.
Its full-width heartbeat age must be ≤20 ms before terrain uses it;
the seven-bit stamp alone cannot detect long-stale data after wrap.

## Stored controller and ISR fields

For each encoded nonnegative range 0..maximum, required bits are
`ceil(log2(maximum + 1))`. Offset binary is used for signed quantities.
One-bit logical fields are C99 `bool` bit-fields. Saturation/fault rules
and the units for biased fields are described below and in the headers.

| Module / type | Field | Valid encoded range | Minimum bits | Representation |
|---|---|---|---:|---|
| board_io / capture_t | `echo_width` | 0..32767 | 15 | unsigned bit-field |
| board_io / capture_t | `echo_state` | 0..3 | 2 | unsigned bit-field |
| board_io / capture_t | `last_left` | 0..3 | 2 | unsigned bit-field |
| board_io / capture_t | `last_right` | 0..3 | 2 | unsigned bit-field |
| board_io / capture_t | `b_left_encoder_fault` | 0..1 | 1 | bool bit-field |
| board_io / capture_t | `b_right_encoder_fault` | 0..1 | 1 | bool bit-field |
| board_io / capture_t | `left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / capture_t | `right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / capture_t | `rise_us` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / capture_t | `trigger_us` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / capture_t | `echo_rises` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / capture_t | `echo_falls` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `width_us` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `rise_after_us` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `tick_config` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `rises` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `falls` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `gpio_control` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `gpio_pad` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `irq_raw` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| board_io / board_io_range_diag_t | `irq_enabled` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| robot_os / shared_t | `b_latched` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_stop` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_reset` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_reset_motion` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_reset_imu` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_reset_scan` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_scan_request` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_motion_done` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_imu_ready` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_line_ready` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `sequence` | 0..15 | 4 | unsigned bit-field |
| robot_os / shared_t | `b_sequence_valid` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `queue_faults` | 0..15 | 4 | unsigned bit-field |
| robot_os / shared_t | `b_range_fault` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_remote_armed` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_network_online` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_line_visible` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `b_line_lost` | 0..1 | 1 | bool bit-field |
| robot_os / shared_t | `reset_at_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| robot_os / cyclic_t | `phase` | 0..99 | 7 | unsigned bit-field |
| robot_os / buttons_t | `stable_count` | 0..3 | 2 | unsigned bit-field |
| robot_os / buttons_t | `candidate` | 0..7 | 3 | unsigned bit-field |
| robot_os / buttons_t | `previous` | 0..7 | 3 | unsigned bit-field |
| robot_os / watchdog_t | `ready` | 0..3 | 2 | unsigned bit-field |
| robot_os / watchdog_t | `b_captured` | 0..1 | 1 | bool bit-field |
| robot_os / watchdog_t | `b_pending` | 0..1 | 1 | bool bit-field |
| robot_os / watchdog_t | `at_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| robot_os / bench_t | `report_at_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| robot_os / bench_t | `start_left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| robot_os / bench_t | `start_right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| robot_os / bench_t | `start_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| robot_os / bench_t | `settle_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| mqtt_utk / mqtt_utk_state_t | `state` | 0..4 | 3 | unsigned bit-field |
| mqtt_utk / mqtt_utk_state_t | `retry_exponent` | 0..5 | 3 | unsigned bit-field |
| mqtt_utk / mqtt_utk_state_t | `b_timer_started` | 0..1 | 1 | bool bit-field |
| mqtt_utk / mqtt_utk_state_t | `retry_at` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| telemetry / telemetry_t | `payload` | 0..255 | 8 | unsigned bit-field |
| telemetry / telemetry_t | `received` | 0..1 | 1 | unsigned bit-field |
| telemetry / telemetry_t | `b_bad` | 0..1 | 1 | bool bit-field |
| telemetry / telemetry_t | `b_complete` | 0..1 | 1 | bool bit-field |
| encoder_test / encoder_test_t | `b_running` | 0..1 | 1 | bool bit-field |
| encoder_test / encoder_test_t | `last_report_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| encoder_test / encoder_test_t | `last_left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| encoder_test / encoder_test_t | `last_right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| motion / motion_t | `position_um` | 0..4095999 | 22 | unsigned bit-field |
| motion / motion_t | `mode` | 0..6 | 3 | unsigned bit-field |
| motion / motion_t | `b_reverse` | 0..1 | 1 | bool bit-field |
| motion / motion_t | `b_done` | 0..1 | 1 | bool bit-field |
| motion / motion_t | `b_stall` | 0..1 | 1 | bool bit-field |
| motion / motion_t | `b_encoder_fault` | 0..1 | 1 | bool bit-field |
| motion / motion_t | `b_timeout` | 0..1 | 1 | bool bit-field |
| motion / motion_t | `speed` | 0..500 | 9 | unsigned bit-field |
| motion / motion_t | `target_mm` | 0..4095 | 12 | unsigned bit-field |
| motion / motion_t | `progress_um` | 0..4095999 | 22 | unsigned bit-field |
| motion / motion_t | `elapsed_ms` | 0..10000 | 14 | unsigned bit-field |
| motion / motion_t | `stall_left_ms` | 0..200 | 8 | unsigned bit-field |
| motion / motion_t | `stall_right_ms` | 0..200 | 8 | unsigned bit-field |
| motion / motion_t | `integral_left` | 0..32000 | 15 | unsigned bit-field |
| motion / motion_t | `integral_right` | 0..32000 | 15 | unsigned bit-field |
| motion / motion_t | `prior_left` | 0..2000 | 11 | unsigned bit-field |
| motion / motion_t | `prior_right` | 0..2000 | 11 | unsigned bit-field |
| motion / motion_t | `remainder` | 0..4798 | 13 | unsigned bit-field |
| motion / motion_t | `last_left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| motion / motion_t | `last_right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| physical_test / physical_test_t | `pause_until_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| physical_test / physical_test_t | `start_left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| physical_test / physical_test_t | `start_right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| physical_test / physical_test_t | `start_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `edge_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `report_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `last_left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `last_right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `scans` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `moved_edges` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `moving_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| barcode_test / barcode_test_t | `ignored` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| line / line_t | `state` | 0..5 | 3 | unsigned bit-field |
| line / line_t | `error` | 0..254 | 8 | unsigned bit-field |
| line / line_t | `barcode` | 0..3 | 2 | unsigned bit-field |
| line / line_t | `b_barcode_valid` | 0..1 | 1 | bool bit-field |
| line / line_t | `b_last_black` | 0..1 | 1 | bool bit-field |
| line / line_t | `b_line_valid` | 0..1 | 1 | bool bit-field |
| line / line_t | `b_lost` | 0..1 | 1 | bool bit-field |
| line / line_t | `segment` | 0..6 | 3 | unsigned bit-field |
| line / line_t | `symbol` | 0..3 | 2 | unsigned bit-field |
| line / line_t | `transition_mm` | 0..4095 | 12 | unsigned bit-field |
| line / line_t | `lost_ms` | 0..150 | 8 | unsigned bit-field |
| line / line_t | `frame_ms` | 0..1000 | 10 | unsigned bit-field |
| line / line_t | `calibration_samples` | 0..32 | 6 | unsigned bit-field |
| line / line_t | `junction_ms` | 0..300 | 9 | unsigned bit-field |
| line / line_t | `threshold_left` | 0..4095 | 12 | unsigned bit-field |
| line / line_t | `threshold_centre` | 0..4095 | 12 | unsigned bit-field |
| line / line_t | `threshold_right` | 0..4095 | 12 | unsigned bit-field |
| scanner / scanner_t | `last_left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| scanner / scanner_t | `last_right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| scanner / scanner_t | `element_edges` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| drv_lsm303 / drv_lsm303_t | `variant` | 0..3 | 2 | unsigned bit-field |
| drv_lsm303 / drv_lsm303_t | `accel_address` | 0..127 | 7 | unsigned bit-field |
| drv_lsm303 / drv_lsm303_t | `mag_address` | 0..127 | 7 | unsigned bit-field |
| imu_test / imu_test_t | `read_us` | 0..65535 | 16 | unsigned bit-field |
| imu_test / imu_test_t | `step_us` | 0..65535 | 16 | unsigned bit-field |
| imu_test / imu_test_t | `max_read_us` | 0..65535 | 16 | unsigned bit-field |
| imu_test / imu_test_t | `max_step_us` | 0..65535 | 16 | unsigned bit-field |
| imu_test / imu_test_t | `last_state` | 0..7 | 3 | unsigned bit-field |
| imu_test / imu_test_t | `b_found` | 0..1 | 1 | bool bit-field |
| imu_test / imu_test_t | `b_reported` | 0..1 | 1 | bool bit-field |
| imu_test / imu_test_t | `last_left` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| imu_test / imu_test_t | `last_right` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| imu_test / imu_test_t | `start_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| imu_test / imu_test_t | `last_row_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| imu_test / imu_test_t | `last_retry_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| imu_test / imu_test_t | `last_timing_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| imu_test / imu_test_t | `last_status_ms` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| safety / safety_t | `faults` | 0..4095 | 12 | unsigned bit-field |
| safety / safety_t | `active` | 0..4095 | 12 | unsigned bit-field |
| safety / safety_t | `b_latched` | 0..1 | 1 | bool bit-field |
| terrain / terrain_input_t | `distance_mm` | 0..4095 | 12 | unsigned bit-field |
| terrain / terrain_input_t | `b_imu_ok` | 0..1 | 1 | bool bit-field |
| terrain / terrain_input_t | `b_motion_fresh` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `state` | 0..6 | 3 | unsigned bit-field |
| terrain / terrain_t | `motion` | 0..7 | 3 | unsigned bit-field |
| terrain / terrain_t | `failures` | 0..3 | 2 | unsigned bit-field |
| terrain / terrain_t | `cal_samples` | 0..63 | 6 | unsigned bit-field |
| terrain / terrain_t | `stale_run` | 0..15 | 4 | unsigned bit-field |
| terrain / terrain_t | `confirm_mm` | 0..63 | 6 | unsigned bit-field |
| terrain / terrain_t | `last_distance` | 0..4095 | 12 | unsigned bit-field |
| terrain / terrain_t | `phase_ms` | 0..16383 | 14 | unsigned bit-field |
| terrain / terrain_t | `hump_ms` | 0..10000 | 14 | unsigned bit-field |
| terrain / terrain_t | `gate_ms` | 0..511 | 9 | unsigned bit-field |
| terrain / terrain_t | `impact_run` | 0..3 | 2 | unsigned bit-field |
| terrain / terrain_t | `pitch_field` | 0..127 | 7 | unsigned bit-field |
| terrain / terrain_t | `hump_peak_mm` | 0..255 | 8 | unsigned bit-field |
| terrain / terrain_t | `run_peak_mm` | 0..255 | 8 | unsigned bit-field |
| terrain / terrain_t | `b_calibrated` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_arming` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_wait_flat` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_side` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_side_wait` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_confirm_left` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `side_wheel` | 0..3 | 2 | unsigned bit-field |
| terrain / terrain_t | `side_confirm_mm` | 0..63 | 6 | unsigned bit-field |
| terrain / terrain_t | `b_on_hump` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_hump_valid` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_peak_valid` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_have_last` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_started` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_finished` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_impact` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_motion_changed` | 0..1 | 1 | bool bit-field |
| terrain / terrain_t | `b_request_init` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `state` | 0..7 | 3 | unsigned bit-field |
| navigation / navigation_t | `stage` | 0..11 | 4 | unsigned bit-field |
| navigation / navigation_t | `b_waiting` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_scan_requested` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_reset_requested` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_stop_requested` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_command_ready` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_fault` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_left` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_slow` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `line_confirm` | 0..3 | 2 | unsigned bit-field |
| navigation / navigation_t | `b_estop` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `barcode` | 0..3 | 2 | unsigned bit-field |
| navigation / navigation_t | `elapsed_ms` | 0..10000 | 14 | unsigned bit-field |
| navigation / navigation_t | `look` | 0..7 | 3 | unsigned bit-field |
| navigation / navigation_t | `left_cm` | 0..511 | 9 | unsigned bit-field |
| navigation / navigation_t | `right_cm` | 0..511 | 9 | unsigned bit-field |
| navigation / navigation_t | `side_mm` | 0..4095 | 12 | unsigned bit-field |
| navigation / navigation_t | `steps` | 0..31 | 5 | unsigned bit-field |
| navigation / navigation_t | `b_margin` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `width_mm` | 0..4095 | 12 | unsigned bit-field |
| navigation / navigation_t | `detected_mm` | 0..4095 | 12 | unsigned bit-field |
| navigation / navigation_t | `b_first_forward` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_line_seen` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `b_body_scan` | 0..1 | 1 | bool bit-field |
| navigation / navigation_t | `body_index` | 0..15 | 4 | unsigned bit-field |
| navigation / navigation_t | `side_remaining_mm` | 0..4095 | 12 | unsigned bit-field |
| navigation / navigation_t | `command` | 0..16777215 | 24 | explicit kind2 + distance12 + speed9 + reverse1; high8 zero |
| obstacle / obstacle_t | `state` | 0..7 | 3 | unsigned bit-field |
| obstacle / obstacle_t | `angle` | 0..60 | 6 | unsigned bit-field |
| obstacle / obstacle_t | `best_angle` | 0..60 | 6 | unsigned bit-field |
| obstacle / obstacle_t | `best_cm` | 0..400 | 9 | unsigned bit-field |
| obstacle / obstacle_t | `range_cm` | 0..400 | 9 | unsigned bit-field |
| obstacle / obstacle_t | `coarse_index` | 0..21 | 5 | unsigned bit-field |
| obstacle / obstacle_t | `retries` | 0..3 | 2 | unsigned bit-field |
| obstacle / obstacle_t | `b_left` | 0..1 | 1 | bool bit-field |
| obstacle / obstacle_t | `b_valid` | 0..1 | 1 | bool bit-field |
| obstacle / obstacle_t | `b_ready` | 0..1 | 1 | bool bit-field |
| obstacle / obstacle_t | `scan_ms` | 0..8000 | 13 | unsigned bit-field |
| terrain_test / sim_t | `seed` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| terrain_test / sim_t | `starts` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| terrain_test / sim_t | `side_starts` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| terrain_test / sim_t | `finishes` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| terrain_test / sim_t | `impacts` | 0..4294967295 | 32 | 32-bit modulo counter/time |
| terrain_test / sim_t | `last_peak_mm` | 0..4294967295 | 32 | 32-bit modulo counter/time |

## Other shared fields and queue representations

| Data | Valid range / meaning | Required bits | Backing storage / synchronization |
|---|---|---:|---|
| ADC triplet | Three values 0..4095 | 36 | Five bytes; four unused bits zero |
| Fault source array | Six independently owned twelve-bit masks | 72 | Nine bytes; all RMW under saved IRQ mask |
| Task heartbeat / command-arrival / retry deadline / reset time | Full unsigned modulo-2^32 ms | 32 each | Native 32-bit fields; core-0 protected snapshots |
| Encoder pair | Each full unsigned modulo-2^32 | 64 | ISR writes; task copies both under IRQ mask |
| Motion command | kind0..3, distance0..4095 mm, speed0..500, reverse0/1 | 24 | Three-byte queue message |
| Barcode queue event | LEFT/RIGHT/STRAIGHT/U-TURN | 2 | One byte; high six bits zero |
| Hump queue event | finished0/1, peak0..255 mm, valid0/1 | 10 | Two bytes; high six bits zero |
| Supervisory queue entry | command2 + sequence4 + arrival32 | 38 | Five bytes; two unused bits zero; age≤500 ms |
| Telemetry queue entry | application221 + arrival32 | 253 | 32 bytes; bits 221..223 zero, arrival at bit 224; latest only |
| ACK | echoed command2 + sequence4 + accepted1 | 7 | One byte; bit7 used by transport for malformed messages |
| Status | WiFi at bit0 + MQTT at bit1 | 2 | One byte, upper six bits zero |

## Units, validity and overflow

* Encoder deltas use unsigned subtraction in either direction; more than
  64 edges per wheel/10-ms sample is an encoder/range guard fault.
* Stored steering uses bias 127. Wheel-speed reports use bias 512.
  Measured magnitude uses the full nine-bit 0..511 range to preserve
  encoder quantization around a 500-mm/s target. Wheel reports use
  -512..511; measurements outside that range set RANGE before saturation.
  Signed PID integrals use bias 16000 and previous errors bias 1000.
* Fine height/peak are 0..65535 in 0.01-mm units. A signed fractional
  carry -9..9 is stored as 0..18 with bias 9, in 0.001-mm units.
  A >255-mm report sets RANGE; saturation never retains valid height.
* Terrain baseline stores milli-g +500. Last terrain speed stores
  signed mm/s +512. IMU failures saturate at three; retries and all
  state timers saturate or transition before their bit-field limit.
* Distance state uses 0..4095999 micrometres (22 bits). The division
  remainder -2399..2399 is stored with bias 2399 (13 bits).
* Queue overflow never blocks control. Essential command/event overflow
  sets a fatal bit; routine telemetry replaces the oldest entry.
  ACK overflow drops the ACK only. Fatal causes remain in Safety's mask.
* Every packed shared RMW is IRQ-protected and every caller is core 0.
  ISR-owned encoder/echo state uses coherent masked snapshots; volatile
  prevents compiler elision but does not provide synchronization by itself.

See [message_comparison.md](message_comparison.md) and
[memory_comparison.md](memory_comparison.md) for measured comparisons.
