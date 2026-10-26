# CSC20 Motor-Wheel

Line-following robot firmware for a Raspberry Pi Pico W / Robo Pico running
micro T-Kernel 3.0. The five subsystem folders replace the starter LED,
producer, consumer, and monitor examples.

The implementation follows the attached Week 6 design review structure,
Tribunal 1 corrections, lecturer feedback, minimum-bit change requirement, and
BARR-C:2018 for application code. Start with [the completed design review](docs/design_review.md).
The supplied PDFs and `CSC20.txt` remain source documents.

## Build and flash

Install the Arm embedded compiler, Make, a host C++ compiler, and the Pico SDK
with its `tinyusb`, `cyw43-driver`, and `lwip` submodules. Set `PICO_SDK_PATH` to
the SDK directory. No new third-party runtime library is required.

```sh
# USB console and radio, autonomous operation without MQTT:
make -C build_make -j8

# Autonomous operation with binary MQTT telemetry and supervisory commands:
# First create config/wifi_secrets.h from its example and set your own values.
make -C build_make -j8 NET=lwip MQTT=1

# Include motor-free format/safety checks before application startup:
make -C build_make -j8 NET=lwip MQTT=1 TEST=1

# Single-core qualification uses the same application and synchronization:
make -C build_make -j8 SMP=0 NET=lwip MQTT=1

# Opt-in low-speed physical motor sequence (no autonomous navigation):
make -C build_make -j8 MOTOR_TEST=1 WIFI=none NET=none

# Motor-off quadrature encoder diagnostic:
make -C build_make -j8 ENCODER_TEST=1 WIFI=none NET=none
```

For example, the MQTT build produces
`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.
Hold BOOTSEL while connecting USB and copy that UF2 onto RPI-RP2.

GP0/GP1 belong to ultrasonic ranging: use USB console and `CRASH_REPORT=0`.
The hardware driver rejects UART-console builds that would claim those pins.
WiFi connection failure does not prevent offline startup or local controls.

### Physical motor test

Build with `MOTOR_TEST=1` only for a bench test. The resulting image keeps the
motors off until the local START switch is pressed, then runs a
200-mm forward move, stop, 200-mm reverse move, stop, left turn, stop, right
turn, and stop. Each movement is encoder-controlled at 150 mm/s and stops
after the target encoder distance. It prints phase messages on the USB console
and stops on an encoder stall or the active-low E-stop on GP21. Keep the wheels
lifted for the first run, release START before boot, and keep the E-stop
accessible. RESET on GP22 returns the test to its armed, motor-off state.

This mode is intentionally separate from the normal autonomous firmware and
must not be used while the robot is on a course. A successful console sequence
ends with `PHYSICAL MOTOR TEST: PASS - all phases complete`.

For encoder diagnosis, build with `ENCODER_TEST=1`. This mode disables motor
outputs and reports cumulative left/right counts and deltas every 250 ms after
START. Rotate each wheel slowly by hand and look for changing counts with
`valid=1`.

## Wiring and operating assumptions

See [TEAM.md](TEAM.md) for every pin and peripheral owner. The implementation
uses the documented A/B encoders and gyro-free GY-511 / LSM303DLHC, as directed
in this session. Wheel circumference, 960 quadrature edges/revolution, motor
polarity, 140-mm track, IR thresholds, and servo limits are provisional values
in [robot_config.h](app_program/robot_config.h) that require bench calibration.

The course barcode format was not supplied. A complete provisional format and
four-symbol decoder are specified in [barcode_protocol.md](docs/barcode_protocol.md).
Place GP28 over the separate barcode lane; GP26/27 track the line.

Local active-low switches connect GP20 START, GP21 E-stop, and GP22 RESET to
GND. Boot holds motor drive off until stationary IMU calibration and line-sensor
startup finish. START is separate from RESET; resetting never resumes movement.
An E-stop latches until its condition clears and RESET completes.

## Host checks and evidence

```sh
python3 tools/run_tests.py --output testing/evidence/host_tests.txt
python3 tools/measure_memory.py
python3 tools/measure_messages.py
python3 tools/check_barr.py

# Use the formatter before building after editing C:
python3 tools/format_c.py
# Or: CLANG_FORMAT=/path/to/clang-format python3 tools/format_c.py
```

The C logic is compiled as ISO C99 with strict warnings and address/undefined
behavior sanitizers. `TEST=1` adds startup checks on the device; it does not
substitute for the physical qualification tests in [validation.md](docs/validation.md).

The project build runs the automated BARR-C subset check. Formatting uses
`.clang-format` plus the BARR-specific definition-spacing and switch-break
rules. See [coding_standard.md](docs/coding_standard.md) for the review scope.

## Binary MQTT

All MQTT calls run inside the existing core-0 WiFi service. Application tasks
use bounded nonblocking queues; they never call lwIP or CYW43 directly.

| Topic | Payload |
|---|---|
| `robot/telemetry` | 28 bytes; 223 meaningful bits; 4 Hz |
| `robot/status` | 1 byte; version2, WiFi1, MQTT1 |
| `robot/command` | 1 byte; command2, sequence4, reserved2=0 |
| `robot/ack` | 1 byte; echoed command2/sequence4, accepted1, malformed1 |

```sh
python3 tools/robot_protocol.py command stop 0
python3 tools/robot_protocol.py command start 1
python3 tools/robot_protocol.py decode e9ffe9df7f24000000ff9fb615c8f201a8ffffffff1700000060807e
```

Send commands without MQTT retain. Each new MQTT connection requires a fresh
STOP or RESET before remote START, so a retained START cannot start the robot
on subscription. Local START remains available independently. Sequences advance
modulo 16 by 1..7; queued commands older than 500 ms are rejected. Detailed
layouts, overflow rules, and synchronization appear in
[data_representation.md](docs/data_representation.md).

The code and host checks are complete. Physical accuracy, control-loop jitter,
end-to-end stopping latency, and broker/radio recovery still need the robot,
track, and instruments; their status is recorded explicitly in the evidence.
