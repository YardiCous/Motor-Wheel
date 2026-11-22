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
# USB and radio, autonomous operation without MQTT:
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

# Real line tracking with obstacle scan, bypass, and line recovery:
make -C build_make -j8 OBSTACLE_TEST=1 WIFI=none NET=none
```

The two standard builds produce:

- Default: `build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`
- MQTT dashboard: `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`

Both include the startup and task-priority fixes. Every build checks that
robot tasks outrank USB and WiFi services. Hold BOOTSEL while connecting USB
and copy the selected UF2 onto RPI-RP2.

The default build uses the USB CDC console. Console diagnostics are available
for boot/watchdog, range, physical-test, encoder-test and bench-test messages
that enable `tm_printf()`. MQTT telemetry remains available without console
output and continues to operate when the USB cable is disconnected.
Set `CRASH_REPORT=1` to mirror console and panic output to UART0 on GP0/GP1;
UART mirroring conflicts with the ultrasonic-ranging pins.

GP0/GP1 belong to ultrasonic ranging: use USB console and `CRASH_REPORT=0`.
The hardware driver rejects UART-console builds that would claim those pins.
WiFi connection failure does not prevent offline startup or local controls.

### Physical motor test

The command above produces
`build_make/mtk3pico_smp1_usb_cdc_motortest.uf2`.

Build with `MOTOR_TEST=1` only for a bench test. The resulting image keeps the
motors off until the local START switch is pressed, then runs a
600-mm forward move, stop, 600-mm reverse move, stop, left turn, stop, right
turn, and stop. Each movement is encoder-controlled at 70 mm/s and stops
after the target encoder distance. It prints phase messages on the USB console
and stops on an encoder stall or the active-low E-stop on GP21. Keep the wheels
lifted for the first run, release START before boot, and keep the E-stop
accessible. RESET on GP22 returns the test to its armed, motor-off state.

The left and right turns target 90 degrees using the configured
`ROBOT_QUARTER_TURN_MM` value, currently 92 mm. Straight moves balance
cumulative encoder travel with a small leftward trim: right wheel +0.5%,
left wheel -0.5%, to compensate for the remaining right drift. Confirm the
angle and straight tracking on the floor after flashing; slip and unequal
wheel diameters still require physical calibration.

This mode is intentionally separate from the normal autonomous firmware and
must not be used while the robot is on a course. A successful console sequence
ends with `PHYSICAL MOTOR TEST: PASS - all phases complete`.

For encoder diagnosis, build with `ENCODER_TEST=1`. This mode disables motor
outputs and reports cumulative left/right counts and deltas every 250 ms after
START. Rotate each wheel slowly by hand and look for changing counts with
`valid=1`.

The obstacle test uses the real GP26/GP27 line sensors, ultrasonic ranging,
and encoder-controlled turns; only the IMU is stubbed. After a bypass it
searches for the line and resumes tracking. A valid forward reading at 30 cm
or nearer starts avoidance. A batch with no echoes reports 400 cm (out of
range), without latching a fault or blocking START/RESET. This sensor result
cannot distinguish open space from a disconnected sensor; check the sensor's
response with a box ahead before a floor run. Range samples are not printed
to the USB serial monitor.
Failed pings also print `ECHO` evidence before capture is cleared: kernel wait
result, capture state (1 waiting for rise, 2 waiting for fall, 3 complete),
pulse width in microseconds, cumulative rising/falling edge counts, GP0/GP1
input levels, GP1 mux/pad setup, and echo interrupt status. These distinguish
no incoming signal from incomplete pulses, invalid widths, or wait failures.
A completed pulse below the sensor's 2-cm measuring range reports `1 cm`
as a near-limit indication and triggers the existing close-obstacle stop.
It is not discarded into the 400-cm no-echo fallback.
The stopped scan covers -30 to +30 degrees in 3-degree steps. It estimates
width from the first clear ray beyond each edge. When only one edge is visible,
telemetry reports twice that edge's offset. With neither edge visible, width
is unavailable. The scan prefers a measured edge with
clearance and otherwise picks the more open side. Width assumes a roughly
flat obstacle face and is affected by sensor position and beam spread.

The selected route turns 90 degrees, moves sideways to clear the chosen edge,
then turns back 90 degrees. The sideways target includes the robot's turning
radius and a 50-mm margin. The initial forward target is the saved detection
distance plus the robot's length. The robot checks clearance after turning;
blocked legs stop. It then checks obstacle depth with bounded extra forward
moves before returning to the line. The plan records the width, sideways
offset, saved forward distance, and selected side.
After each forward leg it turns 90 degrees toward the obstacle and checks.
If still blocked, it turns 90 degrees back, waits for a fresh forward reading,
then advances another 100 mm in obstacle-test mode (50 mm/s times 2000 ms).
This repeats up to the configured retry limit. A clear side check starts a
150-mm forward margin (`ROBOT_WIDTH_MM / 2 + 50`), followed by another check before returning to the line;
a blocked margin recheck resumes the retry loop.
The servo scan has an 8-second deadline. Each bypass turn and leg has a
10-second limit. Large sideways offsets are split into checked moves of at
most eight seconds each.

RESET reinitialises the ranging pins and echo capture. The trigger pulse is
timed directly for 12 microseconds, so it does not depend on a timer ISR.

The startup clock code explicitly configures the watchdog tick divider for
a 1-MHz timer reference on every boot. USB BOOTSEL sets that divider while
flashing, but normal flash boot does not. Inheriting its reset value made
timer-based trigger pulses, echo widths and network delays depend on how the
firmware was started. The firmware now owns that setup after selecting the
12-MHz crystal reference. This starts the tick generator, not a watchdog reset
countdown. `ECHO` diagnostics report `tick=0x20c` for the expected setup.

## Wiring and operating assumptions

See [TEAM.md](TEAM.md) for every pin and peripheral owner. The implementation
uses the documented A/B encoders and gyro-free GY-511 / LSM303DLHC, as directed
in this session. Wheel circumference, 2400 quadrature edges/revolution, motor
polarity, 112-mm track, IR thresholds, and servo limits are provisional values
in [robot_config.h](app_program/robot_config.h) that require bench calibration.

The course barcode format was not supplied. The firmware currently decodes
Code 39 letters from GP28 and applies the A..Z junction table documented in
[barcode_protocol.md](docs/barcode_protocol.md). Place GP28 over the separate
barcode lane; GP26/27 track the line.

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

### Live dashboard

Install the MQTT dependency once, then run the Python launcher. It opens a
local browser dashboard, automatically subscribes to the firmware topics,
and decodes live telemetry using the existing protocol decoder. Start and Stop
buttons send supervisory commands and show the robot's acknowledgement. Press
Stop and wait for confirmation before Start becomes available.

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r tools/dashboard/requirements-dashboard.txt
python tools/dashboard/telemetry_dashboard.py

# Preview simulated readings without a robot or extra dependencies:
python3 tools/dashboard/telemetry_dashboard.py --demo
```

The broker address is read from `config/wifi_secrets.h`; override it with
`--broker 192.168.1.10`. See [dashboard setup and custom Python decoders](docs/telemetry_dashboard.md)
for virtual environment setup, connection options and adding readings without
editing the frontend.

### Wire protocol

MQTT topic names are declared in `app_program/subsystem1/telemetry.h`.
After changing them, rebuild and flash the firmware and update subscribers.

All MQTT calls run inside the existing core-0 WiFi service. Application tasks
use bounded nonblocking queues; they never call lwIP or CYW43 directly.

| Topic | Payload |
|---|---|
| `robot/telemetry` | 31 bytes, 241 meaningful bits, 4 Hz |
| `robot/status` | 1 byte; WiFi at bit 0, MQTT at bit 1; upper six bits zero |
| `robot/command` | 1 byte; command2, sequence4, reserved2=0 |
| `robot/ack` | 1 byte; echoed command2/sequence4, accepted1, malformed1 |

Payloads have no version field. Telemetry starts with the three-bit vehicle
state. Bits 221 and 222 carry the IMU's "picked up" and hump line guard flags.
Byte 29 carries the last scanned Code 39 letter (0 before a scan, 1–26 for
A–Z). Bits 116–118 carry its junction action and validity. Bits 232–243 carry
the estimated obstacle width in millimetres, with 0 indicating unavailable.
Bit 244 identifies estimates based on one edge.
The dashboard also accepts earlier 28- and 29-byte packets. Flash the updated
firmware and restart the dashboard to see the width estimate.

```sh
python3 tools/dashboard/robot_protocol.py command stop 0
python3 tools/dashboard/robot_protocol.py command start 1
python3 tools/dashboard/robot_protocol.py decode fa7ffaf71f090000c0ffa76d05b27c00eaffffffff0500000018a01f
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
