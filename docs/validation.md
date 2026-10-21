# Validation status

## Completed in this workspace

- Cross-configured and linked only for Pico W/RP2040 with Pico SDK 2.3.1, Arm GCC 14.2.1 and the course-linked native micro-T kernel.
- Built a default disarmed image and a separate networking-enabled image using example credentials and on-target tests.
- Checked UF2 block headers, RP2040 family ID, flash addresses and unique block numbering.
- Checked native kernel symbols in the ELF; the networking build also contains SDK CYW43 and MQTT symbols. No FreeRTOS task/scheduler symbols are linked.
- Compiled application code with `-Wall -Wextra -Werror`; checked formatting with clang-format 20.1.8.
- Embedded compile-time message-size assertions and optional on-board tests; no desktop firmware test executable is built.

These are build/artifact checks, **not physical robot test results**. No board was flashed or exercised here. The custom combination of upstream kernel and SDK startup requires on-board scheduling/interrupt validation.

Final artifact measurements (`arm-none-eabi-size`, bytes):

| Image | text | data | bss | UF2 blocks |
|---|---:|---:|---:|---:|
| Default, networking unconfigured | 61,192 | 0 | 67,404 | 224 |
| Networking configured with example values, embedded tests enabled | 372,020 | 0 | 126,752 | 1,438 |

These are different build configurations, not a before/after packing comparison. BSS includes the kernel allocation arena; runtime stack usage still requires measurement. The non-Pico-W configuration rejection was also checked.

## Embedded tests

Configure `ROBOT_ON_TARGET_TESTS=ON`. Before starting robot tasks, the firmware checks:

- All command opcode/sequence combinations and signed 14-bit motion arguments.
- Rejection of invalid ranges, cross-byte packing, telemetry bounds and reserved bits.
- Fault latching, rejected RESET with active causes, explicit reset after clearing, advisory Wi-Fi loss and STOP.
- Timestamp wrap, stale line/range data and the bounded range exemption during stopped scans.
- Line loss/search and barcode suppression deadlines.
- Forward/reverse quadrature, invalid transitions and count wrap.
- Native micro-T message copying, event bit clearing, worker task startup and elapsed kernel delays.

These tests compile into the Pico W test image. Their runtime results must be captured on the board. A pass does not substitute for end-to-end sensor/actuator tests.

## Required hardware acceptance checks

| Requirement | Acceptance/evidence |
|---|---|
| NFR1 | 10 ms Motion period, <1 ms jitter under simultaneous Wi-Fi, USB, I²C and scans; no missed quadrature edges at measured maximum speed |
| NFR2 | Ten 1 m runs: <20 mm sideways drift; commanded distance within ±5%; check reverse and 90/180-degree turns at both battery extremes |
| NFR3 | Physical impact and E-stop to motor PWM zero <=50 ms; confirmed stall to zero <=50 ms; separately report the 200 ms stall detection window |
| Watchdog | Freeze Motion and each critical task; measure reset-to-disabled motor behavior and check idle/IRQ stack high-water marks |
| NFR4 | 4 Hz telemetry; disconnect AP and broker independently; autonomy continues, reconnect succeeds and timing remains within limits |
| FR1/FR5 | Confirm course barcode format first; proposed target >=95 correct decodes per 100 passes at 0.25 m/s with no wrong actions; test junctions, curves, gaps and reversed labels |
| FR2 | Determine threshold from worst sensing latency × speed + measured braking distance + margin; validate every bypass leg and robot footprint against obstacle shapes |
| FR3 | Line reacquisition within 3 s during approach, at most two short extensions, fault on failure; check heading after recovery |
| FR6 | Validate 5-degree climb and 2.5 g impact thresholds; known ramps, proposed peak-height error <=10 mm; test acceleration bias on GY-511 |
| Local inputs | Confirm GP18–21 wiring/polarity and debounce; STOP/E-stop still work with command buffer full |
| Recovery | Active sensor faults prevent reset/start; local reset requires released E-stop and stationary fresh sensors; RESET never starts motors |
| Compact sharing | Stress simultaneous GPIO edges, echo notifications, snapshots and MQTT; confirm no mixed packed fields and no out-of-range accepted payloads |
| Change evidence | Preserve comparable baseline/packed RAM maps, message-size measurements, timing captures and test logs |

The barcode success and hump error targets are proposed additions from the feedback, not supplied measured results. Confirm them with the course. U-turn support is implemented but course placement/rules remain to be confirmed.

## References used for implementation

- [Course kernel hub](https://github.com/sirfonzie/mtk3-hub) and [Pico port](https://github.com/sirfonzie/mtk3smp-rp2040): native kernel and single-core configuration.
- Pico SDK 2.3.1 local source headers and lwIP MQTT implementation: actual GPIO, PWM, ADC, I²C, async Wi-Fi and MQTT APIs.
- [TDK MPU6000/6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf): device identification and accelerometer setup.
- [ST LSM303DLHC datasheet](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf): GY-511 accelerometer setup and scale.
- [ZXing Code39Reader symbol definitions](https://github.com/zxing/zxing/blob/master/core/src/main/java/com/google/zxing/oned/Code39Reader.java): standard wide/narrow command character patterns. The streaming sensor decoder is project code.
