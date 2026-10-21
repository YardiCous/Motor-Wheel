# Application data representation and binary protocol

Field ranges are explicit software bounds; calibration values remain provisional. Persistent application state uses bit-fields, modular counters or packed byte arrays. Wider integer/float locals are arithmetic intermediates, not stored application measurements. Native kernel/SDK handles, transport structures, stack sizes and pointers retain their platform types.

C bit-fields can include ABI padding and are never sent as wire structs. Packed interfaces use explicit masks/shifts through checked `bits_put/get`; fields cross byte boundaries. Bit zero is the least-significant bit of byte zero. Unused tail bits are zero. Values outside ranges are rejected or faulted; intentional modulo wrap is noted below.

## Interfaces

| Object / ordered fields | Range / encoding | Bits | Bytes |
|---|---|---|---:|
| command: opcode, sequence | 0..3; modulo 64 | 2 + 6 | 1 |
| motion command: kind, argument, speed | 0..4; -8192..8191 biased +8192; 0..125 × 2 mm/s | 3 + 14 + 7 | 3 |
| line: error, mode, seen, timestamp | -1000..1000 biased +1000; 0..5; bool; modulo 65536 ms | 11 + 3 + 1 + 16 | 4 |
| event: type, value, timestamp | 0..3; tagged union 0..1023; modulo 65536 ms | 2 + 10 + 16 | 4 |
| motion state: left/right speed, distance, mode, timestamp | -250..250 each biased +250; modulo 2^24 mm; 0..4; modulo 65536 ms | 9 + 9 + 24 + 3 + 16 | 8 |
| acknowledgement: sequence, accepted | modulo 64; bool; bit 7 reserved zero | 6 + 1 | 1 |

Motion kind: 0 STOP, 1 FOLLOW, 2 DISTANCE, 3 TURN, 4 SEARCH. Distance argument is signed millimetres. Turn argument is degrees, validated to ±360 by Motion; positive rotates right. Speed requests have 2 mm/s resolution.

Event tag: 0 BARCODE (value 1..5), 1 HUMP_START (0), 2 HUMP_END (0..1023 mm), 3 SCAN_DONE (0 none/abort, 1 left, 2 right). The union has a ten-bit maximum range; tag-specific values are also validated. The upper four bits of the final event byte are reserved zero. Critical event queue overflow latches a fault.

Motion distance is signed longitudinal displacement represented modulo 2^24 mm. A reverse move reduces it. IMU computes the shortest signed modulo difference between fresh samples; it never treats a wrap as a huge distance. Encoder counts similarly wrap modulo 65536, with fewer than 32768 counts permitted between snapshots. Timestamp comparisons require servicing within half the modulo range; each implemented deadline is <32768 ms.

## Stored subsystem fields

| Field / location | Valid range | Bits each | Representation |
|---|---|---:|---|
| vehicle state | 0..8 | 4 | controller bit-field |
| faults | ten independent flags | 10 | shared controller bitmask |
| latch, readiness, line-seen/missing, range-valid/exempt | bool | 1 | controller bit-fields |
| line mode | 0..5 | 3 | controller bit-field |
| range | 0..4095 mm | 12 | controller bit-field |
| sample/entry/deadline/heartbeat times | modulo 65536 ms | 16 | uint16_t |
| IR ADC samples | 0..4095 | 12 | shared bit-fields |
| encoder count | modulo 65536 | 16 | ISR uint16_t; shared bit-fields |
| encoder previous A/B | 0..3 | 2 | ISR bit-field |
| invalid-transition flag | bool | 1 | ISR bit-field |
| shared highest peak | 0..1023 mm | 10 | bit-field |
| barcode result | 0..5 | 3 | bit-field |
| scan side | 0..2 | 2 | bit-field |
| valid, ready, Wi-Fi/MQTT, scan, completion, reset flags | bool | 1 | synchronized bit-fields |
| reset generation | modulo 256 | 8 | bit-field |
| PID integral | -250000..250000 error·ms | 19 | signed bit-field, saturated |
| PID previous error | -500..500 mm/s | 10 | signed bit-field |
| wheel goal/progress | signed 24-bit μm; ±8192 mm command bound | 24 | signed bit-fields, checked before storage |
| odometry remainder | -999..999 μm | 11 | signed bit-field |
| accumulated displacement | modulo 2^24 mm | 24 | bit-field |
| active motion kind / speed | 0..4; 0..125 × 2 mm/s | 3 / 7 | bit-fields |
| barcode element widths | 10..250 ms (zero while empty) | 8 | nine uint8_t samples |
| barcode element count / phase | 0..9 / 0..4 | 4 / 3 | bit-fields |
| barcode color/gap/active flags | bool | 1 | bit-fields |
| IMU model / I²C address | 0..2 / 0..127 | 2 / 7 | bit-fields |
| consecutive IMU failures | 0..3 | 2 | saturated bit-field |
| calibration count / sum | 0..100 / ±180000 tenth-degrees | 7 / 19 | bit-fields |
| zero/filtered pitch | -1800..1800 tenth-degrees | 12 | signed bit-fields |
| height accumulator | -8388608..1023000 μm | 24 | checked signed bit-field |
| current hump peak | 0..1023 mm | 10 | bit-field |
| terrain state | 0..7 | 3 | bit-field |
| echo pulse width | 0..25000 μs | 15 | synchronized bit-field |
| echo armed/rising/ready | bool | 1 | synchronized bit-fields |
| scan phase / sample index / retry | 0..3 / 0..5 / 0..3 including terminal attempt | 2 / 3 / 2 | bit-fields |
| scan coarse ranges | 0..4095 mm | 12 | five bit-field records |
| bypass step / retry / previous vehicle | 0..6 / 0..2 / 0..8 | 3 / 2 / 4 | bit-fields |
| network state / backoff | 0..4 / 0..30 s | 3 / 5 | bit-fields |
| command/telemetry sequence | modulo 64 | 6 | bit-fields |
| network connection/DNS/receive flags | bool | 1 | bit-fields |
| button raw/stable | bool | 1 | bit-fields |

Kernel deadline variables are 32-bit native uptime values, maintained only for scheduling. Application event timestamps are narrowed on storage. Angles, gains, lookup patterns and pin assignments are immutable configuration constants.

## MQTT payloads

`robot/telemetry`: exactly **20 bytes**, 158 data bits; the top two bits of byte 19 are zero. `robot/status` carries the same frame once per second. The version bit is 1; 0 is reserved.

| Field in wire order | Range | Width |
|---|---|---:|
| version | 0..1 | 1 |
| vehicle | 0..8 | 4 |
| fault mask | ten flags | 10 |
| left / right speed | each -250..250 mm/s, +250 bias | 9 each |
| left / right encoder | modulo 65536 | 16 each |
| signed displacement | modulo 2^24 mm | 24 |
| left / center / right ADC | 0..4095 | 12 each |
| barcode action | 0..5 | 3 |
| highest hump peak | 0..1023 mm | 10 |
| ultrasonic range | 0..4095 mm | 12 |
| Wi-Fi / MQTT | bool | 1 each |
| sequence | modulo 64 | 6 |

`robot/command`: one byte, opcode in bits 0–1 and sequence in bits 2–7. Opcodes are 0 START, 1 STOP, 2 RESET, 3 ESTOP. Example: START sequence 1 is byte `0x04`; STOP sequence 2 is `0x09`. `robot/ack`: one byte, sequence bits 0–5, accepted bit 6, reserved bit 7 zero. A positive ACK means state acceptance, not manoeuvre completion. STOP/E-stop receive immediate acceptance; START/RESET wait for Navigation's guards.

Fault bits 0–9: stall, encoder, IMU, no echo, Wi-Fi/MQTT unavailable, impact, watchdog, queue overflow, navigation failure, E-stop. Only communication unavailability is advisory. Fault status qualifies sensor validity: invalid/no-echo is never interpreted as infinite clearance. Publish settings are non-retained; telemetry uses QoS0 and ACK uses QoS1. Do not publish retained START commands.

## RAM and message evidence

Compile-time assertions enforce the packed sizes on the Arm target. Compared with an **illustrative** int32-per-field representation:

| Message | Illustrative unpacked | Actual packed |
|---|---:|---:|
| command, two fields | 8 B | 1 B |
| motion, three fields | 12 B | 3 B |
| line, four fields | 16 B | 4 B |
| event, three fields | 12 B | 4 B |
| telemetry, seventeen fields | 68 B | 20 B |

The four native message buffers reserve 192 bytes including length headers/alignment: motion 32 B, events 64 B, commands 32 B, ACK 64 B. Their actual message payload capacity is 56 B. The latest line and motion snapshots add 12 B. Kernel object metadata, application structures, task stacks and library buffers are additional.

There is no historical unpacked firmware in this repository, so these are message layout comparisons, not measured before/after whole-device RAM savings. Use `arm-none-eabi-size build-pico/motor_wheel.elf` and the linker map for current static usage. Preserve equal-feature baseline/packed commits and compare `.data + .bss`, dynamic kernel allocations and task stack high-water marks using identical toolchains. Also compare runtime jitter and latency on the board; smaller payloads alone are not performance evidence.
