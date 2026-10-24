# CSC20 ownership and interfaces

Assignments come from the supplied design review and Tribunal 1 documents.

| Buddy | Student ID | Subsystem | Code |
|---|---|---|---|
| Wylie | 2500642 | Binary communications and telemetry | `app_program/subsystem1/`, `mqtt_utk.c` |
| Gordon | 2500206 | Motion, encoders, PID | `app_program/subsystem2/` |
| Jia Jun | 2501069 | Line and barcode perception | `app_program/subsystem3/` |
| Ernest | 2500381 | Terrain, collision, safety | `app_program/subsystem4/` |
| Yu Da | 2500145 | Obstacle handling and navigation | `app_program/subsystem5/` |

`robot_os` owns RTOS integration, shared packed fields, and lifecycle.
`board_io` confines MMIO and ISR installation; its runtime peripheral callers
are limited to their owning tasks below.

## Pins and peripherals

| Pins | Peripheral / signal | Owner |
|---|---|---|
| GP0 / GP1 | HC-SR04 TRIG / divided ECHO | Obstacle task / shared GPIO dispatcher |
| GP2 / GP3 | I2C1 SDA / SCL; LSM303DLHC at 0x19 | Terrain task |
| GP4 / GP5 | Left encoder A/B | GPIO ISR; motion reads snapshots |
| GP8 / GP9 | Left bridge, PWM slice 4 A/B | Motion task |
| GP10 / GP11 | Right bridge, PWM slice 5 A/B | Motion task |
| GP12 | Servo, PWM slice 6 A | Obstacle task |
| GP16 / GP17 | Right encoder A/B | GPIO ISR; motion reads snapshots |
| GP18 | Actual motion-loop timing pulse | Motion task |
| GP20 | Local START, active low | Navigation task |
| GP21 | Local latched E-stop, active low | Navigation / Safety |
| GP22 | Local RESET, active low | Navigation task |
| GP26 / GP27 | ADC0/1, left/centre tracking inputs | Line task |
| GP28 | ADC2, separate barcode lane | Line task |
| GP23 / 24 / 25 / 29 | Reserved radio connections | Kernel WiFi service |
| GP6 / 7 / 13 / 14 / 15 / 19 | Unclaimed | None |

The HC-SR04 needs external 5 V, common ground, and a divider/level shifter on
ECHO. Leave the GP28 VBAT-sense jumper open. Route all three IR AO signals
individually; do not allow a second Grove cable to drive GP26. The IMU and I2C
pull-ups use 3.3 V. Verify motor and encoder polarity on blocks before driving.

The application owns ADC and I2C1 directly. The unused kernel ADC/I2C drivers
are disabled. PWM slices 4, 5, 6 and TIMER alarm 3 are reserved; the kernel's
optional physical PWM timers remain disabled. There is one GPIO ISR for the
four encoder edges and both echo edges. The application never replaces the
CYW43 PIO/DMA interrupt owners.

## Tasks and kernel objects

All seven application tasks and the existing WiFi task are pinned to processor
1 (physical core 0) in SMP builds. Core 1 does not access application packed data.

| Task | Priority | Release | Stack bytes |
|---|---:|---|---:|
| Safety | 1 | 20 ms and immediate fault wake | 2048 |
| Motion | 2 | 10 ms | 2048 |
| Line/barcode | 2 | 5 ms | 2048 |
| Terrain | 3 | 10 ms | 2048 |
| Navigation | 5 | 10 ms | 2048 |
| Obstacle | 6 | 60-ms pings; median every 180 ms; bounded settle | 2048 |
| Telemetry producer | 8 | 250 ms | 2048 |

USB stays at kernel priority 3 and WiFi at 4. The priority change from the
proposal places terrain ahead of WiFi, while navigation remains below it.
The RTOS reserves **14,336 bytes** for these seven task stacks, separately from
static `.bss` and queue storage. Stack high-water measurement remains open.

| Interface | Producer → consumer | Payload / queue capacity | Policy |
|---|---|---|---|
| Motion command | Navigation → Motion | 24 bits in 3 bytes; 4 entries / 32 buffer bytes | Polling send; overflow latches fault; STOP flag flushes all entries |
| Barcode event | Line → Navigation | 2 bits in 1 byte; 4 entries / 32 buffer bytes | One consumer; overflow latches fault |
| Hump event | Terrain → Navigation | finished1 + peak8 + valid1, 2 bytes; 4 entries / 32 buffer bytes | Navigation alone consumes; ON_HUMP is separate shared status |
| Supervisory command | Network → Navigation | command6 + arrival32, 5 bytes; 8 entries / 96 buffer bytes | Reject age >500 ms, duplicate/stale sequence, illegal state |
| Telemetry | Producer → network owner | 28-byte packet + 4-byte time, 32 bytes; 1 entry / 36 buffer bytes | Replace oldest; reject age >500 ms; no outage backlog |
| Acknowledgement | Navigation → network owner | 7 bits in 1 byte; 8 entries / 64 buffer bytes | Polling; excess ACKs dropped; motion never waits |
| Echo completion | GPIO ISR → Obstacle | 1 event flag bit | Clear before ping; wait at most 25 ms |
| Periodic releases | One 5-ms cyclic handler → tasks | Kernel wake-up requests | Collapse queued releases; no catch-up burst |
| Latest state | Each owner → all interested readers | Explicitly packed 28-byte image | All RMW and coherent copies under saved core-0 IRQ mask |
| Fault causes | Six owners → Safety | Six 12-bit masks, 9 bytes | Merge under IRQ mask; fatal bits latch until RESET |

Application budget: 7 tasks, 6 message buffers, 1 event flag and 1 cyclic
handler, within the configured limits. Kernel USB/WiFi allocations are additional.
