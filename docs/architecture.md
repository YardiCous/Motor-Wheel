# Architecture and wiring

## Source decisions

The Week 6 architecture supplies the seven owners and timing requirements. CSC20 feedback corrects encoder direction, local controls, stale-data policies and obstacle timing. Change Requirement 1 adds minimum-bit storage, compact binary communication and synchronized packed access. The user's later instructions select Pico W, micro-T and Pico SDK; they supersede the initial FreeRTOS/host scaffold. Prompts inside the supplied PDFs were treated as reference material, not instructions to perform submissions or external actions.

The course hub links [the Pico port](https://github.com/sirfonzie/mtk3smp-rp2040). Its unmodified kernel sources and licenses are in `third_party/micro_t`; `ORIGIN.md` records the exact commit. The SDK integration here is project-specific and does not inherit the upstream port's hardware qualification.

## Startup and ownership

Pico SDK initializes the RP2040 and its RAM interrupt table. `main.c` sets 125 MHz, initializes motor outputs to zero, sets up hardware and USB, then enters the native kernel. `kernel_core.c` supplies a separate 48 KiB kernel arena, initializes objects and starts the initial task. `kernel_sdk.c` installs PendSV and SysTick handlers in the SDK table. The tick is 1 ms; SysTick and PendSV share lowest interrupt priority as required by the supplied dispatcher. The SDK remains responsible for boot2, reset, USB and peripheral interrupts. The upstream BSP's reset, vector table, UART pin setup and example application are excluded.

Core 1 stays parked (`CNF_SMP=0`). Application snapshots use short SDK interrupt-disable/restore sections. These sections are not an SMP lock; enabling core 1 would require a separate design and validation. A 4 KiB idle/temporary stack accommodates SDK interrupts provisionally; measure stack use on hardware.

| Task | Service interval | micro-T priority | Stack bytes | Owner |
|---|---|---:|---:|---|
| Safety | event or 20 ms | 1 | 2048 | fault latch, local buttons, freshness, watchdog |
| Motion | 10 ms | 2 | 3072 | only runtime motor PWM writer |
| Line | 5 ms | 2 | 3072 | ADC and barcode state |
| IMU | 10 ms | 3 | 4096 | I²C1, pitch/impact/height |
| Navigation | 10 ms | 3 | 3072 | commands, mission and manoeuvre sequencing |
| Obstacle | ECHO event or 5 ms | 4 | 3072 | ultrasonic and servo |
| Network | 10 ms polling; 250 ms publishing | 5 | 8192 | CYW43, lwIP and MQTT |

Smaller numbers mean higher priority. Periodic tasks use absolute kernel deadlines; the adapter accounts for micro-T's extra timeout tick. Missed deadlines yield instead of spinning. No WCET/jitter claim follows from a successful build.

Message buffers use `tk_cre_mbf`, `tk_snd_mbf` and `tk_rcv_mbf`. Tasks use `tk_cre_tsk`, `tk_sta_tsk`, `tk_dly_tsk`; safety and ECHO use `tk_set_flg`/`tk_wai_flg`. There is no FreeRTOS API shim. The SDK may compile a guarded file named `cyw43_arch_freertos.c`; it contributes no FreeRTOS scheduler in the selected polling architecture.

## Pin map

| Function | GPIO | Notes |
|---|---|---|
| Ultrasonic TRIG / ECHO | 0 / 1 | external 5 V sensor supply, divide/level-shift ECHO to 3.3 V |
| IMU I²C1 SDA / SCL | 2 / 3 | confirm sensor/address and pull-ups |
| Left encoder A / B | 4 / 5 | both channels required |
| Left motor A / B | 8 / 9 | PWM, 20 kHz at 125 MHz |
| Right motor A / B | 10 / 11 | PWM, 20 kHz |
| Scan servo | 12 | 50 Hz, default 1000–2000 μs travel |
| Right encoder A / B | 16 / 17 | both channels required |
| START / STOP / RESET / E-stop | 18 / 19 / 20 / 21 | proposed active-low buttons, internal pull-ups; confirm wiring |
| IR left / center / right | 26 / 27 / 28 | ADC0–2 |

Keep the Robo Pico GP28 VBAT-sense jumper open. GP26 is shared by two Grove connectors: wire each IR signal separately without another cable driving GP26. USB stdio avoids GP0/1 UART conflict. No application pins touch Pico W radio pins GP23/24/25/29. An E-stop input is a software stop; motor-driver electrical behavior and reset-state pull-downs must match the board.

A single SDK GPIO callback handles the four encoder inputs, ECHO rising/falling edges and E-stop. It enters/leaves micro-T's interrupt context around native signaling. It never logs or waits. Encoder counts are uint16 modulo values; snapshots copy both counts and invalid-transition flags with IRQs masked. Direction comes from A/B transitions, never PWM sign.

## Communication and freshness

| Channel | Capacity | Full policy | Data-age policy |
|---|---:|---|---|
| latest line snapshot | 1 × 4 B | overwrite atomically | 20 ms |
| latest motion snapshot | 1 × 8 B | overwrite atomically | 30 ms for IMU |
| q_motion_cmd | 4 × 3 B | latch overflow | one current manoeuvre, 15 s finite-motion timeout |
| q_event | 8 × 4 B | latch overflow | reject/fault active navigation after 100 ms |
| q_cmd_in | 4 × 1 B | negative remote ACK | processed once per 10 ms |
| q_ack | 8 × 1 B | drop, do not block controls | drain when connected |
| telemetry | one assembled 20 B snapshot | drop publication on TCP/MQTT pressure | regenerated every 250 ms |

The former separate event queues are consolidated into one tagged queue. Latest-value snapshots replace the proposed telemetry queue so obsolete telemetry cannot accumulate. Native message buffers copy payloads; they also consume a four-byte length header and word alignment per message. STOP/E-stop bypass `q_cmd_in`.

## Control and recovery

Readiness requires valid IR/motion integration, IMU calibration, a valid forward range and confirmed calibration settings. Wi-Fi does not gate readiness. START is accepted only in IDLE; STOP enters COMPLETE; E-stop/fatal faults latch FAULT. RESET returns to IDLE without starting. A cleared cause alone never clears the latch.

Local RESET acknowledges historical stall/impact/encoder/navigation/queue/watchdog/E-stop faults only while stopped, with E-stop released and live sensors. The operator must remove the cause first. Continuously measured IMU/no-echo causes clear only after successful readings. Remote RESET cannot clear those historical causes itself. Each service observes a reset generation and resets only its own private state; Navigation never writes another task's local control state.

Motion computes signed count deltas, fixed-point PID, straight-line mismatch correction and independent wheel completion. PWM goes to zero on stop, stale inputs, invalid encoders or faults. Each wheel has a 200 ms no-motion stall confirmation. This is distinct from the <=50 ms requirement after confirmation; whether the requirement means physical stall onset needs course clarification. The 30 ms hardware watchdog stops being fed if Motion heartbeat exceeds 15 ms or another critical heartbeat exceeds 50 ms. Reset shutdown behavior still requires electrical measurement.

Line loss >150 ms enters search. Barcode/junction interpretation suppresses that timer only within bounded contexts. Code 39 frames have a 5 s total limit, a 1 s start-symbol acquisition limit, 10–250 ms elements and a trailing quiet interval. Continuous full-width black is treated as a junction; an extended junction faults after 1 s. Ambiguous patterns cause no navigation command. The chosen Code 39 format and thresholds must be confirmed on the course.

The IMU driver auto-detects MPU6050 (0x68/0x69) or LSM303DLHC accelerometer (0x19); it does not assume a gyro on GY-511. Both use accelerometer-derived pitch, assuming X forward and Z upward, with a configurable pitch sign. Stationary calibration uses 100 samples; impact threshold is provisionally 2.5 g. Three I²C read failures invalidate the IMU, latch a fault and retry initialization. Climb starts at 5 degrees; height integrates signed distance × sin(pitch), with a 3 s false-climb limit and 10 s overall hump limit. Acceleration can disturb this estimate: measure against known ramps.

Obstacle ranging triggers every >=60 ms with a bounded 10 μs pulse. ECHO edges signal the obstacle task; a 25 ms timeout handles missing echoes. No echo is invalid, not unlimited clearance. On obstacle detection the motors stop before scanning. While stopped, a scan may temporarily exempt only forward-range freshness; line and task watchdogs continue. Five coarse angles and two fine angles choose clearance, with at most two retries per failed reading and an 8 s scan deadline. The servo returns forward and obtains a valid sample before releasing the motion hold.

The bypass uses encoder-completed turns and distances: turn away, lateral offset, realign, pass the obstacle, turn toward the line, approach it and restore the original heading. This expands the review's three-step sketch because three moves alone do not return to the original line. Distances are configurable and need course validation. Forward ranging protects moving legs; it does not provide full side/rear coverage or prove clearance of every obstacle shape. Recovery allows two short extensions within a 3 s deadline, then latches a fault.

## Network

The Network task exclusively owns SDK `pico_cyw43_arch_lwip_poll` and lwIP raw MQTT calls; it polls every 10 ms. Association, DNS and MQTT connect use asynchronous state transitions, with bounded retries up to 30 s. Wi-Fi loss is advisory and does not stop autonomy. START/RESET enforce a six-bit sequence window; STOP/E-stop remain accepted regardless of duplicate sequence. Retained/DUP START is rejected using the SDK 2.3.1 MQTT parser header. This private-header integration must be checked when upgrading SDK/lwIP. Sequence numbers are not authentication.
