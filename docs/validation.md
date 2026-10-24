# Validation and demonstration record

Host/source checks and firmware builds are executable evidence. Physical tests
are **not performed**: no connected robot, track, broker or logic analyser was
used. Blank physical results below must be filled with measurements, not inferred
from a successful build or host simulation.

## Completed checks

| Check | Result | Reproduction / evidence |
|---|---|---|
| Application ISO C99 and strict compiler diagnostics | Pass | `python3 tools/run_tests.py` |
| Packed bit boundaries, neighbor preservation, overflow rejection | Pass | `testing/host/verify_main.c`, `testing/evidence/host_tests.txt` |
| C full telemetry vector decoded independently in Python | Pass | Same test runner and `tools/robot_protocol.py` |
| All 4096 combinations of twelve fault bits | Pass | Exhaustive safety test; WiFi/MQTT advisory exclusions |
| Direction/wrap guard, 1-m synthetic move, turn, stall, E-stop, pre-emption | Pass | Motion and interlock tests |
| Sub-mm hump increments, peak retention, overflow invalidation, impact, failed readings | Pass | Terrain tests |
| Four barcode symbols, one-shot event, scan timeout, line loss | Pass | Line tests; course format remains provisional |
| Coarse/fine scan, retry/abort, median filtering | Pass | Obstacle tests |
| Bypass/recovery, 3-s search timeout, barcode slowdown, obstacle-on-hump stop | Pass | Navigation/interlock tests |
| MQTT zero-byte preservation, topic/length/fragment/reserved-bit rejection | Pass | Fragment tests; broker session behavior still requires integration |
| Equivalent wide-reference and packed controllers | Pass | `python3 tools/measure_memory.py`; same sanitizer suite for both |
| ARM controller state sizes | Measured | `testing/evidence/memory_comparison.json`: 372 →104 bytes, seven contexts |
| Queue/wire message sizes and allocation overhead | Measured | `testing/evidence/message_comparison.json` |
| BARR automated subset | Pass | `python3 tools/check_barr.py`; semantic review scope in coding_standard.md |
| Pico W firmware / MQTT firmware / TEST=1 / SMP=0 | See build matrix | Final sizes in `testing/evidence/build_sizes.txt` |

The managed execution runner uses ptrace, so LeakSanitizer process inspection is
unavailable. The tests allocate no heap; leak inspection is disabled while address
and undefined-behavior instrumentation remain enabled. Sanitizer errors halt tests.

## Representation comparison method

There was no previous functional robot application: the baseline repository held
LED/producer/consumer examples. Do not describe its smaller image as a functional
robot before/after comparison. The reproducible representation baseline is generated
from the implemented controller fields, widened to 32-bit fields, then verified
with the **identical** behavioral tests. Both forms are compiled for the ARM probe
to measure real `sizeof`/alignment. Seven packed contexts total 104 bytes instead
of 372 (268 bytes / 72.04% reduction). These task-local type sizes are separate
from fixed task-stack reservations, not an extra static-BSS saving.

The six bounded queues require 292 buffer bytes rather than the equivalent wide
model's 648 bytes. Telemetry on the wire is 28 bytes rather than 108 bytes in the
explicit wide reference. Kernel length headers/alignment are included in the queue
comparison. No unmeasured historical JSON/text size is claimed.

Full image `data+bss` is reported separately. Stack reservation is 14,336 bytes
for application tasks; measure high-water use during the tests below. The kernel
and network heaps/pools are additional and are not represented by controller sizes.

## Physical acceptance and demonstration plan

Run the sequence first with wheels lifted, then on the track. Record firmware
commit, compiler/build profile, configuration, battery voltage, instruments and
raw logs/video link for each result. Use the same course and settings for packed
and equivalent reference timing when evaluating packing overhead.

| Test ID | Cross-subsystem test and procedure | Numerical success criterion | Required evidence | Result |
|---|---|---|---|---|
| H01 | Gordon + Jia Jun: scope actual A/B encoders at 0.25 and 0.5 m/s, forward/reverse/braking; compare against independent counter | Zero missed edges; correct direction; dispatcher completes before next worst-case edge | Scope trace, encoder counts, handler duration | Not run |
| H02 | Gordon + Wylie: capture GP18 loop starts with telemetry, AP loss and reconnection | 10-ms motion period, jitter <1 ms; no burst catch-up; no deadline fault during WiFi loss | Timestamp CSV and logic-analyser trace | Not run |
| H03 | Ernest + Gordon: induce collision and inject E-stop while MOVE/TURN/FOLLOW active; separately confirm stall | Motor PWM disabled ≤50 ms after impact/E-stop or stall confirmation; stall detection window separately ≤200 ms + scheduling | GP18/IMU event/motor-output scope trace, ten cases/state | Not run |
| H04 | Gordon + Yu Da: ten 1-m moves, reverse moves, and 90°/180° turns at high/low battery | Distance ±5%; lateral drift <20 mm over 1 m; record turn error and calibrate track | Measured distances, angles, video | Not run |
| H05 | Jia Jun + Gordon: official barcode samples at 0.25 and 0.5-m/s entry, junctions/curves and interrupted frames | Ten successful passes per command per speed; zero wrong commands; no false 150-ms loss during valid frame; timeout ≤1 s | ADC/encoder/state logs and video | Not run |
| H06 | Ernest + Gordon: flat calibration, speed steps, ten known ramps at both speeds, tilt and I2C disconnection | Peak error ≤10 mm; no flat acceleration false hump; third failed sample invalidates sensor and latches fault | Ramp dimensions, mg/speed/height CSV, video | Not run |
| H07 | Yu Da + Gordon: approach boxes at both speeds and worst battery/surface; capture each ping and stop | Ping separation ≥60 ms, median window ≤180 ms + measured scheduling; stopping clearance ≥120 mm; establish a≥0.8 m/s² or revise threshold | Echo/trigger/motor trace and distance video | Not run |
| H08 | Yu Da + Gordon + Jia Jun: ten bypasses per side around course-envelope boxes | No contact; fixed-path clearance validated; stationary scan ≤5 s; line reacquired ≤3 s after bypass; no false line-loss fault during detour | State/range/encoder logs and video | Not run |
| H09 | Wylie + Navigation: start offline locally; drop AP and broker during motion; restore both; use retained, duplicate, stale, oversized and fragmented commands | Autonomous operation continues; reconnect/subscription restores; 4-Hz fresh telemetry; no retained START execution; correct ACKs and bounded queues | Broker capture, local button/video and task timing | Not run |
| H10 | Team: repeated packed shared updates under simultaneous encoder/echo load; saturate command/event queues | No torn fields or lost source faults; essential overflow safely latches; telemetry remains latest-only; IRQ exclusion within measured edge/jitter budget | Stress logs, scope and queue high-water values | Not run |
| H11 | Team: full course plus fault clear/RESET/local START, repeated on minimum battery | RESET returns to stopped IDLE; separate START required; task-stack and heap high-water fit allocated budgets | Full video, heap/stack logs and configuration | Not run |

Week 13 evidence consists of the data representation inventory, these reproducible
memory/message comparisons, executable host tests, firmware build matrix, and the
physical results above once measured. No Zoom recording or hardware demonstration
is manufactured as part of this implementation.
