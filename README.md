# Motor-Wheel — Pico W / micro-T

CSC20 robot firmware built **only for Raspberry Pi Pico W (RP2040)**, using native **μT-Kernel 3.0** and the **Raspberry Pi Pico SDK**.

The kernel is vendored from the Pico port linked by the supplied [course resource](https://github.com/sirfonzie/mtk3-hub): [sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040), commit `2e8e6ad480c2d84a369b12995694b9bf5e9ddfb3`. This application uses its single-core scheduler. Pico SDK owns boot, clocks, vectors, peripherals, USB and Wi-Fi; the project adapter installs micro-T's dispatcher and tick without a second startup implementation.

## Build

Requirements: CMake 3.20+, Arm GNU `arm-none-eabi` compiler, Python 3, and a Pico SDK checkout with its submodules. Verified here with SDK **2.3.1** and Arm GCC **14.2.1**.

```sh
export PICO_SDK_PATH=/absolute/path/to/pico-sdk
cmake -S . -B build-pico -DCMAKE_BUILD_TYPE=Debug
cmake --build build-pico -j4
python3 tools/verify_firmware.py build-pico
```

The SDK locates/builds its `picotool` and `pioasm` utilities. If picotool is already installed, pass `-Dpicotool_DIR=/path/to/picotool/cmake/package`. An offline build needs these SDK tools available locally. Do not set `PICO_NO_PICOTOOL=1` if you need a UF2.

Outputs: `build-pico/motor_wheel.uf2`, `.elf`, `.bin`, `.hex`, and `.elf.map`. CMake rejects any board other than `pico_w`. Use a fresh build directory when replacing the earlier host build.

## Configure your robot

Copy `include/robot_local.example.h` to `include/robot_local.h` for local calibration overrides. Defaults and proposed button pins are in `include/robot_config.h`. Confirm wheel geometry, quadrature counts per revolution, encoder/motor signs, IR thresholds, servo direction and PID gains. Set `ROBOT_CALIBRATION_CONFIRMED` to `1` in the local file after entering the correct values. With its default `0`, the firmware runs tasks and diagnostics but rejects START.

The documents do not identify the final IMU or barcode format. This implementation supports automatic detection of **MPU6050** or **LSM303DLHC/GY-511**, with an explicit selection override. It implements forward **Code 39** frames `*L*`, `*R*`, `*S*`, `*U*`, `*E*` for left/right/straight/U-turn/end. Confirm this matches the course before enabling motion. This is a concrete configurable implementation, not a claim that unspecified hardware/course choices have been resolved.

For networking, copy `firmware/robot_secrets.example.h` to `firmware/robot_secrets.h` and set Wi-Fi/broker details. Both local files are ignored by Git. Without credentials, local operation still works. Commands and telemetry are binary; see [the protocol table](docs/data-representation.md). MQTT uses the configured local broker over TCP; TLS is not implemented.

Wi-Fi driver initialization and station setup run before the scheduler and safety
watchdog start, because the SDK uses blocking sleeps during radio startup. The
Network task handles polling, association, MQTT and connection retries. If driver
initialization fails, local operation continues with networking disabled until reboot.

Run `python3 tests/test_kernel_startup.py` to check kernel startup ordering and
dispatch guards using simulated kernel boundaries. This does not validate interrupt
delivery on the board; use `ROBOT_ON_TARGET_TESTS=ON` for on-board tests.

## Implemented

- Seven native micro-T tasks, copied message buffers, event flags and synchronized compact snapshots.
- Per-wheel fixed-point PID, direction-aware quadrature counting, distance/turn completion, line steering, stall detection and motor ownership in one task.
- Three ADC line sensors, junction handling, bounded Code 39 decoding and line search.
- Bounded IMU reads, stationary pitch calibration, impact detection and hump-height integration using wheel distance.
- Periodic ultrasonic ranging with edge IRQs, coarse/fine servo scanning, encoder-driven bypass and bounded line recovery.
- Local START/STOP/RESET/E-stop, latched faults, freshness checks and hardware watchdog recovery.
- Nonblocking Wi-Fi/MQTT connection states, retries, 4 Hz packed telemetry, command checks and binary acknowledgements.

## Formatting and checks

Project C/H files follow `.clang-format` (LLVM base, four spaces, 100 columns). Vendored kernel sources retain upstream formatting and notices.

```sh
cmake --build build-pico --target format-check  # requires clang-format
cmake --build build-pico --target format
```

Optional tests are **embedded in a Pico W image**, not built as host executables:

```sh
cmake -S . -B build-pico-tests -DROBOT_ON_TARGET_TESTS=ON
cmake --build build-pico-tests -j4
```

The test image checks packed formats, controller guards and encoder arithmetic at startup, then exercises the real micro-T message buffers, event flags and delays. A failed assertion stops startup with motor outputs off. USB output can be missed if the host connects after startup; use a debugger to inspect assertions. The cross-build and UF2 checks have been performed here; these on-board tests have **not** been executed on physical hardware.

Read [architecture and wiring](docs/architecture.md), [data representation](docs/data-representation.md), and [validation status](docs/validation.md) before board integration. The code is implemented and cross-compiled; timing, braking, barcode recognition and navigation accuracy still require the actual robot.
