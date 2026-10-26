# CSC20 verification

Run `python3 tools/run_tests.py` from the repository root for the instrumented
host logic tests. Run `python3 tools/measure_memory.py` for the equivalent wide
baseline and ARM layout measurements, and `python3 tools/measure_messages.py`
for actual packed payload/buffer sizes. Stored results are in `evidence/`.

`make -C build_make TEST=1` adds motor-free startup format/safety checks before
the robot tasks begin. Host and startup checks do not measure physical accuracy
or real-time deadlines. Follow [the physical validation plan](../docs/validation.md)
and put measurements/video links in the existing subsystem/integration logs.
The original empty test logs are retained as places for the team to record results.

For a direct bench check of the real motors, build and flash the opt-in
sequence with `MOTOR_TEST=1`:

```sh
make -C build_make -j8 MOTOR_TEST=1 WIFI=none NET=none
```

The test waits for START, then performs encoder-controlled forward, backward,
left-turn and right-turn phases with stops between them. Keep the wheels off
the ground, release START before boot, and keep the E-stop ready. It reports
each phase over the USB console and finishes with
`PHYSICAL MOTOR TEST: PASS - all phases complete`. RESET re-arms the test
without moving the motors.

To diagnose encoder wiring without energizing the motors:

```sh
make -C build_make -j8 ENCODER_TEST=1 WIFI=none NET=none
```

After flashing, press START and rotate each wheel by hand. The console reports
`L=<count> dL=<change> R=<count> dR=<change> valid=<0|1>` every 250 ms.
Counts should change when the corresponding wheel moves, and `valid=1` should
remain stable.
