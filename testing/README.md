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
