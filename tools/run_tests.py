"""Run the real C logic with strict diagnostics and memory/UB instrumentation."""
from pathlib import Path
import argparse
import os
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CORE = ["data_formats.c", "subsystem1/telemetry.c", "subsystem2/motion.c",
        "subsystem3/line.c", "subsystem4/safety.c", "subsystem4/terrain.c",
        "subsystem5/junction.c", "subsystem5/navigation.c",
        "subsystem5/obstacle.c"]
FLAGS = ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wshadow",
         "-Wstrict-prototypes", "-Wmissing-prototypes", "-Wsign-conversion",
         "-pedantic", "-DROBOT_ENCODER_EDGES_TURN=960u",
         "-DROBOT_TRACK_MM=140u", "-DROBOT_QUARTER_TURN_MM=109u",
         "-DROBOT_LINE_STRADDLE=0u", "-DROBOT_LINE_FRAMED_BARCODE=1u",
         "-DROBOT_CRUISE_MM_SECOND=500u", "-DROBOT_CAUTIOUS_MM_SECOND=250u",
         "-DROBOT_TURN_MM_SECOND=150u", "-DROBOT_SEARCH_MS=3000u",
         "-DROBOT_BYPASS_MAX_STEPS=12u"]


def build(root, executable, sanitize=True, bench=False):
    command = [os.environ.get("CC", "gcc"), *FLAGS, "-g", "-O1"]
    if sanitize:
        command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    if bench:
        command += ["-DOBSTACLE_TEST=1"]
    command += [str(root / "testing/host/verify_main.c")]
    command += [str(root / "testing/host/motor_test.c")]
    command += [str(root / "testing/host/terrain_test.c")]
    command += [str(root / "app_program" / source) for source in CORE]
    command += ["-o", str(executable)]
    subprocess.run(command, check=True)


def build_board_io(root, executable):
    command = [os.environ.get("CC", "gcc"), *FLAGS, "-g", "-O1",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "-DBOARD_IO_HOST_TEST=1", "-I" + str(root / "testing/board_io"),
               str(root / "testing/board_io/verify_range.c"),
               str(root / "app_program/board_io.c"), "-o", str(executable)]
    subprocess.run(command, check=True)


def build_clock(root, executable):
    command = [os.environ.get("CC", "gcc"), "-std=c99", "-g", "-O1",
               "-Wall", "-Wextra", "-Werror", "-D_PICO_RP2040_",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "-I" + str(root / "testing/clock"),
               "-I" + str(root / "include"), "-I" + str(root / "config"),
               str(root / "testing/clock/verify_clock.c"),
               str(root / "kernel/sysdepend/pico_rp2040/cpu_clock.c"),
               "-o", str(executable)]
    subprocess.run(command, check=True)


def run(executable):
    environment = dict(os.environ)
    # These tests allocate no heap memory. The managed runner uses ptrace,
    # which prevents LeakSanitizer's process inspection; ASan/UBSan stay on.
    environment["ASAN_OPTIONS"] = "detect_leaks=0"
    environment["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    return subprocess.check_output([str(executable)], text=True, env=environment)


def build_robot_os(root, executable):
    # ASan's global registration would retain the unused RTOS task-entry
    # table and prevent linking just the real functions under test.
    command = [os.environ.get("CC", "gcc"), *FLAGS, "-g", "-O1",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "--param=asan-globals=0",
               "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
               "-I" + str(root / "testing/robot_os"),
               str(root / "testing/robot_os/verify_startup.c"),
               str(root / "app_program/data_formats.c"),
               str(root / "app_program/subsystem3/scanner.c"),
               str(root / "app_program/subsystem3/code39.c"),
               "-o", str(executable)]
    subprocess.run(command, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    subprocess.run([sys.executable, str(ROOT / "tools/check_barr.py")], check=True)
    subprocess.run([sys.executable, str(ROOT / "tools/check_task_priorities.py")], check=True)
    with tempfile.TemporaryDirectory(prefix="motor-test-") as temporary:
        executable = Path(temporary) / "verify"
        build(ROOT, executable)
        output = run(executable)
        board_executable = Path(temporary) / "verify_board_io"
        build_board_io(ROOT, board_executable)
        output += run(board_executable)
        clock_executable = Path(temporary) / "verify_clock"
        build_clock(ROOT, clock_executable)
        output += run(clock_executable)
        os_executable = Path(temporary) / "verify_robot_os"
        build_robot_os(ROOT, os_executable)
        output += run(os_executable)
        bench_executable = Path(temporary) / "verify_bench"
        build(ROOT, bench_executable, bench=True)
        output += run(bench_executable)
        print(output, end="")
        from robot_protocol import decode, encode_command
        vector = next(line.split()[1] for line in output.splitlines()
                      if line.startswith("VECTOR "))
        decoded = decode(bytes.fromhex(vector))
        expected = {"distance_mm": 4095, "speed_mm_second": 500,
                    "pitch_deg": -5.0, "peak_mm": 43, "range_cm": 400,
                    "encoder_left": 4294967295, "encoder_right": 2,
                    "wheel_left_mm_second": -500,
                    "wheel_right_mm_second": 500, "servo_deg": 150,
                    "barcode_command": 3, "barcode_valid": True,
                    "faults": 0x123}
        assert all(decoded[key] == value for key, value in expected.items())
        for command in ("stop", "start", "reset", "estop"):
            for sequence in range(16):
                assert encode_command(command, sequence)[0] < 64
        try:
            decode(bytes(27) + b"\x20")
        except ValueError:
            pass
        else:
            raise AssertionError("decoder accepted nonzero reserved padding")
        print("PASS: Python command encoder and decoder rejection checks")
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output +
                "PASS: Python command encoder and decoder rejection checks\n")


if __name__ == "__main__":
    main()
