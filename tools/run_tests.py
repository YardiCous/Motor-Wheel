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
        "subsystem5/navigation.c", "subsystem5/obstacle.c"]
FLAGS = ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wshadow",
         "-Wstrict-prototypes", "-Wmissing-prototypes", "-Wsign-conversion",
         "-pedantic"]


def build(root, executable, sanitize=True):
    command = [os.environ.get("CC", "gcc"), *FLAGS, "-g", "-O1"]
    if sanitize:
        command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    command += [str(root / "testing/host/verify_main.c")]
    command += [str(root / "testing/host/motor_test.c")]
    command += [str(root / "app_program" / source) for source in CORE]
    command += ["-o", str(executable)]
    subprocess.run(command, check=True)


def run(executable):
    environment = dict(os.environ)
    # These tests allocate no heap memory. The managed runner uses ptrace,
    # which prevents LeakSanitizer's process inspection; ASan/UBSan stay on.
    environment["ASAN_OPTIONS"] = "detect_leaks=0"
    environment["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    return subprocess.check_output([str(executable)], text=True, env=environment)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    subprocess.run([sys.executable, str(ROOT / "tools/check_barr.py")], check=True)
    with tempfile.TemporaryDirectory(prefix="motor-test-") as temporary:
        executable = Path(temporary) / "verify"
        build(ROOT, executable)
        output = run(executable)
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
            decode(bytes(28))
        except ValueError:
            pass
        else:
            raise AssertionError("decoder accepted a zero protocol version")
        print("PASS: Python command encoder and decoder rejection checks")
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output +
                "PASS: Python command encoder and decoder rejection checks\n")


if __name__ == "__main__":
    main()
