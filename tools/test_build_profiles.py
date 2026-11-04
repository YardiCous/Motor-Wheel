"""Check firmware mode selection and image names without requiring the SDK."""
from itertools import product
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MODES = ("MOTOR_TEST", "ENCODER_TEST", "IMU_TEST", "OBSTACLE_TEST", "LINE_TEST")
SUFFIXES = ("_motortest", "_encodertest", "_imutest", "_obstacletest", "_linetest")
BASE_IMAGE = "mtk3pico_smp1_usb_cdc"


def main():
    # Execute the real profile-selection section in isolation. The remaining
    # makefile requires the SDK and removes old objects when profiles change.
    source = (ROOT / "build_make/pico_rp2040.mk").read_text()
    selection, separator, _ = source.partition("GCC := arm-none-eabi-gcc")
    assert separator, "Could not locate the end of profile selection"
    selection += "\n.PHONY: check_profile\ncheck_profile:\n"
    selection += "\t@printf '%s\\n' '$(EXE_FILE)' "
    selection += "'$(MOTOR_TEST) $(ENCODER_TEST) $(IMU_TEST) "
    selection += "$(OBSTACLE_TEST) $(LINE_TEST) $(BODY_SCAN)'\n"

    with tempfile.TemporaryDirectory(prefix="motor-profiles-") as temporary:
        makefile = Path(temporary) / "profiles.mk"
        makefile.write_text(selection)

        def check(options):
            return subprocess.run(
                ["make", "-s", "-f", str(makefile), "check_profile",
                 "WIFI=none", "NET=none", "SMP=1", "CONSOLE=usb_cdc",
                 "CRASH_REPORT=0", "TEST=0", *options],
                cwd=temporary, text=True, capture_output=True)

        defaults = check([])
        assert defaults.returncode == 0, defaults.stderr
        assert defaults.stdout.splitlines() == [BASE_IMAGE, "0 0 0 0 0 0"]
        allowed = 0
        rejected = 0
        for values in product((0, 1), repeat=len(MODES)):
            motor, encoder, imu, obstacle, line = values
            valid = ((motor + encoder + obstacle + line) <= 1 and
                     not (imu and (encoder or obstacle or line)))
            for body in (0, 1):
                options = [f"{name}={value}" for name, value in zip(MODES, values)]
                result = check([*options, f"BODY_SCAN={body}"])
                if not valid:
                    assert result.returncode != 0, options
                    assert ("Use only one bench test" in result.stderr or
                            "IMU_TEST=1 is only supported" in result.stderr)
                    rejected += 1
                    continue
                assert result.returncode == 0, result.stderr
                suffix = "".join(s for s, v in zip(SUFFIXES, values) if v)
                suffix += "_bodyscan" if body else ""
                expected_values = " ".join(map(str, (*values, body)))
                assert result.stdout.splitlines() == [
                    BASE_IMAGE + suffix, expected_values], options
                allowed += 1

        for name in (*MODES, "BODY_SCAN"):
            for invalid in ("2", "-1", "yes"):
                result = check([f"{name}={invalid}"])
                assert result.returncode != 0, (name, invalid)
                assert f"Unknown {name}" in result.stderr, result.stderr

    print(f"PASS: firmware profiles ({allowed} valid combinations, "
          f"{rejected} conflicts rejected, invalid switches rejected)")


if __name__ == "__main__":
    main()
