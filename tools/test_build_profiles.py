"""Check firmware mode selection and image names without requiring the SDK."""
from itertools import product
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MODES = ("MOTOR_TEST", "ENCODER_TEST", "OBSTACLE_TEST", "LINE_TEST", "BARCODE_TEST")
SUFFIXES = ("_motortest", "_encodertest", "_obstacle_nolinetest", "_linetest")
BARCODE_SUFFIXES = ("", "_barcoderaw", "_barcodedecode")
BASE_IMAGE = "mtk3pico_smp1_usb_cdc"


def main():
    # Execute the real profile-selection section in isolation. The remaining
    # makefile requires the SDK and removes old objects when profiles change.
    source = (ROOT / "build_make/pico_rp2040.mk").read_text()
    selection, separator, _ = source.partition("GCC := arm-none-eabi-gcc")
    assert separator, "Could not locate the end of profile selection"
    selection += "\n.PHONY: check_profile\ncheck_profile:\n"
    selection += "\t@printf '%s\\n' '$(EXE_FILE)' "
    selection += "'$(MOTOR_TEST) $(ENCODER_TEST) $(OBSTACLE_TEST) "
    selection += "$(LINE_TEST) $(BARCODE_TEST)'\n"

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
        assert defaults.stdout.splitlines() == [BASE_IMAGE, "0 0 0 0 0"]
        allowed = 0
        rejected = 0
        for values in product(*(range(2) for _ in SUFFIXES), range(3)):
            valid = sum(value != 0 for value in values) <= 1
            options = [f"{name}={value}" for name, value in zip(MODES, values)]
            result = check(options)
            if not valid:
                assert result.returncode != 0, options
                assert ("Use only one bench test" in result.stderr or
                        "BARCODE_TEST cannot be combined" in result.stderr)
                rejected += 1
                continue
            assert result.returncode == 0, result.stderr
            suffix = "".join(s for s, v in zip(SUFFIXES, values) if v)
            suffix += BARCODE_SUFFIXES[values[-1]]
            expected_values = " ".join(map(str, values))
            assert result.stdout.splitlines() == [
                BASE_IMAGE + suffix, expected_values], options
            allowed += 1

        for name in MODES:
            first_invalid = "3" if name == "BARCODE_TEST" else "2"
            for invalid in (first_invalid, "-1", "yes"):
                result = check([f"{name}={invalid}"])
                assert result.returncode != 0, (name, invalid)
                assert f"Unknown {name}" in result.stderr, result.stderr

    print(f"PASS: firmware profiles ({allowed} valid combinations, "
          f"{rejected} conflicts rejected, invalid switches rejected)")


if __name__ == "__main__":
    main()
