"""Build supported Pico firmware profiles sequentially and record ELF RAM sizes."""
from pathlib import Path
import argparse
import os
import subprocess
import tempfile

from format_c import ROOT

PROFILES = [
    ("dual-core-radio", [], "mtk3pico_smp1_usb_cdc_wifi"),
    ("dual-core-offline", ["WIFI=none"], "mtk3pico_smp1_usb_cdc"),
    ("dual-core-mqtt", ["NET=lwip", "MQTT=1"],
     "mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt"),
    ("dual-core-mqtt-selftest", ["NET=lwip", "MQTT=1", "TEST=1"],
     "mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test"),
    ("single-core-mqtt", ["NET=lwip", "MQTT=1", "SMP=0"],
     "mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt"),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", default=os.environ.get("PICO_SDK_PATH"))
    args = parser.parse_args()
    if not args.sdk:
        raise SystemExit("Set PICO_SDK_PATH or pass --sdk /path/to/pico-sdk")
    rows = ["profile text_bytes data_bytes bss_bytes static_ram_bytes status"]
    for name, options, image in PROFILES:
        with tempfile.NamedTemporaryFile(prefix=f'motor-{name}-', suffix='.log',
                                         delete=False) as log:
            result = subprocess.run(["make", "-C", str(ROOT / "build_make"),
                                     "-j8", f'PICO_SDK_PATH={args.sdk}', *options],
                                    stdout=log, stderr=subprocess.STDOUT)
            log_path = Path(log.name)
        if result.returncode:
            print(log_path.read_text()[-10000:])
            raise SystemExit(f'FAIL {name}; full log: {log_path}')
        output = subprocess.check_output([
            "arm-none-eabi-size", str(ROOT / "build_make" / (image + ".elf"))],
            text=True)
        text, data, bss = map(int, output.splitlines()[-1].split()[:3])
        assert (ROOT / "build_make" / (image + ".uf2")).is_file()
        row = f'{name} {text} {data} {bss} {data + bss} PASS'
        rows.append(row)
        print(row, flush=True)
        # Kernel/SDK warnings are reported too; application warnings are errors.
        warnings = [line for line in log_path.read_text().splitlines()
                    if "warning:" in line]
        if warnings:
            rows.extend("WARNING " + warning for warning in warnings)
    guard = subprocess.run([
        "arm-none-eabi-gcc", "-std=c99", "-D_PICO_RP2040_",
        "-DTM_CONSOLE_UART=1", "-I" + str(ROOT / "include"),
        "-I" + str(ROOT / "config"), "-I" + str(ROOT / "kernel/knlinc"),
        "-fsyntax-only", str(ROOT / "app_program/board_io.c")],
        text=True, capture_output=True)
    assert guard.returncode != 0 and "Robot ultrasonic pins require USB" in guard.stderr
    rows += ["UART/CRASH_REPORT pin conflict rejected: PASS", "",
             "Original starter (different functionality, not packing baseline):",
             "dual-core-radio 285752 88 24476 24564 measured before replacement",
             "", "Static RAM = data+bss; dynamic RTOS stacks/queues and network heap",
             "must be accounted separately. See docs/validation.md."]
    (ROOT / "testing/evidence/build_sizes.txt").write_text("\n".join(rows) + "\n")


if __name__ == "__main__":
    main()
