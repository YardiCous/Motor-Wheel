"""Verify that background USB/radio work cannot preempt any robot task."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = (ROOT / 'app_program/robot_os.c').read_text()
    values = re.search(r'g_priorities\[ROBOT_TASK_COUNT\]\s*=\s*\{([^}]+)\}', source)
    if values is None:
        raise SystemExit('Cannot discover robot task priorities')
    robot = [int(value.strip()) for value in values[1].split(',')]
    services = {}
    for name, path in [
        ('USB_TASK_PRIORITY', 'lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c'),
        ('CYW43_TASK_PRIORITY', 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c'),
    ]:
        match = re.search(r'^#define\s+' + name + r'\s+(\d+)\s*$',
                          (ROOT / path).read_text(), re.M)
        if match is None:
            raise SystemExit(f'Cannot discover {name}')
        services[name] = int(match[1])
    usb, wifi = services['USB_TASK_PRIORITY'], services['CYW43_TASK_PRIORITY']
    if not robot or not 0 < min(robot) <= max(robot) < usb < wifi:
        raise SystemExit(f'Priority error: robot={robot}, USB={usb}, WiFi={wifi}; '
                         'all robot tasks must preempt USB and WiFi')
    print(f'PASS: robot priorities {min(robot)}..{max(robot)} preempt USB {usb} and WiFi {wifi}')


if __name__ == '__main__':
    main()
