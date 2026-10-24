"""Generate API and field inventories from the reviewed C headers."""
from pathlib import Path
import re

from format_c import ROOT, sources
from robot_protocol import FIELDS, PACKET_BYTES

MAXIMUMS = {
    "motion_t": {"mode": 6, "speed": 500, "progress_um": 4095999,
                 "position_um": 4095999, "elapsed_ms": 10000,
                 "stall_left_ms": 200, "stall_right_ms": 200,
                 "integral_left": 32000, "integral_right": 32000,
                 "prior_left": 2000, "prior_right": 2000, "remainder": 1918},
    "line_t": {"state": 5, "error": 254, "segment": 6, "lost_ms": 150,
               "frame_ms": 1000, "calibration_samples": 32, "junction_ms": 300},
    "terrain_t": {"state": 6, "samples": 32, "baseline_mg": 1000,
                  "last_speed": 1023, "climb_ms": 3000, "hump_ms": 10000,
                  "fractional": 18, "calibration_sum": 32000},
    "obstacle_t": {"state": 6, "angle": 60, "best_angle": 60, "best_cm": 400,
                   "range_cm": 400, "coarse_index": 5, "fine_index": 3,
                   "scan_ms": 5000},
    "navigation_t": {"stage": 8, "elapsed_ms": 10000},
    "mqtt_utk_state_t": {"state": 4, "retry_exponent": 5},
    "cyclic_t": {"phase": 99},
}


def main():
    api = ["# Application API", "", "Generated from public header comments by",
           "`python3 tools/generate_docs.py`.", ""]
    for path in sources():
        if path.suffix != ".h":
            continue
        entries = re.findall(r'/\*\*\s*@brief\s+(.*?)\*/\s*([^;{}]+\([^;{}]*\));',
                             path.read_text(), re.S)
        if entries:
            api += [f'## {path.relative_to(ROOT)}', ""]
            for comment, declaration in entries:
                comment = re.sub(r'\n\s*\*\s?', ' ', comment).strip()
                declaration = " ".join(declaration.split())
                api += [f'`{declaration};`', "", comment, ""]
    (ROOT / "docs/api.md").write_text("\n".join(api))
    data = ["# Application data representation", "",
            "All transport fields use explicit little-endian masks/shifts. Internal",
            "C bit-fields are task-private or protected under the core-0 IRQ lock;",
            "their compiler-specific ordering is never a network or queue ABI.", "",
            "Widths below are information widths. Byte boundaries, compiler structure",
            "padding and naturally aligned native cells can leave unused physical bits;",
            "those are not extra application fields. Reserved wire bits are zero.", "",
            "Arithmetic temporaries keep sufficient native precision before encoding.",
            "Pointers, RTOS IDs, stack pointers, hardware register images and kernel",
            "timestamps retain their platform contracts; they are not narrowed as if",
            "they were bounded robot states. The table's full-width counter/time fields",
            "have the entire unsigned 32-bit modulo range as their valid domain.", "",
            "## Telemetry wire layout", "",
            "| Field | Bit offset | Bits | Meaning / valid range |",
            "|---|---:|---:|---|"]
    meanings = {"version": "1; reject other versions", "vehicle": "0..7 state",
                "distance_mm": "0..4095 mm; wraps; signed delta requires <2048 mm",
                "speed_mm_second": "0..511; command target is at most 500; sign is separate",
                "reverse": "0/1, actual centre-speed sign",
                "motion_mode": "0..6", "tick_5ms": "0..127, wraps at 640 ms",
                "faults": "12 independent bits, 0..4095",
                "terrain": "0..6", "pitch_offset": "0..127 = -32..31.5 degrees",
                "peak_mm": "0..255 mm; overflow sets RANGE and clears validity",
                "range_cm": "2..400 cm; 0 means invalid; 1 and 401..511 rejected",
                "servo_3deg": "0..60 = 0..180 degrees; selected profile angle in PLAN, otherwise commanded servo angle",
                "barcode_event": "code2 + known1; last valid barcode, not a queue pulse",
                "line_error_offset": "0..254 = -127..127; 255 unused",
                "line_state": "0..5", "obstacle_state": "0..6",
                "link": "WiFi1 | MQTT1", "on_hump": "0/1, whole-hump status",
                "height_valid": "0/1; never infer validity from a zero height",
                "wheel_left_offset": "0..1023 = -512..511 mm/s, bias 512",
                "wheel_right_offset": "0..1023 = -512..511 mm/s, bias 512"}
    offset = 0
    for name, width in FIELDS:
        default = "0..4095 ADC counts" if name.startswith("ir_") else \
                  "0..4294967295, unsigned wrapping encoder count"
        data.append(f'| {name} | {offset} | {width} | {meanings.get(name, default)} |')
        offset += width
    assert offset == 223
    data += [f'| Reserved padding | 223 | 1 | Must be zero; total {PACKET_BYTES} bytes |', "",
             "Motion snapshot offset 5 occupies exactly 32 bits: 12+9+1+3+7.",
             "Its full-width heartbeat age must be ≤20 ms before terrain uses it;",
             "the seven-bit stamp alone cannot detect long-stale data after wrap.", "",
             "## Stored controller and ISR fields", "",
             "For each encoded nonnegative range 0..maximum, required bits are",
             "`ceil(log2(maximum + 1))`. Offset binary is used for signed quantities.",
             "One-bit logical fields are C99 `bool` bit-fields. Saturation/fault rules",
             "and the units for biased fields are described below and in the headers.", "",
             "| Module / type | Field | Valid encoded range | Minimum bits | Representation |",
             "|---|---|---|---:|---|"]
    for path in sources():
        text = path.read_text()
        for body, typename in re.findall(r'typedef struct\s*\{(.*?)\}\s*(\w+);', text, re.S):
            for name, width in re.findall(r'(?:uint(?:8|16|32)_t|bool)\s+(\w+)\s*:\s*(\d+);', body):
                width = int(width)
                maximum = MAXIMUMS.get(typename, {}).get(name, (1 << width) - 1)
                if maximum.bit_length() != width:
                    raise ValueError(f'{typename}.{name}: width {width}, maximum {maximum}')
                data.append(f'| {path.stem} / {typename} | `{name}` | 0..{maximum} | '
                            f'{width} | {"bool bit-field" if name.startswith("b_") else "unsigned bit-field"} |')
            for name in re.findall(r'uint32_t\s+(\w+)\s*;', body):
                meaning = "32-bit modulo counter/time"
                bits = 32
                if name == "command":
                    bits = 24
                    meaning = "explicit kind2 + distance12 + speed9 + reverse1; high8 zero"
                data.append(f'| {path.stem} / {typename} | `{name}` | '
                            f'0..{(1 << bits) - 1} | {bits} | {meaning} |')
    data += ["", "## Other shared fields and queue representations", "",
             "| Data | Valid range / meaning | Required bits | Backing storage / synchronization |",
             "|---|---|---:|---|",
             "| ADC triplet | Three values 0..4095 | 36 | Five bytes; four unused bits zero |",
             "| Fault source array | Six independently owned twelve-bit masks | 72 | Nine bytes; all RMW under saved IRQ mask |",
             "| Task heartbeat / command-arrival / retry deadline / reset time | Full unsigned modulo-2^32 ms | 32 each | Native 32-bit fields; core-0 protected snapshots |",
             "| Encoder pair | Each full unsigned modulo-2^32 | 64 | ISR writes; task copies both under IRQ mask |",
             "| Motion command | kind0..3, distance0..4095 mm, speed0..500, reverse0/1 | 24 | Three-byte queue message |",
             "| Barcode queue event | LEFT/RIGHT/STRAIGHT/U-TURN | 2 | One byte; high six bits zero |",
             "| Hump queue event | finished0/1, peak0..255 mm, valid0/1 | 10 | Two bytes; high six bits zero |",
             "| Supervisory queue entry | command2 + sequence4 + arrival32 | 38 | Five bytes; two unused bits zero; age≤500 ms |",
             "| Telemetry queue entry | application223 + arrival32 | 255 | 32 bytes; one padding bit zero; latest only |",
             "| ACK | echoed command2 + sequence4 + accepted1 | 7 | One byte; bit7 used by transport for malformed messages |",
             "| Status | protocol version2 + WiFi1 + MQTT1 | 4 | One byte, upper four bits zero |", "",
             "## Units, validity and overflow", "",
             "* Encoder deltas use unsigned subtraction in either direction; more than",
             "  64 edges per wheel/10-ms sample is an encoder/range guard fault.",
             "* Stored steering uses bias 127. Wheel-speed reports use bias 512.",
             "  Measured magnitude uses the full nine-bit 0..511 range to preserve",
             "  encoder quantization around a 500-mm/s target. Wheel reports use",
             "  -512..511; measurements outside that range set RANGE before saturation.",
             "  Signed PID integrals use bias 16000 and previous errors bias 1000.",
             "* Fine height/peak are 0..65535 in 0.01-mm units. A signed fractional",
             "  carry -9..9 is stored as 0..18 with bias 9, in 0.001-mm units.",
             "  A >255-mm report sets RANGE; saturation never retains valid height.",
             "* Terrain baseline stores milli-g +500. Last terrain speed stores",
             "  signed mm/s +512. IMU failures saturate at three; retries and all",
             "  state timers saturate or transition before their bit-field limit.",
             "* Distance state uses 0..4095999 micrometres (22 bits). The division",
             "  remainder -959..959 is stored with bias 959 (11 bits).",
             "* Queue overflow never blocks control. Essential command/event overflow",
             "  sets a fatal bit; routine telemetry replaces the oldest entry.",
             "  ACK overflow drops the ACK only. Fatal causes remain in Safety's mask.",
             "* Every packed shared RMW is IRQ-protected and every caller is core 0.",
             "  ISR-owned encoder/echo state uses coherent masked snapshots; volatile",
             "  prevents compiler elision but does not provide synchronization by itself.", "",
             "See [message_comparison.md](message_comparison.md) and",
             "[memory_comparison.md](memory_comparison.md) for measured comparisons.", ""]
    (ROOT / "docs/data_representation.md").write_text("\n".join(data))


if __name__ == "__main__":
    main()
