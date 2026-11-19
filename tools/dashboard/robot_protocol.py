"""Binary robot protocol, with independent host decoding."""
from __future__ import annotations

import argparse
import json

# Fields appear least-significant bit first. Wire order is independent of ABI.
FIELDS = [
    ("vehicle", 3), ("distance_mm", 12),
    ("speed_mm_second", 9), ("reverse", 1), ("motion_mode", 3),
    ("tick_5ms", 7), ("faults", 12), ("ir_left", 12),
    ("ir_centre", 12), ("ir_right", 12), ("terrain", 3),
    ("pitch_offset", 7), ("peak_mm", 8), ("range_cm", 9),
    ("servo_3deg", 6), ("barcode_event", 3), ("line_error_offset", 8),
    ("line_state", 3), ("obstacle_state", 3), ("link", 2),
    ("on_hump", 1), ("height_valid", 1), ("encoder_left", 32),
    ("encoder_right", 32), ("wheel_left_offset", 10),
    ("wheel_right_offset", 10), ("lifted", 1), ("line_guard", 1),
]
LEGACY_PACKET_BYTES = 28
BARCODE_PACKET_BYTES = LEGACY_PACKET_BYTES + 1
PACKET_BYTES = BARCODE_PACKET_BYTES + 2
BARCODE_LETTER_BITS = 5
OBSTACLE_WIDTH_BITS = 12
OBSTACLE_WIDTH_ONE_EDGE = 1 << OBSTACLE_WIDTH_BITS
PAYLOAD_BITS = 224 + BARCODE_LETTER_BITS + OBSTACLE_WIDTH_BITS
assert sum(width for _, width in FIELDS) == 223


def decode(payload: bytes) -> dict[str, int | float | bool | str | None]:
    """Validate length, padding and field ranges before scaling values."""
    if len(payload) not in (LEGACY_PACKET_BYTES, BARCODE_PACKET_BYTES, PACKET_BYTES):
        raise ValueError("telemetry requires 28, 29, or 31 bytes")
    word = int.from_bytes(payload[:LEGACY_PACKET_BYTES], "little")
    if word >> 223:
        raise ValueError("reserved padding bits are set")
    letter = payload[LEGACY_PACKET_BYTES] if len(payload) >= BARCODE_PACKET_BYTES else 0
    if letter > 26:
        raise ValueError("barcode letter code exceeds its valid range")
    width_field = int.from_bytes(payload[BARCODE_PACKET_BYTES:], "little")
    if width_field >> (OBSTACLE_WIDTH_BITS + 1):
        raise ValueError("reserved obstacle width bits are set")
    obstacle_width = width_field & (OBSTACLE_WIDTH_ONE_EDGE - 1)
    one_edge = bool(width_field & OBSTACLE_WIDTH_ONE_EDGE)
    if one_edge and not obstacle_width:
        raise ValueError("one-edge flag requires an obstacle width")
    result = {}
    offset = 0
    for name, width in FIELDS:
        result[name] = (word >> offset) & ((1 << width) - 1)
        offset += width
    for name, maximum in [("vehicle", 7), ("speed_mm_second", 511),
                          ("motion_mode", 6), ("terrain", 6),
                          ("range_cm", 400), ("servo_3deg", 60),
                          ("line_state", 5), ("obstacle_state", 7)]:
        if result[name] > maximum:
            raise ValueError(f"{name} exceeds its valid range")
    # Firmware uses 1 cm as a below-2-cm indication, including short echoes.
    if result["line_error_offset"] == 255:
        raise ValueError("line_error_offset is reserved (255)")
    result["pitch_deg"] = (result.pop("pitch_offset") - 64) / 2
    result["line_error"] = result.pop("line_error_offset") - 127
    result["wheel_left_mm_second"] = result.pop("wheel_left_offset") - 512
    result["wheel_right_mm_second"] = result.pop("wheel_right_offset") - 512
    result["servo_deg"] = result.pop("servo_3deg") * 3
    result["barcode_valid"] = bool(result["barcode_event"] & 4)
    result["barcode_command"] = result.pop("barcode_event") & 3
    result["barcode_letter"] = chr(ord("A") + letter - 1) if letter else None
    result["obstacle_width_mm"] = obstacle_width or None
    result["obstacle_width_one_edge"] = one_edge
    return result


def encode_command(command: str, sequence: int) -> bytes:
    """Commands use two bits and a four-bit modulo sequence; never retain them."""
    commands = {"stop": 0, "start": 1, "reset": 2, "estop": 3}
    if command not in commands or not 0 <= sequence <= 15:
        raise ValueError("command must be stop/start/reset/estop; sequence 0..15")
    return bytes([commands[command] | (sequence << 2)])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="operation", required=True)
    decoder = subcommands.add_parser("decode")
    decoder.add_argument("hex_payload")
    encoder = subcommands.add_parser("command")
    encoder.add_argument("command", choices=["stop", "start", "reset", "estop"])
    encoder.add_argument("sequence", type=int)
    args = parser.parse_args()
    if args.operation == "decode":
        print(json.dumps(decode(bytes.fromhex(args.hex_payload)), indent=2))
    else:
        print(encode_command(args.command, args.sequence).hex())


if __name__ == "__main__":
    main()
