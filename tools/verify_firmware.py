#!/usr/bin/env python3
"""Validate an RP2040 UF2 and its linked kernel symbols without executing firmware."""
import argparse
import pathlib
import struct
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("build", type=pathlib.Path)
parser.add_argument("--network", action="store_true", help="also require linked MQTT/WiFi functions")
args = parser.parse_args()
blob = (args.build / "motor_wheel.uf2").read_bytes()
assert blob and len(blob) % 512 == 0, "invalid UF2 length"
blocks = len(blob) // 512
addresses = set()
for index in range(blocks):
    block = blob[index * 512:(index + 1) * 512]
    magic0, magic1, flags, address, length, number, count, family = struct.unpack_from("<8I", block)
    assert (magic0, magic1) == (0x0A324655, 0x9E5D5157), "bad UF2 header"
    assert struct.unpack_from("<I", block, 508)[0] == 0x0AB16F30, "bad UF2 trailer"
    assert flags & 0x2000 and family == 0xE48BFF56, "not an RP2040 image"
    assert length == 256 and number == index and count == blocks, "bad block numbering"
    assert 0x10000000 <= address < 0x10200000 and address % 256 == 0, "outside Pico W flash"
    assert address not in addresses, "duplicate UF2 address"
    addresses.add(address)
symbols = subprocess.check_output(["arm-none-eabi-nm", str(args.build / "motor_wheel.elf")], text=True)
names = {line.split()[-1] for line in symbols.splitlines() if line.split()}
required = {"tk_cre_tsk", "tk_wai_flg", "tk_rcv_mbf", "knl_dispatch_entry", "kernel_start", "board_init"}
if args.network:
    required |= {"cyw43_arch_init", "mqtt_client_connect", "mqtt_publish"}
assert required <= names, f"missing linked symbols: {required - names}"
assert not ({"xTaskCreate", "vTaskStartScheduler", "vTaskDelay"} & names), "unexpected FreeRTOS scheduler"
print(f"Verified {blocks} RP2040 UF2 blocks, flash bounds, native micro-T symbols" +
      (", and Pico SDK WiFi/MQTT symbols." if args.network else "."))
