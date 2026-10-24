# Message size comparison

Measured with explicit little-endian 32-bit reference fields and actual
packed byte counts. The starter had no equivalent robot messages; these
are reproducible before/after representation models, not historical logs.

| Message | Wide bytes | Packed bytes | Meaningful bits |
|---|---:|---:|---:|
| Motion snapshot | 20 | 4 | 32 |
| Motion command | 16 | 3 | 24 |
| Barcode event | 4 | 1 | 2 |
| Hump event | 12 | 2 | 10 |
| MQTT command | 8 | 1 | 6 |
| Supervisory queue entry | 12 | 5 | 38 |
| Telemetry wire packet | 108 | 28 | 223 |
| Telemetry queue entry | 112 | 32 | 255 |
| MQTT status | 12 | 1 | 4 |
| MQTT acknowledgement | 16 | 1 | 8 |

## Actual RTOS buffer allocation

The kernel rounds each message to a four-byte boundary and adds a
four-byte length header. Control blocks are unchanged and excluded.

| Queue | Depth | Wide buffer bytes | Packed buffer bytes |
|---|---:|---:|---:|
| Motion | 4 | 80 | 32 |
| Barcode | 4 | 32 | 32 |
| Hump | 4 | 64 | 32 |
| Supervisory | 8 | 128 | 96 |
| Telemetry | 1 | 116 | 36 |
| ACK | 8 | 128 | 64 |
| Total | | 548 | 292 |

A one-byte barcode message saves wire/application bytes but does not
reduce this kernel's per-entry allocation compared with a four-byte
reference message. The buffer totals account for that distinction.

Reproduce: `python3 tools/measure_messages.py`.
