# Measured representation comparison

The ARM compiler measures one instance of each real controller
context. The reference widens each logical bit-field to uint32_t;
both variants pass the identical instrumented behavioral suite.
This is a reproducible representation baseline, not a historical
pre-change robot firmware measurement (the repository held examples).

| Context | Wide bytes | Packed bytes | Saved bytes |
|---|---:|---:|---:|
| telemetry_t | 16 | 4 | 12 |
| motion_t | 80 | 36 | 44 |
| line_t | 76 | 20 | 56 |
| safety_t | 12 | 4 | 8 |
| terrain_t | 80 | 20 | 60 |
| navigation_t | 60 | 12 | 48 |
| obstacle_t | 48 | 8 | 40 |
| Total | 372 | 104 | 268 |

Reproduce with `python3 tools/measure_memory.py`.

Allocated RTOS stacks and kernel queue headers are separate from
these task-local contexts; do not add their savings to the static
ELF BSS figure. Whole-image sizes are recorded in validation.md.
