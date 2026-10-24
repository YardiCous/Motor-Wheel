# Cross-subsystem integration record

| Test | Scope | Result | Evidence |
|---|---|---|---|
| Host behavior | Barcode → navigation, obstacle plan → bypass/recovery, safety → output inhibition | PASS, synthetic inputs | [Host results](../evidence/host_tests.txt) |
| Wide/packed equivalence | Same controller logic and input scenarios, both representations | PASS | [Memory report](../../docs/memory_comparison.md) |
| C/Python protocol compatibility | C-generated full packet independently decoded in Python | PASS | [Host results](../evidence/host_tests.txt) |
| Physical integrated robot | H01–H11 | Not run | [Acceptance plan](../../docs/validation.md) |

## Physical run record

| Date | Test ID | Firmware/configuration | Tester | Result | Raw measurements / evidence link |
|---|---|---|---|---|---|
