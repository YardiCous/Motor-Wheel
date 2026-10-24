# BARR-C:2018 application coding rules

The attached BARR-C:2018 PDF is the governing standard for application code.
New modules use ISO C99, fixed-width integer types, unsigned encoded bit-fields,
`bool` for Boolean variables/one-bit fields, module-prefixed public functions and
types, private `static` state, and guarded matching headers. No allocation or
function-like macro is introduced in the application control logic.

Formatting uses spaces, LF, four-column indentation, ≤80 columns, Allman braces,
complete control blocks, one statement per line, aligned declarations/assignments,
BARR function-definition spacing and case-aligned `break`. Each C file includes
its own header, documents functions with Doxygen-compatible comments, and ends
with a file marker and blank line. Functions are limited to 100 lines.
[API reference](api.md) is generated from the public header comments.

New arithmetic uses fixed-point units. Signed values use offset binary on packed
interfaces; masks/shifts operate on unsigned operands. Unsigned counter/timer
wrap is deliberate. Cast comments identify the bounds or hardware contract.
Peripheral pointer casts, the PRIMASK assembly instruction and IRQ registration
are confined to `board_io`. T-Kernel's `TA_HLNG` assembly wrapper provides actual
interrupt entry/exit for its static `_isr` callbacks.

The existing kernel-required `usermain` symbol retains its legacy name and ABI;
new application functions use module prefixes. Standard `main` in the host
verification program likewise follows its external C runtime contract. Existing
kernel, SDK, and CYW43 source is not claimed to be rewritten or certified to
BARR-C. The modified MQTT adapter is formatted/reviewed with the application.
No new product-level exception is hidden in a compliance claim.

## Enforcement

`tools/check_barr.py` gates every application build and checks the mechanically
enforceable subset: whitespace/width, filenames, guards/header pairing, public
prefixes, function length, forbidden constructs and basic control-block style.
`tools/format_c.py` applies `.clang-format` and the BARR-specific finishing rules.
The checker's success is not a claim that it proves every BARR rule.

The host compiler additionally checks prototypes, sign conversions, shadowing,
initialization diagnostics and ordinary warnings, with warnings treated as errors.
AddressSanitizer and UndefinedBehaviorSanitizer exercise boundary/fault cases.
Hardware adapters compile as C99 with `-Wall -Wextra -Werror -Wshadow` and missing
prototype checks; inherited kernel headers retain their original ABI types.

Manual review remains necessary for semantic naming, comments, cast bounds,
logical operand parentheses, source organization, concurrency assumptions,
all branch/transition behavior and maintenance of these rules. Timing and
physical safety compliance require instrumented target tests. No formal external
BARR-C certification or independent peer review has been performed.

## Project abbreviations

| Term | Meaning |
|---|---|
| ADC | Analog-to-digital converter |
| API / ABI | Application programming / binary interface |
| AO | Analog output |
| BDU | Block-data update |
| GPIO | General-purpose input/output |
| HR | High resolution |
| IMU | Inertial measurement unit; here an accelerometer/magnetometer without gyro |
| ISR | Interrupt service routine |
| I2C | Inter-integrated circuit serial bus |
| IRQ | Interrupt request |
| MMIO | Memory-mapped input/output |
| MQTT | Message Queuing Telemetry Transport |
| PID | Proportional-integral-derivative controller |
| PWM | Pulse-width modulation |
| RMW | Read-modify-write |
| RTOS | Real-time operating system |
| SMP | Symmetric multiprocessing |
| `g` / `p` / `b` / `h` prefixes | Global / pointer / Boolean / handle |
| ms / us / um / mg / cm / mm | Milliseconds / microseconds / micrometres / milli-g / centimetres / millimetres |

Reusable source/header templates are in [tools/templates](../tools/templates/).
