# Subsystem 3 — Line tracking and barcode perception

Student ID: 2501069. Source: `app_program/subsystem3/`.
Starter example test cases have been replaced by robot acceptance cases.

| Test | Method | Criterion | Status / evidence |
|---|---|---|---|
| Host behavior | Instrumented C99 suite | Four symbols, ambiguous/reverse frame rejection and line loss | PASS; [host log](../evidence/host_tests.txt) |
| Physical qualification | H05, H08 in the [test plan](../../docs/validation.md) | Apply the numerical acceptance criteria in that plan | Not run |

## Physical run record

Record a row for every measured run; retain previous results.
Video observations are reported in the [video index](../../videos/README.md#video-index).

| Date | Test ID | Firmware/configuration | Tester | Result | Measurement / evidence link |
|---|---|---|---|---|---|
| 2026-11-06 | ALL-U01 | `5d82670` | ALL | All subsystem LEDs blinked | [LED demo](https://youtube.com/shorts/w-7WDevNEQU) |
| 2026-11-06 | S3-U01 | `5d82670` | 2501069 | Decoded a letter from a barcode | [S3-U01 barcode demo](https://youtu.be/2otoH5Rzr1w) |
| 2026-11-06 | S3-U02 | `5d82670` | 2501069 | Decoded a letter from a barcode | [S3-U02 barcode demo](https://youtu.be/RbboIaG_dns) |
| 2026-11-18 | S3-U03 | `d26ab08` | 2501069, 2500642 | Robot tracked the line and read a barcode while telemetry operated | [Line tracking, barcode and telemetry demo](https://youtube.com/shorts/Yu3QxEyx8QI) |
