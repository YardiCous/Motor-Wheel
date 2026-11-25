# Subsystem 1 — Binary communications and telemetry

Student ID: 2500642. Source: `app_program/subsystem1/`.
Starter example test cases have been replaced by robot acceptance cases.

| Test | Method | Criterion | Status / evidence |
|---|---|---|---|
| Host behavior | Instrumented C99 suite | MQTT topic/length/fragment validation | PASS; [host log](../evidence/host_tests.txt) |
| Physical qualification | H02, H09 in the [test plan](../../docs/validation.md) | Apply the numerical acceptance criteria in that plan | Not run |

## Physical run record

Record a row for every measured run; retain previous results.
Video observations are reported in the [video index](../../videos/README.md#video-index).

| Date | Test ID | Firmware/configuration | Tester | Result | Measurement / evidence link |
|---|---|---|---|---|---|
| 2026-11-06 | ALL-U01 | `5d82670` | ALL | All subsystem LEDs blinked | [LED demo](https://youtube.com/shorts/w-7WDevNEQU) |
| 2026-11-18 | S4-U01 | `d26ab08` | 2500381, 2500642 | Telemetry operated during hump-height detection | [Hump-height and telemetry demo](https://youtube.com/shorts/p3Vi7uIfzRw) |
| 2026-11-18 | S3-U03 | `d26ab08` | 2501069, 2500642 | Telemetry operated during line tracking and barcode reading | [Line tracking, barcode and telemetry demo](https://youtube.com/shorts/Yu3QxEyx8QI) |
