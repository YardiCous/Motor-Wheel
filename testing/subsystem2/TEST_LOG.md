# Subsystem 2 — Motion, encoders and PID

Student ID: 2500206. Source: `app_program/subsystem2/`.
Starter example test cases have been replaced by robot acceptance cases.

| Test | Method | Criterion | Status / evidence |
|---|---|---|---|
| Host behavior | Instrumented C99 suite | Distance/turn completion, wrap, stall and pre-emption | PASS; [host log](../evidence/host_tests.txt) |
| Physical qualification | H01, H02, H03, H04, H07 in the [test plan](../../docs/validation.md) | Apply the numerical acceptance criteria in that plan | Not run |

## Physical run record

Record a row for every measured run; retain previous results.
Video observations are reported in the [video index](../../videos/README.md#video-index).

| Date | Test ID | Firmware/configuration | Tester | Result | Measurement / evidence link |
|---|---|---|---|---|---|
| 2026-10-23 | S2-U01 | `35d5c16` | 2500206 | Demonstrated forward and reverse motion and left and right turns | [Motor movement demo](https://youtube.com/shorts/tDaFEudXuF0) |
| 2026-11-06 | ALL-U01 | `5d82670` | ALL | All subsystem LEDs blinked | [LED demo](https://youtube.com/shorts/w-7WDevNEQU) |
