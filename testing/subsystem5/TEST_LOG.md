# Subsystem 5 — Obstacle handling and navigation

Student ID: 2500145. Source: `app_program/subsystem5/`.
Starter example test cases have been replaced by robot acceptance cases.

| Test | Method | Criterion | Status / evidence |
|---|---|---|---|
| Host behavior | Instrumented C99 suite | Coarse/fine scan, bypass, recovery, caution and timeout | PASS; [host log](../evidence/host_tests.txt) |
| Physical qualification | H07, H08 in the [test plan](../../docs/validation.md) | Apply the numerical acceptance criteria in that plan | Not run |

## Physical run record

Record a row for every measured run; retain previous results.
Video observations are reported in the [video index](../../videos/README.md#video-index).

| Date | Test ID | Firmware/configuration | Tester | Result | Measurement / evidence link |
|---|---|---|---|---|---|
| 2026-11-06 | ALL-U01 | `5d82670` | ALL | All subsystem LEDs blinked | [LED demo](https://youtube.com/shorts/w-7WDevNEQU) |
| 2026-11-06 | S5-U01 | `5d82670` | 2500145 | Robot bypassed an obstacle | [S5-U01 obstacle bypass demo](https://youtu.be/diCABZcrVwM) |
| 2026-11-06 | S5-U02 | `5d82670` | 2500145 | Robot bypassed an obstacle | [S5-U02 obstacle bypass demo](https://youtube.com/shorts/pmjnHA5H1rs) |
