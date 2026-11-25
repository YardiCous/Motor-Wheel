# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-10-23 | S2 | S2-U01 | `35d5c16` | | https://youtube.com/shorts/tDaFEudXuF0 | Motor moves front, reverses, turn left and turn right |
| 2026-11-06 | S1 - S5 | ALL-U01 | `5d82670` | | https://youtube.com/shorts/w-7WDevNEQU | All LEDs blinking |
| 2026-11-06 | S5 | S5-U01 | `5d82670` | | https://youtu.be/diCABZcrVwM | Obstacle can bypass |
| 2026-11-06 | S5 | S5-U02 | `5d82670` | | https://youtube.com/shorts/pmjnHA5H1rs | Obstacle can bypass |
| 2026-11-06 | S3 | S3-U01 | `5d82670` | | https://youtu.be/2otoH5Rzr1w | Barcode can get alphabet |
| 2026-11-06 | S3 | S3-U02 | `5d82670` | | https://youtu.be/RbboIaG_dns | Barcode can get alphabet |
| 2026-11-18 | S4 | S4-U01 | `d26ab08` | | https://youtube.com/shorts/p3Vi7uIfzRw | Can get hump height |
| 2026-11-18 | S3 | S3-U03 | `d26ab08` | | https://youtube.com/shorts/Yu3QxEyx8QI | Can line track and get barcode |
