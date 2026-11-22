# GP28 Code 39 barcode protocol

No official course barcode symbology or sample was supplied. Confirm the course
printout before track qualification. The current scanner implementation reads
standard Code 39 patterns and maps A..Z letters to junction actions.

GP26 and GP27 straddle the tracking line. GP28 reads a separate barcode lane.
ADC readings are preserved at 12 bits. Larger readings mean black, with per-sensor
default thresholds configured in `robot_config.h`. The sensor thresholds can be
calibrated with measured white/black samples.

The streaming scanner starts at a white-to-black edge while the wheels move.
It measures every alternating bar and gap in mean wheel encoder edges. It stops
after a long white quiet zone or immediately when `scanner_flush()` is called as
the line sensors reach a junction. The scan uses encoder distance rather than
timing, so its measurements are intended to remain stable across forward speed
changes.

Each Code 39 character contains nine alternating elements: five bars and four
gaps, starting with a bar. Exactly three elements are wide. Characters are
separated by a narrow white gap, and printed barcodes are framed by `*`; for
example, `*A*` is 29 measured elements. The decoder accepts `*X*`, `*X`, `X*`,
or a lone `X`-style partial reading and rejects unknown or unclear patterns.
Reverse travel is handled by trying the widths in reverse.

The last A..Z letter is remembered until the next junction. The table in
`junction.c` assigns one action to each letter:

- A..P, R..V and X..Y: FORWARD
- F: U-TURN
- Q and Z: RIGHT
- No letter: FORWARD

The table is explicit implementation data and may be changed for the course.
The LEFT action exists in the API but no letter currently maps to it.

At a junction, the remembered letter is used once and cleared. The robot then
executes FORWARD, LEFT, RIGHT or U-TURN, while unknown and non-letter readings
are ignored. Host tests cover the letter/action table and stored one-shot
behavior. Physical qualification still requires the actual printed barcodes,
entry speed, junction position and scanner threshold to be confirmed.

The old two-bit framed barcode remains opt-in only for host compatibility:
`ROBOT_LINE_FRAMED_BARCODE=1` enables the legacy implementation in `line.c`.
The normal firmware default is `0`, with the streaming Code 39 scanner active.
