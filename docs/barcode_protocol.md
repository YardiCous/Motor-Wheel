# Provisional four-command barcode protocol

No official course barcode symbology or sample was supplied. This is an explicit,
implemented assumption, retained after the user directed us to follow the
documented hardware first. Confirm the course printout before track qualification.
The format can be changed within `line.c` without changing the packed command API.

GP26 and GP27 straddle the tracking line; GP28 observes a separate barcode lane
along straight track. Larger ADC readings mean black; use `line_calibrate` with
measured white/black references, or adjust the configured threshold. Preserve
12-bit ADC readings. The default threshold is a provisional 2048.

A frame starts after a black guard 24..36 mm long. It then contains:

1. White gap, 4..12 mm.
2. Black data bar for the most significant bit.
3. White gap, 4..12 mm.
4. Black data bar for the least significant bit.
5. White gap, 4..12 mm.
6. Black stop guard, 24..36 mm, terminated by white.

A data bar 4..12 mm means 0; 14..22 mm means 1. Widths in between or outside
these bands invalidate the frame. Commands are 00 LEFT, 01 RIGHT, 10 STRAIGHT,
and 11 U-TURN. Widths come from forward modulo-4096 encoder distance, not time.
Reverse travel invalidates a frame. A complete frame times out at 1000 ms.

At 0.5 m/s the entry guard provides multiple samples before the controller
slows to 0.25 m/s for SCAN/JUNCTION. A barcode event is emitted exactly once
on a valid stop edge. Line-loss suppression during SCAN is limited by the frame
timeout; JUNCTION suppression lasts at most 300 ms. Outside planned departures,
150 ms without line visibility enters line search. Search is limited to 3 s.

LEFT/RIGHT execute a calibrated 90-degree encoder turn. U-TURN uses twice that
wheel arc. STRAIGHT moves 120 mm before returning to line following. The last
valid decoded command remains available in telemetry; queue events are one-shot.

Host tests cover all four symbols, malformed frames/timeouts, and line loss.
Hardware qualification requires ten successful runs per symbol at 0.25 m/s,
then the 0.5-m/s entry speed with automatic slowdown, with zero wrong commands.
