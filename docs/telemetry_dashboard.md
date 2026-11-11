# Live telemetry dashboard

Run one Python file to subscribe, decode and display the robot's readings in
your browser. There is no frontend build, Node.js, browser MQTT extension or
broker WebSocket configuration.

From the repository root, set up the one MQTT dependency once:

```sh
python3 -m venv .venv
# Linux / macOS:
source .venv/bin/activate
# Windows PowerShell instead: .venv\Scripts\Activate.ps1
python -m pip install -r tools/dashboard/requirements-dashboard.txt
```

Then, each time:

```sh
python tools/dashboard/telemetry_dashboard.py
```

The launcher opens **http://127.0.0.1:8765**. Leave the terminal running;
Ctrl+C stops it. Keep the repository files together: the launcher uses
`tools/dashboard/robot_protocol.py` and `tools/dashboard/telemetry_dashboard.html` alongside it.

The broker defaults to `MQTT_BROKER_IP` / `MQTT_BROKER_PORT` in your existing
`config/wifi_secrets.h`, or `localhost:1883` if that file is absent. Topic names
come from `app_program/subsystem1/telemetry.h`. This reads configuration without
compiling or running C, and never exposes WiFi passwords to the browser.
Restart the launcher after changing configuration or decoder code.

If stream activity reports `ValueError: reserved sensor value`, update both
`robot_protocol.py` copies and restart the Python launcher. Firmware reports
`range_cm=1` for a below-2-cm echo; the dashboard displays this as `<2 cm`.
Obstacle state `7` means body scan. These values must remain visible rather
than causing the whole telemetry packet to be rejected. Updating this decoder
does not require reflashing the robot or correct a false physical echo.

For readings stuck at `<2 cm`, current MQTT firmware also prints USB `RANGE:`
summaries and `ECHO:` diagnostics for failed or below-range pings. Capture a
few lines with a stationary target at a known distance. `width` is the
captured pulse duration; `rise_after` is the time from the software trigger
timestamp to the rising-edge ISR, so it includes interrupt latency. These
diagnostics preserve the existing close-stop behaviour.

USB disconnecting and reconnecting, Wi-Fi joining again, and echo counters
returning to 1 indicate a full chip restart. The GP22 software-reset message
does not apply to that path. Firmware startup now explicitly initializes the
microsecond timer's reference; `ECHO` diagnostics show `tick=0x20c` when its
enable bit and 12-cycle divider are correct. This removes reliance on timer
setup inherited from USB BOOTSEL flashing.

The robot must be built with `NET=lwip MQTT=1`, and its configured broker must
be reachable from the computer running Python. The dashboard connects to an
existing MQTT broker; it does not start one or flash the robot.

## Preview without hardware

```sh
python tools/dashboard/telemetry_dashboard.py --demo
```

Demo mode needs only Python 3.10+ and the repository files. It creates valid
28-byte packets and feeds them through the same decoder as real telemetry.
The dashboard clearly labels simulated readings. Robot Start and Stop controls
are disabled in demo mode.

## Connection options

```sh
python tools/dashboard/telemetry_dashboard.py --broker 192.168.1.10
python tools/dashboard/telemetry_dashboard.py --broker broker.example.com --broker-port 8883 --tls
python tools/dashboard/telemetry_dashboard.py --port 8766 --no-browser
```

`--topic` overrides the telemetry topic. `--ca path/to/ca.pem` supplies a custom
TLS CA certificate together with `--tls`. The client always verifies TLS
certificates. For authenticated brokers, set `MQTT_USERNAME` and `MQTT_PASSWORD`
in your terminal environment. Broker address and port can also be set through
`MQTT_BROKER` and `MQTT_PORT`. CLI flags take precedence over environment
variables, which take precedence over firmware configuration.

The Python MQTT client automatically retries connection failures and
resubscribes after reconnection. It uses its own client ID so it does not
disconnect the robot. The dashboard listens only on local loopback. Its Start
and Stop buttons publish the firmware's one-byte commands to the configured
`robot/command` topic without retain, using QoS 0 so commands are not queued
for reconnection. Broker permissions must allow publishing to that topic as
well as subscribing to telemetry, status and acknowledgements.

## Readings and controls

- Start and Stop buttons with robot acknowledgement and rejection feedback.
- Ground and wheel speeds, wrapping distance, ultrasonic clearance and rate.
- Left, Right and Barcode IR sensors, pitch, valid peak height, terrain, barcode,
  servo and encoders.
- Vehicle/controller states and all twelve fault bits, with WiFi/MQTT flags
  marked as advisory according to the firmware's fatal mask.
- A 15 / 30 / 60 second chart, with any numeric decoded field selectable.
- A searchable field explorer showing every field and the received packet hex.
- Connection and decoder events, rejected packet count, and stale data status.

The IR sensor panel, chart selector and field explorer use the physical roles:
Left (GP26), Right (GP27) and Barcode (GP28). Their telemetry keys remain
`ir_left`, `ir_centre` and `ir_right`, respectively; these keys also remain in
CSV exports and can be searched in the field explorer. Hover over an IR field
label to see its telemetry key.

Readings older than two seconds are marked stale. The last valid values remain
visible for inspection. MQTT retained messages are ignored because cached
readings cannot establish that the robot is currently reporting. Status and
acknowledgement packets are also decoded; their latest values are available in
the local `/api/state` endpoint. The robot link indicator comes from the latest
telemetry packet; the header describes the dashboard's broker connection.

**Pause view** freezes the displayed readings while collection continues.
Robot controls and command feedback remain live while the view is paused.
**Stop** halts the robot's current mission. Press Stop and wait for its accepted
acknowledgement before using **Start**, which resumes line following. Start
requires fresh telemetry and a connected broker. Stop remains available with
stale telemetry and can override a pending Start. The firmware's local E-stop,
fault latch and reset requirements still apply; Stop is a normal supervisory
command, and does not clear latched faults.

Start is also blocked while telemetry reports a fatal fault. For example,
`0x080` means WATCHDOG: a firmware task missed its heartbeat deadline, or
startup readiness timed out. This is separate from the physical E-stop;
the motion state "Emergency stop" alone does not identify the fault cause.
Reset cannot clear an ongoing fault. Current firmware records the first
watchdog trip and repeats the saved USB console line once per second while
the fault remains latched, so opening the serial monitor late still shows it:

```text
WATCHDOG: ms=2500 motion=5 line=5 imu=5 nav=45 scan=60 ready=3
```

The ages are milliseconds at the trip, in the named task order. Limits are
20 / 20 / 20 / 40 / 180 ms respectively. `ready=3` means line and IMU startup
are complete; bit0 is line-ready and bit1 is IMU-ready. Capture happens in
Safety, and printing happens in the lower-priority telemetry task. The
watchdog thresholds and reset policy are unchanged. Junctions ignored while
idle or faulted no longer flood the console, and navigation processes at
most four perception events per cycle so producers cannot keep it in an
unbounded queue-draining loop.

Boot, idle and fault states keep sampling the line sensors but do not queue
barcode or junction events for an inactive mission. A black patch during
startup no longer overrides the required 32 line calibration samples.
Queue faults also print a `QUEUE: sources=0x...` line once per second:
bit0 is command input, bit1 line/barcode, bit2 IMU and bit3 motion output.
The four-bit source mask preserves which queues overflowed until RESET.

A capture with `nav=60` and the other ages within their limits identifies a
Navigation heartbeat miss, with the 40-ms deadline unchanged. USB and WiFi
now run at priorities 9 and 10, below all robot tasks (1..8), so connection
and console work cannot preempt Navigation at priority 5. The diagnostic
timestamp and all control deadlines use kernel operating time in milliseconds
since boot, independent of changes to the wall clock. Physical timing under
radio and console load still needs verification on the robot.

After an outage or dashboard restart, confirm Stop again before Start. The
dashboard synchronizes the firmware's four-bit sequence with up to three Stop
attempts when needed; Start is never automatically retried. Sending is shown
separately from robot acceptance. If no acknowledgement arrives within two
seconds, the result is marked unconfirmed. These buttons require the built-in
robot decoder and are hidden when using `--decoder`.

**Export CSV** downloads the browser's rolling buffer (up to 1,200 samples,
about five minutes at 4 Hz). This is a live monitor, not a durable recorder;
restarting Python clears history. A new browser tab initially loads the current
server buffer. The chart leaves a gap across outages instead of joining them.

## Add readings using only Python

For the existing robot, no decoder code is needed: the dashboard already uses
`robot_protocol.decode`, including its strict packet validation and signed/scaled
conversions. The field explorer and chart options discover decoded fields
automatically.

For a different packet format, write a trusted Python module such as
`my_decoder.py`:

```python
import struct

TELEMETRY_TOPIC = "sensors/readings"  # optional; --topic takes precedence

def decode(payload: bytes) -> dict:
    if len(payload) != 8:
        raise ValueError("Expected two little-endian floats")
    temperature, voltage = struct.unpack("<ff", payload)
    return {"temperature_c": temperature, "voltage_v": voltage}
```

Then run:

```sh
python tools/dashboard/telemetry_dashboard.py --decoder my_decoder.py --broker 192.168.1.10
```

The launcher imports the module, subscribes to its topic, and calls `decode`
for each raw MQTT payload. The returned dictionary drives the dashboard: the
first three numeric fields become summary cards, all numeric fields become
chart choices, and all fields appear in the explorer. Robot-specific panels
are hidden for custom decoders. Return string keys and finite numbers,
booleans, strings or `None`; raise `ValueError` for invalid packets. Exceptions
are counted and displayed without stopping collection. Custom modules execute
as normal Python and should contain only code you trust.

## Verification

```sh
python -m unittest discover -s testing/dashboard -v
```

The suite covers firmware configuration discovery, packet validation, stale
and retained data, decoder failures, HTTP responses and the demo CLI. Command
checks cover sequence synchronization, acknowledgement matching, timeouts,
Start gating, Stop priority and local-origin request validation. With the MQTT
dependency installed and `mosquitto` on PATH, it also runs a local broker
integration test for command delivery, acknowledgements, subscription,
malformed packets and reconnection without command replay.
