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

The robot must be built with `NET=lwip MQTT=1`, and its configured broker must
be reachable from the computer running Python. The dashboard connects to an
existing MQTT broker; it does not start one or flash the robot.

## Preview without hardware

```sh
python tools/dashboard/telemetry_dashboard.py --demo
```

Demo mode needs only Python 3.10+ and the repository files. It creates valid
28-byte packets and feeds them through the same decoder as real telemetry.
The dashboard clearly labels simulated readings.

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
disconnect the robot. The dashboard listens only on local loopback and only
subscribes: it never publishes robot commands.

## Readings and controls

- Ground and wheel speeds, wrapping distance, ultrasonic clearance and rate.
- Line sensors, pitch, valid peak height, terrain, barcode, servo and encoders.
- Vehicle/controller states and all twelve fault bits, with WiFi/MQTT flags
  marked as advisory according to the firmware's fatal mask.
- A 15 / 30 / 60 second chart, with any numeric decoded field selectable.
- A searchable field explorer showing every field and the received packet hex.
- Connection and decoder events, rejected packet count, and stale data status.

Readings older than two seconds are marked stale. The last valid values remain
visible for inspection. MQTT retained messages are ignored because cached
readings cannot establish that the robot is currently reporting. Status and
acknowledgement packets are also decoded; their latest values are available in
the local `/api/state` endpoint. The robot link indicator comes from the latest
telemetry packet; the header describes the dashboard's broker connection.

**Pause view** freezes the displayed readings while collection continues.
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
and retained data, decoder failures, HTTP responses and the demo CLI. With the
MQTT dependency installed and `mosquitto` on PATH, it also runs a local broker
integration test for subscription, malformed packets and broker reconnection.
