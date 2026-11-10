#!/usr/bin/env python3
"""Run the local Motor-Wheel MQTT dashboard. Use --demo without a broker."""
from __future__ import annotations

import argparse
from collections import deque
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit
import uuid
import webbrowser

from robot_protocol import FIELDS, decode, encode_command

ROOT = Path(__file__).resolve().parents[2]
HTML = Path(__file__).with_suffix('.html')
HISTORY_LIMIT = 1200
COMMAND_TIMEOUT = 2
FAULT_NAMES = ('Stall', 'Encoder', 'IMU', 'No echo', 'WiFi', 'Impact',
               'E-stop', 'Watchdog', 'Range', 'Timeout', 'Queue', 'MQTT')


def read_defines(path: Path) -> dict[str, str]:
    """Read only literal defines, never execute a configuration file."""
    if not path.exists():
        return {}
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', path.read_text(), flags=re.S)
    return {name: string or number for name, string, number in re.findall(
        r'^\s*#define\s+(\w+)\s+(?:"([^"\n]*)"|(\d+))\s*$', source, re.M)}


def load_decoder(path: Path | None):
    if path is None:
        return decode, None
    spec = importlib.util.spec_from_file_location('dashboard_decoder', path)
    if spec is None or spec.loader is None:
        raise ValueError(f'Cannot load Python decoder: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, 'decode', None)):
        raise ValueError('Custom decoder must define decode(payload: bytes) -> dict')
    return module.decode, getattr(module, 'TELEMETRY_TOPIC', None)


class Telemetry:
    """Thread-safe bounded state shared by MQTT and browser requests."""

    def __init__(self, decoder=decode, *, broker='localhost:1883', demo=False,
                 topics=None, custom=False):
        self.decoder = decoder
        self.lock = threading.Lock()
        self.history = deque(maxlen=HISTORY_LIMIT)
        self.events = deque(maxlen=30)
        self.started = time.time()
        self.received = 0
        self.rejected = 0
        self.retained = 0
        self.connection = 'connecting'
        self.connection_detail = 'Connecting to MQTT broker'
        self.broker = broker
        self.demo = demo
        self.custom = custom
        self.topics = topics or {}
        self.status = None
        self.ack = None
        self.publish_command = None
        self.next_sequence = 0
        self.stop_confirmed = False
        self.command = None

    def controls(self):
        # Call with the state lock held. Stop remains available with stale telemetry
        # and while Start is awaiting an acknowledgement.
        now = time.time()
        if self.command and self.command['status'] == 'pending' and now - self.command['time'] > COMMAND_TIMEOUT:
            self.command['status'] = 'unconfirmed'
            self.command['message'] = 'No robot acknowledgement. Command outcome unknown; send Stop before retrying.'
            self.stop_confirmed = False
            self.event(self.command['message'], 'warning')
        fresh = bool(self.history and now - self.history[-1]['time'] <= 2)
        link = self.history[-1]['values'].get('link', 0) if self.history else 0
        values = self.history[-1]['values'] if self.history else {}
        faults = values.get('faults', 0)
        fatal = faults & 0x7ef if isinstance(faults, int) else 0
        if not fresh or not isinstance(link, int) or not link & 2:
            self.stop_confirmed = False
        available = (not self.demo and not self.custom and self.connection == 'connected'
                     and self.publish_command is not None and bool(self.topics.get('command')))
        pending = bool(self.command and self.command['status'] == 'pending')
        can_start = available and fresh and self.stop_confirmed and not pending and not fatal
        if self.demo:
            detail = 'Demo mode: robot controls are disabled.'
        elif self.custom:
            detail = 'Robot controls are unavailable with a custom decoder.'
        elif not available:
            detail = 'Connect to MQTT to control the robot.'
        elif not fresh:
            detail = 'Waiting for live robot telemetry. Stop is available.'
        elif fatal:
            names = ', '.join(name for bit, name in enumerate(FAULT_NAMES) if fatal & (1 << bit))
            detail = f'Start blocked: {names} fault (0x{fatal:03X}). Clear the cause and press local RESET (GP22).'
            if fatal & 0x080:
                detail += ' Watchdog means a task missed its deadline or startup readiness timed out; check USB WATCHDOG diagnostics.'
        elif pending:
            detail = 'Waiting for the robot acknowledgement. Stop remains available.'
        elif not self.stop_confirmed:
            detail = 'Press Stop and wait for confirmation before Start.'
        else:
            detail = 'Ready. Start resumes line following; Stop halts the robot.'
        return {'can_start': can_start, 'can_stop': available, 'detail': detail,
                'command': dict(self.command) if self.command else None}

    def _publish(self, name, sequence, attempts=0):
        # QoS 0 avoids queued/replayed commands after reconnecting. Never retain.
        try:
            result = self.publish_command(self.topics['command'], encode_command(name, sequence),
                                          qos=0, retain=False)
            if result.rc != 0:
                raise ValueError('MQTT publish failed')
        except (OSError, RuntimeError, ValueError) as error:
            self.stop_confirmed = False
            raise ValueError('MQTT could not send the command; check the broker connection.') from error
        self.next_sequence = (sequence + 1) % 16
        self.command = {'name': name, 'sequence': sequence, 'time': time.time(),
                        'status': 'pending', 'attempts': attempts,
                        'message': f'{name.title()} sent; waiting for robot acknowledgement…'}
        self.event(f'{name.title()} sent (sequence {sequence})')

    def send_command(self, name):
        if name not in ('start', 'stop'):
            raise ValueError('Command must be start or stop.')
        with self.lock:
            controls = self.controls()
            if not controls[f'can_{name}']:
                raise ValueError(controls['detail'])
            if name == 'stop':
                self.stop_confirmed = False
            self._publish(name, self.next_sequence)
            return dict(self.command)

    def event(self, message, level='info'):
        # Call with the state lock held.
        self.events.appendleft({'time': time.time(), 'message': message, 'level': level})

    def set_connection(self, state, detail):
        with self.lock:
            if (state, detail) != (self.connection, self.connection_detail):
                self.event(detail, 'error' if state == 'disconnected' else 'info')
            self.connection = state
            self.connection_detail = detail
            if state != 'connected':
                self.stop_confirmed = False
                if self.command and self.command['status'] == 'pending':
                    self.command['status'] = 'unconfirmed'
                    self.command['message'] = 'MQTT disconnected. Command outcome unknown; send Stop after reconnecting.'

    def ingest(self, topic: str, payload: bytes, retained=False):
        try:
            # Robot publishers never retain. Cached packets cannot establish freshness.
            if retained:
                with self.lock:
                    self.retained += 1
                    self.event(f'Ignored retained message on {topic}', 'warning')
                return
            if topic == self.topics.get('telemetry'):
                values = self.decoder(payload)
                if not isinstance(values, dict) or not values:
                    raise ValueError('Decoder must return a nonempty dict')
                if not all(isinstance(k, str) and isinstance(v, (int, float, bool, str, type(None)))
                           and (not isinstance(v, float) or math.isfinite(v))
                           for k, v in values.items()):
                    raise ValueError('Decoded fields must be finite numbers, booleans, strings or null')
                # Reject values that cannot be sent as JSON, before mutating state.
                json.dumps(values, allow_nan=False)
                with self.lock:
                    self.received += 1
                    sample = {'id': self.received, 'time': time.time(), 'values': values,
                              'hex': payload.hex()}
                    if not self.history:
                        self.event('First telemetry packet decoded')
                    previous = self.history[-1]['values'].get('faults') if self.history else None
                    if values.get('faults') != previous and values.get('faults') is not None:
                        mask = values['faults']
                        self.event(f'Fault mask changed to {mask}', 'warning' if mask else 'info')
                    self.history.append(sample)
            elif topic == self.topics.get('status'):
                if len(payload) != 1 or payload[0] & 0xF0 or payload[0] & 3 != 1:
                    raise ValueError('Invalid status packet')
                with self.lock:
                    self.status = {'time': time.time(), 'wifi': bool(payload[0] & 4),
                                   'mqtt': bool(payload[0] & 8), 'version': payload[0] & 3}
            elif topic == self.topics.get('ack'):
                if len(payload) != 1:
                    raise ValueError('Acknowledgement requires one byte')
                with self.lock:
                    self.ack = {'time': time.time(), 'command': payload[0] & 3,
                                'sequence': (payload[0] >> 2) & 15,
                                'accepted': bool(payload[0] & 64),
                                'malformed': bool(payload[0] & 128)}
                    self.event('Observed command acknowledgement')
                    command = self.command
                    if (command and command['status'] == 'pending'
                            and time.time() - command['time'] <= COMMAND_TIMEOUT
                            and self.ack['sequence'] == command['sequence']
                            and self.ack['command'] == (1 if command['name'] == 'start' else 0)):
                        if (command['name'] == 'stop' and not self.ack['accepted']
                                and not self.ack['malformed'] and command['attempts'] < 2):
                            # The firmware keeps its sequence across dashboard restarts.
                            # Three STOP probes spaced by seven cover every modulo-16
                            # starting point without ever retrying START.
                            try:
                                self._publish('stop', (command['sequence'] + 7) % 16,
                                              command['attempts'] + 1)
                            except ValueError as error:
                                command['status'] = 'unconfirmed'
                                command['message'] = str(error)
                        else:
                            accepted = self.ack['accepted'] and not self.ack['malformed']
                            command['status'] = 'accepted' if accepted else 'rejected'
                            command['message'] = (f'{command["name"].title()} accepted by robot.' if accepted else
                                                  f'{command["name"].title()} rejected by robot. Check robot state and local reset.')
                            self.stop_confirmed = accepted and command['name'] == 'stop'
                            if not accepted and command['name'] == 'start':
                                command['message'] += ' ' + self.controls()['detail']
                            self.event(command['message'], 'info' if accepted else 'warning')
                    elif self.ack['accepted'] and not self.ack['malformed']:
                        # Track other operators without treating their ACK as our result.
                        if not command or command['status'] != 'pending':
                            self.next_sequence = (self.ack['sequence'] + 1) % 16
                        self.stop_confirmed = False
        except Exception as error:
            # A user decoder failure must not kill Paho's network thread.
            with self.lock:
                self.rejected += 1
                self.event(f'Rejected {topic}: {type(error).__name__}: {error}', 'error')

    def snapshot(self, since=0):
        with self.lock:
            latest = self.history[-1] if self.history else None
            now = time.time()
            age = now - latest['time'] if latest else None
            recent = [s for s in self.history if now - s['time'] < 5]
            rate = ((len(recent) - 1) / (recent[-1]['time'] - recent[0]['time'])
                    if len(recent) > 1 and recent[-1]['time'] > recent[0]['time'] else 0)
            return {'server_time': now, 'started': self.started, 'demo': self.demo,
                    'custom': self.custom, 'broker': self.broker, 'topics': self.topics,
                    'connection': self.connection, 'connection_detail': self.connection_detail,
                    'age': age, 'stale': age is None or age > 2,
                    'received': self.received, 'rejected': self.rejected,
                    'retained': self.retained, 'rate': round(rate, 1),
                    'latest': latest, 'samples': [s for s in self.history if s['id'] > since],
                    'controls': self.controls(),
                    'events': list(self.events), 'status': self.status, 'ack': self.ack}


def start_mqtt(state, args):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        raise SystemExit('MQTT dependency missing. Run:\n'
                         '  python3 -m pip install -r tools/dashboard/requirements-dashboard.txt\n'
                         'Or preview without dependencies using --demo.') from None
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=f'mw-dashboard-{uuid.uuid4().hex[:8]}',
                         protocol=mqtt.MQTTv311)
    if args.username:
        client.username_pw_set(args.username, os.environ.get('MQTT_PASSWORD'))
    if args.tls:
        client.tls_set(ca_certs=str(args.ca) if args.ca else None)
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            state.set_connection('disconnected', f'Broker refused connection: {reason_code}')
            return
        state.set_connection('subscribing', 'Broker connected; subscribing to telemetry')
        result, _ = client.subscribe([(topic, 0) for name, topic in state.topics.items()
                                      if name != 'command'])
        if result != mqtt.MQTT_ERR_SUCCESS:
            state.set_connection('disconnected', f'Subscription failed: {result}')

    def on_subscribe(client, userdata, mid, reason_codes, properties):
        if not reason_codes or any(code.is_failure for code in reason_codes):
            state.set_connection('disconnected', 'Broker rejected subscription; check topic permissions')
        else:
            state.set_connection('connected', 'Subscribed; waiting for live telemetry')

    def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
        state.set_connection('disconnected', 'MQTT disconnected; reconnecting automatically')

    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_disconnect = on_disconnect
    client.on_connect_fail = lambda client, userdata: state.set_connection(
        'disconnected', 'Cannot reach broker; retrying automatically')
    client.on_message = lambda client, userdata, msg: state.ingest(
        msg.topic, msg.payload, msg.retain)
    state.publish_command = client.publish
    client.connect_async(args.broker, args.broker_port, keepalive=30)
    client.loop_start()
    return client


def demo_loop(state, stop):
    """Generate wire packets and send them through the real decoder."""
    state.set_connection('connected', 'Simulated telemetry · no robot connected')
    start = time.monotonic()
    while not stop.is_set():
        t = time.monotonic() - start
        hump = max(0, math.sin(t / 6))
        values = dict(version=1, vehicle=2, distance_mm=int(t * 240) % 4096,
                      speed_mm_second=int(240 + 35 * math.sin(t)), reverse=0,
                      motion_mode=1, tick_5ms=int(t * 200) % 128, faults=0,
                      ir_left=int(1100 + 400 * math.sin(t)), ir_centre=3200,
                      ir_right=int(1100 - 400 * math.sin(t)), terrain=2 if hump > .3 else 1,
                      pitch_offset=int(64 + 20 * hump), peak_mm=int(35 * hump),
                      range_cm=int(100 + 40 * math.sin(t / 4)), servo_3deg=30,
                      barcode_event=6, line_error_offset=int(127 + 24 * math.sin(t)),
                      line_state=1, obstacle_state=0, link=3, on_hump=int(hump > .3),
                      height_valid=1, encoder_left=int(t * 600) % (1 << 32),
                      encoder_right=int(t * 600) % (1 << 32),
                      wheel_left_offset=int(752 + 42 * math.sin(t)),
                      wheel_right_offset=int(752 + 28 * math.cos(t)))
        word, offset = 0, 0
        for name, width in FIELDS:
            word |= values[name] << offset
            offset += width
        state.ingest(state.topics['telemetry'], word.to_bytes(28, 'little'))
        stop.wait(.25)


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if urlsplit(self.path).path != '/api/command':
                self.send_error(404)
                return
            host = self.headers.get('Host', '')
            allowed = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
            if host not in allowed or self.headers.get('Origin', f'http://{host}') != f'http://{host}':
                self.send_error(403, 'Commands require the local dashboard origin')
                return
            if self.headers.get('Content-Type', '').lower() != 'application/json':
                self.send_error(415, 'Commands require application/json')
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 128:
                    raise ValueError('Invalid command body length')
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict) or request.get('command') not in ('start', 'stop'):
                    raise ValueError('Command must be start or stop')
            except (ValueError, UnicodeDecodeError) as error:
                self.respond(400, {'error': str(error)})
                return
            try:
                command = state.send_command(request['command'])
            except ValueError as error:
                self.respond(409, {'error': str(error)})
                return
            self.respond(202, {'command': command})

        def respond(self, status, payload):
            self.send_body(status, json.dumps(payload, allow_nan=False).encode(), 'application/json')

        def do_GET(self):
            parsed = urlsplit(self.path)
            if parsed.path == '/':
                body, content_type = HTML.read_bytes(), 'text/html; charset=utf-8'
            elif parsed.path == '/api/state':
                try:
                    since = max(0, int(parse_qs(parsed.query).get('since', ['0'])[0]))
                except ValueError:
                    self.send_error(400, 'since must be an integer')
                    return
                body = json.dumps(state.snapshot(since), allow_nan=False).encode()
                content_type = 'application/json'
            elif parsed.path == '/favicon.ico':
                self.send_response(204)
                self.end_headers()
                return
            else:
                self.send_error(404)
                return
            self.send_body(200, body, content_type)

        def send_body(self, status, body, content_type):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'unsafe-inline'; "
                             "style-src 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; "
                             "frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, format, *args):
            pass

    return Handler


def main():
    config = read_defines(ROOT / 'config/wifi_secrets.h')
    topics = read_defines(ROOT / 'app_program/subsystem1/telemetry.h')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--broker', default=os.environ.get('MQTT_BROKER', config.get('MQTT_BROKER_IP', 'localhost')))
    parser.add_argument('--broker-port', type=int, default=int(os.environ.get(
        'MQTT_PORT', config.get('MQTT_BROKER_PORT', '1883'))))
    parser.add_argument('--port', type=int, default=8765, help='Local dashboard port (default: 8765)')
    parser.add_argument('--username', default=os.environ.get('MQTT_USERNAME'))
    parser.add_argument('--tls', action='store_true', help='Verify broker TLS certificate')
    parser.add_argument('--ca', type=Path, help='Custom CA certificate (requires --tls)')
    parser.add_argument('--topic', help='Override the telemetry topic')
    parser.add_argument('--decoder', type=Path, help='Trusted Python module defining decode(payload)')
    parser.add_argument('--demo', action='store_true', help='Simulated readings; no dependencies required')
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    if args.ca and not args.tls:
        parser.error('--ca requires --tls')
    if args.demo and args.decoder:
        parser.error('--demo uses the built-in robot decoder; omit --decoder')
    if not 0 <= args.port <= 65535 or not 1 <= args.broker_port <= 65535:
        parser.error('Port must be in range 1..65535 (dashboard --port also accepts 0)')
    try:
        decoder, decoder_topic = load_decoder(args.decoder)
    except Exception as error:
        parser.error(f'Cannot load decoder: {error}')
    selected_topics = {
        'telemetry': args.topic or decoder_topic or topics.get('TELEMETRY_TOPIC_TELEMETRY', 'robot/telemetry'),
        'status': topics.get('TELEMETRY_TOPIC_STATUS', 'robot/status'),
        'ack': topics.get('TELEMETRY_TOPIC_ACK', 'robot/ack'),
        'command': topics.get('TELEMETRY_TOPIC_COMMAND', 'robot/command'),
    }
    if any(not isinstance(topic, str) or not topic or '+' in topic or '#' in topic
           for topic in selected_topics.values()) or len(set(selected_topics.values())) != 4:
        parser.error('Use distinct, nonempty exact topics without MQTT wildcards')
    state = Telemetry(decoder, broker=f'{args.broker}:{args.broker_port}', demo=args.demo,
                      topics=selected_topics, custom=args.decoder is not None)
    stop = threading.Event()
    client = None
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(state))
    except OSError as error:
        parser.error(f'Cannot open dashboard: {error}. Try --port 8766.')
    try:
        if args.demo:
            threading.Thread(target=demo_loop, args=(state, stop), daemon=True).start()
        else:
            client = start_mqtt(state, args)
        url = f'http://127.0.0.1:{server.server_port}'
        print(f'Motor-Wheel dashboard: {url}', flush=True)
        print('DEMO · simulated data' if args.demo else f'MQTT: {state.broker} · {selected_topics["telemetry"]}', flush=True)
        print('Press Ctrl+C to stop.', flush=True)
        if not args.no_browser:
            threading.Timer(.4, lambda: webbrowser.open(url)).start()
        # SIGTERM follows the same cleanup path as Ctrl+C.
        def terminate(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, terminate)
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if client:
            client.disconnect()
            client.loop_stop()
        server.server_close()


if __name__ == '__main__':
    main()
