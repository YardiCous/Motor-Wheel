"""Dashboard checks; run with python -m unittest discover -s testing/dashboard -v."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))
import telemetry_dashboard as dashboard

VECTOR = bytes.fromhex('fa7ffaf71f090000c0ffa76d05b27c00eaffffffff0500000018a01f')
TOPICS = {'telemetry': 'robot/telemetry', 'status': 'robot/status', 'ack': 'robot/ack',
          'command': 'robot/command'}


def with_field(packet, offset, width, value):
    word = int.from_bytes(packet, 'little')
    mask = ((1 << width) - 1) << offset
    return ((word & ~mask) | (value << offset)).to_bytes(28, 'little')


class ProtocolTests(unittest.TestCase):
    def test_firmware_sensor_values_in_both_decoders(self):
        for relative in ['tools/robot_protocol.py', 'tools/dashboard/robot_protocol.py']:
            spec = importlib.util.spec_from_file_location('protocol_under_test', ROOT / relative)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.assertNotIn('version', module.decode(VECTOR))
            self.assertEqual(module.decode(bytes(28))['vehicle'], 0)
            for bit in (221, 222, 223):
                with self.subTest(decoder=relative, padding_bit=bit):
                    with self.assertRaises(ValueError):
                        module.decode(with_field(VECTOR, bit, 1, 1))
            for distance in [0, 1, 2, 400]:
                for obstacle in range(8):
                    with self.subTest(decoder=relative, range_cm=distance, obstacle=obstacle):
                        packet = with_field(VECTOR, 101, 9, distance)
                        packet = with_field(packet, 130, 3, obstacle)
                        values = module.decode(packet)
                        self.assertEqual(values['range_cm'], distance)
                        self.assertEqual(values['obstacle_state'], obstacle)
                        self.assertEqual(values['pitch_deg'], -5.0)
            for offset, width, value in [(101, 9, 401), (119, 8, 255)]:
                with self.subTest(decoder=relative, invalid=value):
                    with self.assertRaises(ValueError):
                        module.decode(with_field(VECTOR, offset, width, value))


def until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError('Timed out waiting for asynchronous state')


class StateTests(unittest.TestCase):
    def setUp(self):
        self.state = dashboard.Telemetry(topics=TOPICS)

    def test_firmware_config_and_comments(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'config.h'
            path.write_text('/* #define MQTT_BROKER_PORT 7 */\n'
                            '#define MQTT_BROKER_IP "192.168.1.8" // comment\n'
                            '#define MQTT_BROKER_PORT 1884\n'
                            '#define NOT_A_LITERAL (1 + 2)\n')
            self.assertEqual(dashboard.read_defines(path), {
                'MQTT_BROKER_IP': '192.168.1.8', 'MQTT_BROKER_PORT': '1884'})
            self.assertEqual(dashboard.read_defines(Path(temp) / 'missing'), {})
        self.assertEqual(dashboard.read_defines(
            ROOT / 'app_program/subsystem1/telemetry.h')['TELEMETRY_TOPIC_TELEMETRY'],
            TOPICS['telemetry'])

    def test_binary_decode_and_incremental_history(self):
        self.state.ingest(TOPICS['telemetry'], VECTOR)
        snapshot = self.state.snapshot()
        self.assertEqual(snapshot['latest']['values']['distance_mm'], 4095)
        self.assertEqual(snapshot['latest']['values']['pitch_deg'], -5.0)
        self.assertEqual(snapshot['latest']['hex'], VECTOR.hex())
        self.assertEqual(snapshot['received'], 1)
        self.assertFalse(snapshot['stale'])
        self.assertEqual(self.state.snapshot(since=1)['samples'], [])
        self.state.ingest(TOPICS['telemetry'], VECTOR)
        self.assertEqual([s['id'] for s in self.state.snapshot(since=1)['samples']], [2])

    def test_near_echo_and_body_scan_keep_dashboard_live(self):
        near = with_field(VECTOR, 101, 9, 1)
        body = with_field(near, 130, 3, 7)
        for packet in [VECTOR, near, body]:
            self.state.ingest(TOPICS['telemetry'], packet)
        snapshot = self.state.snapshot()
        self.assertEqual(snapshot['received'], 3)
        self.assertEqual(snapshot['rejected'], 0)
        self.assertFalse(snapshot['stale'])
        self.assertEqual(snapshot['latest']['values']['range_cm'], 1)
        self.assertEqual(snapshot['latest']['values']['obstacle_state'], 7)
        self.state.ingest(TOPICS['telemetry'], VECTOR)
        self.assertEqual(self.state.snapshot()['received'], 4)
        self.assertEqual(self.state.snapshot()['latest']['values']['range_cm'], 400)

    def test_invalid_and_retained_packets_do_not_refresh_readings(self):
        self.state.ingest(TOPICS['telemetry'], VECTOR)
        first_time = self.state.snapshot()['latest']['time']
        for packet in [b'', with_field(VECTOR, 25, 3, 7), VECTOR[:-1],
                       VECTOR[:-1] + bytes([VECTOR[-1] | 128])]:
            self.state.ingest(TOPICS['telemetry'], packet)
        self.state.ingest(TOPICS['telemetry'], VECTOR, retained=True)
        self.state.ingest(TOPICS['status'], b'\x03', retained=True)
        snapshot = self.state.snapshot()
        self.assertEqual(snapshot['received'], 1)
        self.assertEqual(snapshot['rejected'], 4)
        self.assertEqual(snapshot['retained'], 2)
        self.assertEqual(snapshot['latest']['time'], first_time)
        self.assertIsNone(snapshot['status'])

    def test_staleness_history_bound_and_events_bound(self):
        self.assertTrue(self.state.snapshot()['stale'])
        for _ in range(dashboard.HISTORY_LIMIT + 2):
            self.state.ingest(TOPICS['telemetry'], VECTOR)
        self.state.history[-1]['time'] -= 3
        self.assertTrue(self.state.snapshot()['stale'])
        self.assertEqual(len(self.state.snapshot()['samples']), dashboard.HISTORY_LIMIT)
        for i in range(40):
            self.state.set_connection('disconnected', f'Retry {i}')
        self.assertEqual(len(self.state.snapshot()['events']), 30)

    def test_status_ack_and_invalid_status(self):
        for value in range(4):
            self.state.ingest(TOPICS['status'], bytes([value]))
            status = self.state.snapshot()['status']
            self.assertEqual(status['wifi'], bool(value & 1))
            self.assertEqual(status['mqtt'], bool(value & 2))
            self.assertNotIn('version', status)
        self.state.ingest(TOPICS['status'], b'\x03')
        self.state.ingest(TOPICS['ack'], bytes([1 | (9 << 2) | 64]))
        snapshot = self.state.snapshot()
        self.assertTrue(snapshot['status']['wifi'])
        self.assertTrue(snapshot['status']['mqtt'])
        self.assertEqual(snapshot['ack']['sequence'], 9)
        self.assertEqual(snapshot['ack']['command'], 1)
        self.assertTrue(snapshot['ack']['accepted'])
        for packet in [b'', b'\x04', b'\xfd', b'\x03\x00']:
            self.state.ingest(TOPICS['status'], packet)
        self.state.ingest(TOPICS['ack'], b'\x00\x00')
        self.assertEqual(self.state.snapshot()['rejected'], 5)
        self.assertTrue(self.state.snapshot()['status']['wifi'])

    def test_custom_decoder_and_failure_isolation(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'decoder.py'
            path.write_text('TELEMETRY_TOPIC = "sensor/readings"\n'
                            'def decode(payload):\n'
                            '    return {"temperature_c": payload[0], "valid": True}\n')
            decoder, topic = dashboard.load_decoder(path)
            self.assertEqual(topic, 'sensor/readings')
            state = dashboard.Telemetry(decoder, topics={'telemetry': topic}, custom=True)
            state.ingest(topic, b'\x19')
            self.assertEqual(state.snapshot()['latest']['values']['temperature_c'], 25)
            state.ingest(topic, b'')  # IndexError in user code must not kill collection.
            self.assertEqual(state.snapshot()['rejected'], 1)
            self.assertEqual(state.snapshot()['received'], 1)
        for invalid in [None, {}, {'v': float('nan')}, {'v': float('inf')},
                        {1: 2}, {'nested': {'v': 2}}]:
            state = dashboard.Telemetry(lambda _: invalid, topics=TOPICS)
            state.ingest(TOPICS['telemetry'], b'anything')
            self.assertEqual(state.snapshot()['rejected'], 1)
            self.assertEqual(state.snapshot()['received'], 0)
        state = dashboard.Telemetry(lambda _: {'link': 'online'}, topics=TOPICS, custom=True)
        state.ingest(TOPICS['telemetry'], b'custom')
        self.assertEqual(state.snapshot()['latest']['values']['link'], 'online')
        self.assertFalse(state.snapshot()['controls']['can_start'])


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.state = dashboard.Telemetry(topics=TOPICS)
        self.state.publish_command = self.publish
        self.state.set_connection('connected', 'Test broker')
        self.state.ingest(TOPICS['telemetry'], with_field(VECTOR, 35, 12, 0))

    def publish(self, topic, payload, **options):
        self.sent.append((topic, payload, options))
        return SimpleNamespace(rc=0)

    def acknowledge(self, command, accepted=True, **options):
        value = dashboard.encode_command(command['name'], command['sequence'])[0]
        self.state.ingest(TOPICS['ack'], bytes([value | (64 if accepted else 0)]), **options)

    def test_stop_then_start_and_sequence_wrap(self):
        with self.assertRaises(ValueError):
            self.state.send_command('start')
        self.state.next_sequence = 15
        stop = self.state.send_command('stop')
        self.assertFalse(self.state.snapshot()['controls']['can_start'])
        self.acknowledge(stop, retained=True)
        self.assertFalse(self.state.snapshot()['controls']['can_start'])
        self.acknowledge(stop)
        self.assertTrue(self.state.snapshot()['controls']['can_start'])
        start = self.state.send_command('start')
        self.assertEqual(start['sequence'], 0)
        self.acknowledge(start)
        self.assertEqual(self.state.snapshot()['controls']['command']['status'], 'accepted')
        self.assertEqual(self.sent, [
            (TOPICS['command'], b'\x3c', {'qos': 0, 'retain': False}),
            (TOPICS['command'], b'\x01', {'qos': 0, 'retain': False})])

    def test_stop_resynchronizes_every_firmware_sequence(self):
        for last_sequence in range(16):
            with self.subTest(last_sequence=last_sequence):
                self.setUp()
                command = self.state.send_command('stop')
                for _ in range(3):
                    accepted = 1 <= (command['sequence'] - last_sequence) % 16 <= 7
                    self.acknowledge(command, accepted=accepted)
                    command = self.state.snapshot()['controls']['command']
                    if accepted:
                        break
                self.assertEqual(command['status'], 'accepted')
                self.assertTrue(self.state.snapshot()['controls']['can_start'])
                self.assertTrue(all(payload[0] & 3 == 0 for _, payload, _ in self.sent))

    def test_stop_preempts_pending_start_and_start_is_never_retried(self):
        self.acknowledge(self.state.send_command('stop'))
        start = self.state.send_command('start')
        with self.assertRaises(ValueError):
            self.state.send_command('start')
        stop = self.state.send_command('stop')
        self.acknowledge(start)
        self.assertEqual(self.state.snapshot()['controls']['command']['status'], 'pending')
        self.acknowledge(stop)
        self.assertTrue(self.state.snapshot()['controls']['can_start'])
        start = self.state.send_command('start')
        self.assertEqual(start['sequence'], (stop['sequence'] + 1) % 16)
        self.acknowledge(start, accepted=False)
        self.assertEqual(len(self.sent), 4)
        self.assertEqual(self.state.snapshot()['controls']['command']['status'], 'rejected')

    def test_disconnect_staleness_timeout_and_publish_failure(self):
        self.acknowledge(self.state.send_command('stop'))
        self.state.history[-1]['time'] -= 3
        self.assertFalse(self.state.snapshot()['controls']['can_start'])
        self.assertTrue(self.state.snapshot()['controls']['can_stop'])
        self.state.send_command('stop')
        self.state.command['time'] -= dashboard.COMMAND_TIMEOUT + 1
        self.assertEqual(self.state.snapshot()['controls']['command']['status'], 'unconfirmed')
        self.state.set_connection('disconnected', 'Test outage')
        with self.assertRaises(ValueError):
            self.state.send_command('stop')
        self.state.set_connection('connected', 'Reconnected')
        self.state.publish_command = lambda *args, **kwargs: SimpleNamespace(rc=4)
        with self.assertRaises(ValueError):
            self.state.send_command('stop')
        self.assertFalse(self.state.snapshot()['controls']['can_start'])

    def test_watchdog_blocks_start_but_not_stop(self):
        self.acknowledge(self.state.send_command('stop'))
        # faults starts at bit 35; watchdog is bit 7 of that field.
        self.state.ingest(TOPICS['telemetry'], with_field(VECTOR, 35, 12, 0x080))
        controls = self.state.snapshot()['controls']
        self.assertFalse(controls['can_start'])
        self.assertTrue(controls['can_stop'])
        self.assertIn('Watchdog', controls['detail'])
        self.assertIn('0x080', controls['detail'])
        with self.assertRaisesRegex(ValueError, 'Watchdog'):
            self.state.send_command('start')
        self.state.send_command('stop')

    def test_network_advisories_do_not_block_start(self):
        self.state.ingest(TOPICS['telemetry'], with_field(VECTOR, 35, 12, 0x810))
        self.acknowledge(self.state.send_command('stop'))
        self.assertTrue(self.state.snapshot()['controls']['can_start'])

    def test_unrelated_ack_demo_custom_and_unknown_commands(self):
        stop = self.state.send_command('stop')
        self.acknowledge(dict(stop, sequence=(stop['sequence'] + 1) % 16))
        self.assertFalse(self.state.snapshot()['controls']['can_start'])
        self.acknowledge(stop)
        for mode in ['demo', 'custom']:
            setattr(self.state, mode, True)
            for name in ['start', 'stop']:
                with self.assertRaises(ValueError):
                    self.state.send_command(name)
            setattr(self.state, mode, False)
        with self.assertRaises(ValueError):
            self.state.send_command('estop')


class HTTPTests(unittest.TestCase):
    def test_command_api_validation_and_delivery(self):
        state = dashboard.Telemetry(topics=TOPICS)
        sent = []
        state.publish_command = lambda *args, **kwargs: sent.append(args) or SimpleNamespace(rc=0)
        state.set_connection('connected', 'Test broker')
        server = dashboard.ThreadingHTTPServer(('127.0.0.1', 0), dashboard.make_handler(state))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        url = f'http://127.0.0.1:{server.server_port}/api/command'
        try:
            cases = [(b'{"command":"start"}', {}, 409),
                     (b'{"command":"reset"}', {}, 400),
                     (b'invalid', {}, 400),
                     (b'[]', {}, 400),
                     (b' ' * 129, {}, 400),
                     (b'{"command":"stop"}', {'Origin': 'https://example.com'}, 403),
                     (b'{"command":"stop"}', {'Host': 'example.com'}, 403),
                     (b'{"command":"stop"}', {'Content-Type': 'text/plain'}, 415)]
            for body, headers, status in cases:
                with self.subTest(status=status, body=body):
                    request = Request(url, data=body, headers={'Content-Type': 'application/json', **headers})
                    with self.assertRaises(HTTPError) as error:
                        urlopen(request)
                    self.assertEqual(error.exception.code, status)
                    error.exception.close()
            self.assertEqual(sent, [])
            request = Request(url, data=b'{"command":"stop"}', headers={'Content-Type': 'application/json'})
            with urlopen(request) as response:
                self.assertEqual(response.status, 202)
                self.assertEqual(json.load(response)['command']['status'], 'pending')
            self.assertEqual(sent, [(TOPICS['command'], b'\x00')])
        finally:
            server.shutdown()
            server.server_close()
            worker.join()

    def test_api_and_dashboard_routes(self):
        state = dashboard.Telemetry(topics=TOPICS)
        state.ingest(TOPICS['telemetry'], VECTOR)
        server = dashboard.ThreadingHTTPServer(('127.0.0.1', 0), dashboard.make_handler(state))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        url = f'http://127.0.0.1:{server.server_port}'
        try:
            with urlopen(url) as response:
                self.assertEqual(response.status, 200)
                self.assertIn(b'Motion over time', response.read())
                self.assertIn("frame-ancestors 'none'", response.headers['Content-Security-Policy'])
            with urlopen(url + '/api/state') as response:
                payload = json.load(response)
                self.assertEqual(payload['received'], 1)
            with urlopen(url + '/api/state?since=1') as response:
                self.assertEqual(json.load(response)['samples'], [])
            for path, status in [('/api/state?since=nope', 400), ('/config/wifi_secrets.h', 404)]:
                with self.assertRaises(HTTPError) as error:
                    urlopen(url + path)
                self.assertEqual(error.exception.code, status)
                error.exception.close()
        finally:
            server.shutdown()
            server.server_close()
            worker.join()

    def test_demo_cli_runs_without_mqtt_dependency(self):
        process = subprocess.Popen([sys.executable, '-S', str(ROOT / 'tools/dashboard/telemetry_dashboard.py'),
                                    '--demo', '--no-browser', '--port', '0'],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            line = process.stdout.readline().strip()
            self.assertTrue(line.startswith('Motor-Wheel dashboard: http://127.0.0.1:'), line)
            url = line.split(': ', 1)[1]
            with urlopen(url + '/api/state') as response:
                payload = json.load(response)
                self.assertTrue(payload['demo'])
                self.assertGreater(payload['received'], 0)
                self.assertNotIn('version', payload['latest']['values'])
                self.assertEqual(payload['latest']['values']['vehicle'], 2)
        finally:
            process.terminate()
            _, error = process.communicate(timeout=5)
        self.assertEqual(process.returncode, 0, error)


MQTT_AVAILABLE = importlib.util.find_spec('paho') is not None and shutil.which('mosquitto')


@unittest.skipUnless(MQTT_AVAILABLE, 'Install dashboard requirements and mosquitto for broker integration')
class MQTTTests(unittest.TestCase):
    def test_subscribe_reject_and_reconnect(self):
        import paho.mqtt.client as mqtt
        live_packet = with_field(VECTOR, 35, 12, 0)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        state = dashboard.Telemetry(topics=TOPICS)
        args = argparse.Namespace(username=None, tls=False, broker='127.0.0.1', broker_port=port)
        broker = None
        client = None
        publisher = None

        def start_broker():
            process = subprocess.Popen(['mosquitto', '-p', str(port)],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            def ready():
                if process.poll() is not None:
                    raise AssertionError('Local test broker exited')
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=.1):
                        return True
                except OSError:
                    return False
            until(ready)
            return process

        def new_publisher():
            pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            pub.connect('127.0.0.1', port)
            pub.loop_start()
            until(pub.is_connected)
            return pub

        try:
            # Start the dashboard before the broker to check retry on initial failure.
            client = dashboard.start_mqtt(state, args)
            until(lambda: state.snapshot()['connection'] == 'disconnected')
            broker = start_broker()
            until(lambda: state.snapshot()['connection'] == 'connected')
            publisher = new_publisher()
            commands = []
            publisher.on_message = lambda client, userdata, message: commands.append(
                (message.payload, message.retain, message.qos))
            subscribed = threading.Event()
            publisher.on_subscribe = lambda *args: subscribed.set()
            publisher.subscribe(TOPICS['command'])
            self.assertTrue(subscribed.wait(3))
            publisher.publish(TOPICS['telemetry'], live_packet, qos=1).wait_for_publish(3)
            until(lambda: state.snapshot()['received'] == 1)
            command = state.send_command('stop')
            until(lambda: len(commands) == 1)
            self.assertEqual(commands[0], (dashboard.encode_command('stop', command['sequence']), False, 0))
            self.assertEqual(state.snapshot()['controls']['command']['status'], 'pending')
            publisher.publish(TOPICS['ack'], bytes([commands[0][0][0] | 64]), qos=1).wait_for_publish(3)
            until(lambda: state.snapshot()['controls']['can_start'])
            command = state.send_command('start')
            until(lambda: len(commands) == 2)
            self.assertEqual(commands[1], (dashboard.encode_command('start', command['sequence']), False, 0))
            publisher.publish(TOPICS['ack'], bytes([commands[1][0][0] | 64]), qos=1).wait_for_publish(3)
            until(lambda: state.snapshot()['controls']['command']['status'] == 'accepted')
            publisher.publish(TOPICS['telemetry'], b'bad', qos=1).wait_for_publish(3)
            until(lambda: state.snapshot()['rejected'] == 1)
            publisher.disconnect()
            publisher.loop_stop()
            publisher = None
            broker.terminate()
            broker.wait(timeout=5)
            until(lambda: state.snapshot()['connection'] == 'disconnected')
            broker = start_broker()
            until(lambda: state.snapshot()['connection'] == 'connected')
            publisher = new_publisher()
            publisher.publish(TOPICS['telemetry'], live_packet, qos=1).wait_for_publish(3)
            until(lambda: state.snapshot()['received'] == 2)
            self.assertEqual(len(commands), 2)  # No command is replayed on reconnect.
            self.assertFalse(state.snapshot()['controls']['can_start'])
            self.assertTrue(state.snapshot()['controls']['can_stop'])
            self.assertEqual(state.snapshot()['latest']['values']['pitch_deg'], -5.0)
        finally:
            if publisher:
                publisher.disconnect()
                publisher.loop_stop()
            if client:
                client.disconnect()
                client.loop_stop()
            if broker and broker.poll() is None:
                broker.terminate()
                broker.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
