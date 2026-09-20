"""Real loopback TLS tests for the standard-library robot-side transport."""
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import shutil
import ssl
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from nomad_navigation.campus_client_core import (
    CampusClientCore, FatalClientError, HTTPResponseError, HTTPSInference,
)
from nomad_navigation.campus_protocol import decode_json, encode_json


def fixture():
    core = CampusClientCore()
    for stamp in (10_000_000_000, 10_250_000_000, 10_500_000_000, 10_750_000_000):
        core.add_camera(b'\xff\xd8test\xff\xd9', stamp, 'camera_color_optical_frame', stamp, stamp/1e9)
    payload = core.request(10_750_000_000, 10.75)
    # Transport-only fixtures use synthetic JPEG envelopes, not real pixels.
    # Production reaches this boundary through UploadImagePreparer instead.
    for frame in payload['frames']:
        frame['jpeg_b64'] = base64.b64encode(frame.pop('jpeg_bytes')).decode('ascii')
    return payload


class QuietServer(ThreadingHTTPServer):
    daemon_threads = True
    def handle_error(self, request, client_address):
        pass


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    def log_message(self, *args):
        pass

    def do_POST(self):
        payload = decode_json(self.rfile.read(int(self.headers['Content-Length'])))
        paths = [[[.1*(i+1), .01] for i in range(8)] for _ in range(8)]
        result = dict(version=1, session_id=payload['session_id'], sequence=payload['sequence'],
                      observation_stamp_ns=payload['frames'][-1]['stamp_ns'], frame_id='base_link',
                      state='running', waypoint=paths[0][2], trajectories=paths,
                      inference_ms=100., waypoint_scale=.05)
        mode = self.server.mode
        if mode == 'mismatch':
            result['sequence'] += 1
        body = encode_json(result)
        if mode == 'header_trickle':
            for character in b'HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n':
                self.wfile.write(bytes([character]))
                self.wfile.flush()
                time.sleep(.035)
            return
        status = int(mode.split('_', 1)[1]) if mode.startswith('http_') else (403 if mode == 'auth' else 200)
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Connection', 'close')
        if mode == 'duplicate_length':
            self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        if mode == 'body_trickle':
            for character in body:
                self.wfile.write(bytes([character]))
                self.wfile.flush()
                time.sleep(.035)
        else:
            self.wfile.write(body)
            self.wfile.flush()


class TLSClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        openssl = shutil.which('openssl')
        if openssl is None:
            raise unittest.SkipTest('openssl required to generate ephemeral loopback certificate')
        cls.directory = tempfile.TemporaryDirectory()
        root = Path(cls.directory.name)
        cls.cert, cls.key, cls.token = root/'cert.pem', root/'key.pem', root/'token'
        config = root/'openssl.cnf'
        config.write_text('[req]\ndistinguished_name=dn\n[dn]\n', encoding='ascii')
        subprocess.run([openssl, 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
                        '-config', str(config),
                        '-keyout', str(cls.key), '-out', str(cls.cert), '-days', '1',
                        '-subj', '/CN=localhost', '-addext', 'subjectAltName=IP:127.0.0.1'],
                       check=True, capture_output=True)
        cls.token.write_text('test-only-token-'+'x'*32, encoding='ascii')
        cls.token.chmod(0o600)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.server = QuietServer(('127.0.0.1', 0), Handler)
        self.server.mode = 'normal'
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(self.cert, self.key)
        self.server.socket = context.wrap_socket(self.server.socket, server_side=True)
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={'poll_interval': .02}, daemon=True)
        self.thread.start()
        self.transport = HTTPSInference('https://127.0.0.1:' + str(self.server.server_port),
                                        self.cert, self.token)

    def tearDown(self):
        self.transport.cancel()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=.5)

    def test_actual_https_success_preserves_robot_stamp_and_ignores_proxy_env(self):
        payload = fixture()
        with patch.dict('os.environ', {'HTTPS_PROXY': 'http://127.0.0.1:1',
                                       'https_proxy': 'http://127.0.0.1:1'}):
            reply = self.transport.infer(payload)
        self.assertEqual(reply['observation_stamp_ns'], payload['frames'][-1]['stamp_ns'])
        self.assertEqual(reply['session_id'], payload['session_id'])

    def test_bad_certificate_is_fatal(self):
        self.transport.context = ssl.create_default_context()
        with self.assertRaises(FatalClientError):
            self.transport.infer(fixture())

    def test_authentication_protocol_and_response_identity_fatal(self):
        for mode in ('auth', 'mismatch', 'duplicate_length'):
            with self.subTest(mode=mode):
                self.server.mode = mode
                with self.assertRaises(FatalClientError):
                    self.transport.infer(fixture())

    def test_http_error_preserves_status_not_response_body(self):
        for status in (408, 429, 503, 504):
            with self.subTest(status=status):
                self.server.mode = 'http_%d' % status
                with self.assertRaises(HTTPResponseError) as caught:
                    self.transport.infer(fixture())
                self.assertEqual(caught.exception.status, status)
                self.assertEqual(str(caught.exception), 'Inference service returned HTTP %d' % status)
                self.assertNotIn('waypoint', str(caught.exception))

    def test_http_401_remains_fatal_authentication_not_transient_http_error(self):
        self.server.mode = 'http_401'
        with self.assertRaises(FatalClientError):
            self.transport.infer(fixture())

    def test_header_slowloris_cannot_extend_total_deadline(self):
        self.server.mode = 'header_trickle'
        started = time.monotonic()
        with self.assertRaises(Exception):
            self.transport.infer(fixture())
        elapsed = time.monotonic()-started
        self.assertLess(elapsed, 1.05)
        self.assertGreater(elapsed, .65)

    def test_body_slowloris_close_connection_cannot_extend_total_deadline(self):
        self.server.mode = 'body_trickle'
        started = time.monotonic()
        with self.assertRaises(Exception):
            self.transport.infer(fixture())
        elapsed = time.monotonic()-started
        self.assertLess(elapsed, 1.05)
        self.assertGreater(elapsed, .65)

    def test_expired_deadline_never_sends(self):
        with self.assertRaises(TimeoutError):
            self.transport.infer(fixture(), started=time.monotonic()-1)


if __name__ == '__main__':
    unittest.main()
