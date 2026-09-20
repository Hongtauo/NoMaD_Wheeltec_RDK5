"""Injected backend + genuine local TLS: no GPU, ROS, robot, or network changes."""

import base64
import copy
import http.client
import io
import json
import os
from pathlib import Path
import shutil
import socket
import ssl
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import uuid

from nomad_navigation.campus_api_server import (
    APIError, AuditLog, CampusInferenceService, create_server, decode_images,
)
from nomad_navigation.campus_protocol import (
    ProtocolError, _validate_request_and_decode_jpegs, decode_jpeg_bytes,
    decode_json, encode_json,
)


TOKEN = 'test-token-not-a-real-secret-' + 'x' * 32


def request_fixture(sequence=1, session_id=None, offset=0):
    return {'version': 1, 'session_id': session_id or str(uuid.uuid4()),
            'sequence': sequence, 'frame_id': 'camera_color_optical_frame',
            'observation_now_ns': 10_800_000_000 + offset,
            'frames': [{'stamp_ns': 10_000_000_000 + n * 250_000_000 + offset,
                        'jpeg_b64': base64.b64encode(b'\xff\xd8test\xff\xd9').decode()}
                       for n in range(4)]}


def jpeg_request_fixture(size=(320, 240)):
    try:
        from PIL import Image
        import numpy  # noqa: F401
    except ImportError:
        raise unittest.SkipTest('PIL/numpy unavailable in this interpreter')
    request = request_fixture()
    stream = io.BytesIO()
    Image.new('RGB', size, (20, 30, 40)).save(stream, 'JPEG')
    encoded = base64.b64encode(stream.getvalue()).decode()
    for frame in request['frames']:
        frame['jpeg_b64'] = encoded
    return request


class MockBackend:
    context_frames = 4
    num_samples = 8
    config = {'len_traj_pred': 8, 'image_size': [96, 96]}

    def __init__(self):
        self.calls = []

    def infer(self, frames, sequence):
        self.calls.append(sequence)
        return [[[float(step), 0.] for step in range(8)] for _ in range(8)], 25.


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.backend = MockBackend()
        self.results = []
        self.service = CampusInferenceService(self.backend, TOKEN, clock=lambda: self.now,
            image_decoder=lambda p: p['frames'], on_result=lambda *args: self.results.append(args))
        self.addCleanup(self.service.close)

    def assert_api_error(self, code, action):
        with self.assertRaises(APIError) as caught:
            action()
        self.assertEqual(caught.exception.code, code)

    def test_scale_and_original_stamp(self):
        request = request_fixture()
        response = self.service.infer(request)
        self.assertEqual(response['waypoint'], [0.1, 0.])
        self.assertEqual(response['trajectories'][0][7], [0.35000000000000003, 0.])
        self.assertEqual(response['observation_stamp_ns'], request['frames'][-1]['stamp_ns'])
        self.assertEqual(self.backend.calls, [1])
        self.assertEqual(len(self.results), 1)

    def test_bad_backend_configuration_and_tokens(self):
        for token in ('x', 'x' * 257, 'é' * 32, 'x ' * 32):
            with self.assertRaises(ValueError):
                CampusInferenceService(self.backend, token)
        self.backend.context_frames = 6
        with self.assertRaises(ValueError):
            CampusInferenceService(self.backend, TOKEN)

    def test_authentication_exact(self):
        self.assertTrue(self.service.authenticate('Bearer ' + TOKEN))
        for value in (None, TOKEN, 'Bearer wrong', 'bearer ' + TOKEN, 'Bearer ' + TOKEN + '\n', 'Bearer é'):
            self.assertFalse(self.service.authenticate(value))

    def test_duplicate_sequence_stamp_and_clock_rollback(self):
        request = request_fixture()
        self.service.infer(request)
        self.assert_api_error('replayed_or_nonmonotonic_observation', lambda: self.service.infer(request))
        stale = copy.deepcopy(request)
        stale['sequence'] = 2
        self.assert_api_error('replayed_or_nonmonotonic_observation', lambda: self.service.infer(stale))
        next_request = request_fixture(2, request['session_id'], 250_000_000)
        self.service.infer(next_request)
        self.assertEqual(self.backend.calls, [1, 2])

    def test_session_isolation_retirement_and_capacity(self):
        first = request_fixture()
        self.service.infer(first)
        second = request_fixture()
        self.assert_api_error('session_busy', lambda: self.service.infer(second))
        self.now += 5.1
        self.service.infer(second)
        self.now += 5.1
        self.assert_api_error('retired_session', lambda: self.service.infer(first))
        self.service.max_sessions = 2
        self.assert_api_error('session_capacity_restart_required', lambda: self.service.infer(request_fixture()))

    def test_busy_is_not_queued(self):
        self.service._gpu.acquire()
        try:
            self.assert_api_error('busy', lambda: self.service.infer(request_fixture()))
            self.assertEqual(self.backend.calls, [])
        finally:
            self.service._gpu.release()

    def test_processing_deadline_discards_and_consumes_sequence(self):
        request = request_fixture()
        original = self.backend.infer
        def slow(*args):
            self.now += 0.601
            actions, _ = original(*args)
            return actions, 601.
        self.backend.infer = slow
        self.assert_api_error('processing_deadline_exceeded', lambda: self.service.infer(request))
        self.assertEqual(self.results, [])
        self.assert_api_error('replayed_or_nonmonotonic_observation', lambda: self.service.infer(request))

    def test_invalid_model_result_never_previews(self):
        for bad in ([], [[[float('nan'), 0.]] * 8] * 8,
                    [[[True, 0.]] * 8] * 8, [[['1.0', 0.]] * 8] * 8):
            self.backend.infer = lambda frames, sequence, bad=bad: (bad, 10.)
            service = CampusInferenceService(self.backend, TOKEN, image_decoder=lambda p: p['frames'])
            self.addCleanup(service.close)
            with self.assertRaises(APIError):
                service.infer(request_fixture())
        self.assertEqual(self.results, [])

    def test_audit_unique_and_no_input_data(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = AuditLog(directory), AuditLog(directory)
            self.assertNotEqual(first.run_dir, second.run_dir)
            self.service.audit = first
            self.service.infer(request_fixture())
            with self.assertRaises(ValueError):
                first.write('bad', token=TOKEN)
            first.close()
            second.close()
            text = (first.run_dir / 'events.jsonl').read_text()
            self.assertNotIn(TOKEN, text)
            self.assertNotIn('jpeg_b64', text)
            self.assertIn('prediction', text)

    def test_audit_diagnostics_reject_sensitive_fields_and_close_safely(self):
        with tempfile.TemporaryDirectory() as directory:
            audit = AuditLog(directory)
            row = dict(phase='body', status=408, code='request_deadline_exceeded',
                       declared_body_bytes=100, body_bytes=1, body_read_ms=600.,
                       json_decode_ms=None, service_ms=None, elapsed_ms=601.)
            self.assertTrue(audit.write('request_stats', **row))
            for name in ('token', 'body', 'authorization', 'jpeg_b64', 'headers', 'url', 'path'):
                with self.assertRaises(ValueError):
                    audit.write('request_stats', **{name: 'must-not-appear'})
            for name, value in (('phase', TOKEN), ('body_bytes', TOKEN),
                                ('declared_body_bytes', -1), ('service_ms', TOKEN),
                                ('elapsed_ms', float('nan')), ('json_decode_ms', True)):
                with self.assertRaises(ValueError):
                    audit.write('request_stats', **{name: value})
            audit.close()
            audit.close()
            errors = []
            def late_write():
                try:
                    self.assertFalse(audit.write('request_stats', **row))
                except Exception as error:
                    errors.append(error)
            thread = threading.Thread(target=late_write)
            thread.start()
            thread.join(1.)
            self.assertEqual(errors, [])
            rows = (audit.run_dir/'events.jsonl').read_text().splitlines()
            self.assertEqual(len(rows), 1)
            self.assertEqual(json.loads(rows[0])['body_bytes'], 1)
            self.assertNotIn(TOKEN, rows[0])

    def test_audit_record_cap_still_bounds_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            audit = AuditLog(directory)
            audit.count = 1_000_000
            self.assertFalse(audit.write('request_stats', phase='complete', status=200))
            audit.close()
            self.assertEqual((audit.run_dir/'events.jsonl').read_text(), '')

    def test_tls_is_mandatory(self):
        with self.assertRaises(ValueError):
            create_server(('127.0.0.1', 0), self.service)
        with self.assertRaises(ValueError):
            create_server(('127.0.0.1', 0), self.service,
                          tls_context=ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT))

    def test_real_jpeg_dimensions_and_pixels(self):
        try:
            from PIL import Image
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest('PIL/numpy unavailable in this interpreter')
        request = request_fixture()
        buffer = io.BytesIO()
        Image.new('RGB', (640, 480), (20, 30, 40)).save(buffer, 'JPEG')
        for frame in request['frames']:
            frame['jpeg_b64'] = base64.b64encode(buffer.getvalue()).decode()
        frames = decode_images(request)
        self.assertEqual(frames[0].shape, (480, 640, 3))
        buffer = io.BytesIO()
        Image.new('RGB', (1930, 20)).save(buffer, 'JPEG')
        request['frames'][0]['jpeg_b64'] = base64.b64encode(buffer.getvalue()).decode()
        with self.assertRaises(ProtocolError):
            decode_images(request)

    def test_standalone_decoder_still_validates_entire_request_and_decodes_base64_once(self):
        request = jpeg_request_fixture()
        with patch('nomad_navigation.campus_protocol.decode_jpeg_bytes',
                   wraps=decode_jpeg_bytes) as decode:
            frames = decode_images(request)
        self.assertEqual(decode.call_count, 4)
        self.assertEqual(len(frames), 4)
        self.assertEqual(frames[0].shape, (240, 320, 3))
        for mutate in (lambda p: p.update(sequence=True),
                       lambda p: p.update(session_id='invalid'),
                       lambda p: p['frames'][-1].update(jpeg_b64='***'),
                       lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns']+1_000_000_001)):
            invalid = copy.deepcopy(request)
            mutate(invalid)
            with patch('nomad_navigation.campus_api_server._decode_validated_jpegs') as pixels:
                with self.assertRaises(ProtocolError):
                    decode_images(invalid)
                pixels.assert_not_called()

    def test_default_worker_reuses_validated_bytes_without_skipping_image_checks(self):
        for request in (request_fixture(), jpeg_request_fixture((1930, 20)),
                        jpeg_request_fixture((1920, 1200)), jpeg_request_fixture((15, 30))):
            backend = MockBackend()
            service = CampusInferenceService(backend, TOKEN)
            self.addCleanup(service.close)
            with patch('nomad_navigation.campus_protocol.decode_jpeg_bytes',
                       wraps=decode_jpeg_bytes) as decode:
                with self.assertRaises(APIError) as error:
                    service.infer(request)
            self.assertEqual((error.exception.status, error.exception.code),
                             (422, 'invalid_image_or_model_result'))
            self.assertEqual(decode.call_count, 4)
            self.assertEqual(backend.calls, [])
            # Valid protocol / bad pixel decoding still consumes its sequence.
            with self.assertRaises(APIError) as repeated:
                service.infer(request)
            self.assertEqual(repeated.exception.code, 'replayed_or_nonmonotonic_observation')

    def test_injected_decoder_keeps_strict_input_validation_and_worker_payload_interface(self):
        calls = []
        def injected(request):
            calls.append((request, threading.get_ident()))
            return request['frames']
        self.service.image_decoder = injected
        request = request_fixture()
        self.service.infer(request)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], request)
        self.assertIsNot(calls[0][0], request)
        self.assertEqual(calls[0][1], self.service._worker.ident)
        invalid = request_fixture(2, request['session_id'], 250_000_000)
        invalid['frames'][-1]['jpeg_b64'] = '***'
        self.assert_api_error('invalid_request', lambda: self.service.infer(invalid))
        self.assertEqual(len(calls), 1)

    def test_validation_bytes_and_worker_identity_share_detached_request_snapshot(self):
        from nomad_navigation.campus_api_server import _decode_validated_jpegs
        request = jpeg_request_fixture()
        original = copy.deepcopy(request)
        backend = MockBackend()
        service = CampusInferenceService(backend, TOKEN)
        self.addCleanup(service.close)
        entered, release = threading.Event(), threading.Event()
        replies, errors = [], []
        def paused_decoder(jpegs):
            self.assertIsInstance(jpegs, tuple)
            self.assertTrue(all(type(image) is bytes for image in jpegs))
            entered.set()
            self.assertTrue(release.wait(.5))
            return _decode_validated_jpegs(jpegs)
        def invoke():
            try:
                replies.append(service.infer(request))
            except Exception as error:
                errors.append(error)
        with patch('nomad_navigation.campus_api_server._decode_validated_jpegs',
                   side_effect=paused_decoder):
            caller = threading.Thread(target=invoke)
            caller.start()
            try:
                self.assertTrue(entered.wait(.5))
                request['session_id'] = str(uuid.uuid4())
                request['sequence'] = 999
                request['frames'][-1]['stamp_ns'] += 250_000_000
                request['frames'][-1]['jpeg_b64'] = '***'
            finally:
                release.set()
                caller.join(1.)
        self.assertFalse(caller.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(backend.calls, [original['sequence']])
        self.assertEqual(replies[0]['session_id'], original['session_id'])
        self.assertEqual(replies[0]['observation_stamp_ns'], original['frames'][-1]['stamp_ns'])
        self.assertEqual(service._active, original['session_id'])

    def test_validation_and_copy_do_not_refresh_original_processing_budget(self):
        def slow_validation(request):
            result = _validate_request_and_decode_jpegs(request)
            self.now += .601
            return result
        with patch('nomad_navigation.campus_api_server._validate_request_and_decode_jpegs',
                   side_effect=slow_validation):
            self.assert_api_error('processing_deadline_exceeded',
                                  lambda: self.service.infer(request_fixture(), started=100.))
        self.assertEqual(self.backend.calls, [])
        self.assertEqual(self.service._sessions, {})


class HTTPSNetworkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        openssl = shutil.which('openssl')
        if not openssl:
            raise unittest.SkipTest('openssl required for temporary loopback-only TLS certificate')
        cls.directory = tempfile.TemporaryDirectory()
        cls.cert = Path(cls.directory.name) / 'cert.pem'
        cls.key = Path(cls.directory.name) / 'key.pem'
        subprocess.run([openssl, 'req', '-config', os.devnull, '-x509', '-newkey', 'rsa:2048', '-nodes',
                        '-keyout', str(cls.key), '-out', str(cls.cert), '-days', '1',
                        '-subj', '/CN=localhost', '-addext', 'subjectAltName=DNS:localhost,IP:127.0.0.1'],
                       check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.service = CampusInferenceService(MockBackend(), TOKEN,
            image_decoder=lambda p: p['frames'])
        self.server = create_server(('127.0.0.1', 0), self.service, cert=self.cert, key=self.key)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
        self.thread.start()
        self.context = ssl.create_default_context(cafile=str(self.cert))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPSConnection('localhost', self.server.server_port,
                                                context=self.context, timeout=2)
        try:
            connection.request(method, path, body, headers or {})
            response = connection.getresponse()
            return response.status, decode_json(response.read())
        finally:
            connection.close()

    def collect_audit(self):
        class MemoryAudit:
            def __init__(self):
                self.rows = []
                self.complete = threading.Event()
            def write(self, event, **fields):
                self.rows.append(dict(event=event, **fields))
                if event == 'request_stats':
                    self.complete.set()
        audit = MemoryAudit()
        self.service.audit = audit
        return audit

    def test_tls_health_infer_auth_and_replay(self):
        self.assertEqual(self.request('GET', '/healthz'), (200, {'version': 1, 'ready': True}))
        request = request_fixture()
        body = encode_json(request)
        status, _ = self.request('POST', '/v1/infer', body, {'Content-Type': 'application/json'})
        self.assertEqual(status, 401)
        headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN}
        status, response = self.request('POST', '/v1/infer', body, headers)
        self.assertEqual(status, 200)
        self.assertEqual(response['waypoint'], [0.1, 0.])
        status, response = self.request('POST', '/v1/infer', body, headers)
        self.assertEqual(status, 409)
        self.assertNotIn('waypoint', response)

    def test_tls_default_decoder_base64_decodes_each_frame_once(self):
        request = jpeg_request_fixture()
        self.service.image_decoder = decode_images
        headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN}
        with patch('nomad_navigation.campus_protocol.decode_jpeg_bytes',
                   wraps=decode_jpeg_bytes) as decode:
            status, response = self.request('POST', '/v1/infer', encode_json(request), headers)
        self.assertEqual(status, 200)
        self.assertEqual(decode.call_count, 4)
        self.assertEqual(response['session_id'], request['session_id'])
        self.assertEqual(response['observation_stamp_ns'], request['frames'][-1]['stamp_ns'])
        self.assertEqual(self.service.backend.calls, [1])

    def test_raw_camera_to_compact_tls_model_preserves_identity_with_only_wire_decoding(self):
        from nomad_navigation.campus_client_core import (
            CampusClientCore, DeadlineHTTPSConnection, HTTPSInference,
        )
        from nomad_navigation.campus_image import compact_jpeg
        from nomad_navigation.campus_upload_transport import CompactUploadTransport
        try:
            from PIL import Image
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest('PIL/OpenCV required for real compact-upload integration')

        # Four distinct images prevent the content cache hiding repeated work.
        original_jpegs = []
        for index in range(4):
            stream = io.BytesIO()
            Image.new('RGB', (640, 480), (20+index*30, 30, 40)).save(stream, 'JPEG')
            original_jpegs.append(stream.getvalue())
        self.assertEqual(len(set(original_jpegs)), 4)
        stamps = tuple(10_000_000_000+index*250_000_000 for index in range(4))
        preview = []
        self.service.image_decoder = decode_images
        self.service.on_result = lambda *args: preview.append(args)
        connections, decoded_on_threads = [], []
        main_thread = threading.get_ident()

        def connection_factory(*args, **kwargs):
            connection = DeadlineHTTPSConnection(*args, **kwargs)
            connections.append(connection)
            return connection

        def count_decode(encoded):
            decoded_on_threads.append((threading.get_ident(), encoded))
            return decode_jpeg_bytes(encoded)

        with tempfile.TemporaryDirectory() as directory:
            token_file = Path(directory)/'token'
            token_file.write_text(TOKEN, encoding='ascii')
            token_file.chmod(0o600)
            https = HTTPSInference('https://127.0.0.1:'+str(self.server.server_port),
                                   self.cert, token_file)
            self.addCleanup(https.cancel)
            transport = CompactUploadTransport(https)
            transport.warmup()  # Import/codecs precede the original request deadline.
            core = CampusClientCore()
            started = time.monotonic()
            with patch('nomad_navigation.campus_protocol.decode_jpeg_bytes',
                       side_effect=count_decode) as decode, \
                    patch('nomad_navigation.campus_image.base64.b64encode',
                          wraps=base64.b64encode) as encode, \
                    patch('nomad_navigation.campus_image.compact_jpeg',
                          wraps=compact_jpeg) as compact, \
                    patch('nomad_navigation.campus_client_core.DeadlineHTTPSConnection',
                          side_effect=connection_factory), \
                    patch.object(self.service.backend, 'infer',
                                 wraps=self.service.backend.infer) as model:
                for index, (stamp, jpeg) in enumerate(zip(stamps, original_jpegs)):
                    self.assertTrue(core.add_camera(jpeg, stamp, 'camera_color_optical_frame',
                                                    stamp, started-.75+index*.25))
                request = core.request(stamps[-1], started)
                self.assertIsNotNone(request)
                self.assertEqual(decode.call_count, 0)
                self.assertEqual(encode.call_count, 0)
                self.assertTrue(all(set(frame) == {'stamp_ns', 'jpeg_bytes'}
                                    for frame in request['frames']))
                reply = transport.infer(request, started=started)
                completed = time.monotonic()
                accepted = core.complete(reply, None, completed,
                    stamps[-1]+int((completed-started)*1e9), completed)
                self.assertIs(accepted, reply)
                self.assertEqual(core.accepted_results, 1)
                self.assertEqual(decode.call_count, 8)
                self.assertEqual(encode.call_count, 4)
                self.assertEqual(compact.call_count, 4)
                self.assertEqual([call.args[0] for call in compact.call_args_list], original_jpegs)
                self.assertTrue(all(call.args[0] not in original_jpegs for call in encode.call_args_list))
                model.assert_called_once()
                self.assertEqual([frame.shape for frame in model.call_args.args[0]],
                                 [(240, 320, 3)]*4)
                self.assertEqual(model.call_args.args[1], request['sequence'])

            self.assertEqual(sum(thread == main_thread for thread, _ in decoded_on_threads), 4)
            self.assertEqual(sum(thread != main_thread for thread, _ in decoded_on_threads), 4)
            self.assertEqual(len(connections), 1)
            self.assertEqual(connections[0].deadline, started+.8)
            self.assertTrue(connections[0].cancelled.is_set())
            self.assertLess(completed-started, .8)
            self.assertEqual(https.snapshot()['counts'], {'attempts': 1, 'succeeded': 1})

        self.assertEqual(len(preview), 1)
        wire_request, preview_response, preview_frames = preview[0]
        self.assertEqual([frame['stamp_ns'] for frame in wire_request['frames']], list(stamps))
        self.assertTrue(all(set(frame) == {'stamp_ns', 'jpeg_b64'}
                            for frame in wire_request['frames']))
        self.assertEqual(wire_request['session_id'], core.session_id)
        self.assertEqual(wire_request['sequence'], request['sequence'])
        self.assertEqual(wire_request['observation_now_ns'], stamps[-1])
        self.assertEqual(preview_response, reply)
        self.assertEqual(reply['session_id'], core.session_id)
        self.assertEqual(reply['sequence'], request['sequence'])
        self.assertEqual(reply['observation_stamp_ns'], stamps[-1])
        self.assertEqual([frame.shape for frame in preview_frames], [(240, 320, 3)]*4)

    def test_tls_bad_fields_and_jpeg_still_fail_before_model(self):
        self.service.image_decoder = decode_images
        headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN}
        malformed = request_fixture()
        malformed['frame_id'] = 'base_link'
        status, response = self.request('POST', '/v1/infer', encode_json(malformed), headers)
        self.assertEqual((status, response['error']), (400, 'invalid_request'))
        self.assertEqual(self.service._sessions, {})
        status, response = self.request('POST', '/v1/infer', encode_json(request_fixture()), headers)
        self.assertEqual((status, response['error']), (422, 'invalid_image_or_model_result'))
        self.assertEqual(self.service.backend.calls, [])

    def test_accepted_tls_socket_disables_nagle(self):
        import socket
        from unittest.mock import patch
        from nomad_navigation.campus_api_server import CampusRequestHandler
        observed = []
        original = CampusRequestHandler._send_json

        def inspect_socket(handler, status, payload):
            observed.append(handler.connection.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY))
            return original(handler, status, payload)

        with patch.object(CampusRequestHandler, '_send_json', inspect_socket):
            self.assertEqual(self.request('GET', '/healthz')[0], 200)
        self.assertEqual(observed, [1])

    def test_oversized_invalid_json_content_type(self):
        auth = {'Authorization': 'Bearer ' + TOKEN, 'Content-Type': 'application/json'}
        self.assertEqual(self.request('POST', '/v1/infer', b'{"x":NaN}', auth)[0], 400)
        self.assertEqual(self.request('POST', '/v1/infer', b'{}', {**auth, 'Content-Length': str(5 * 1024 * 1024)})[0], 413)
        self.assertEqual(self.request('POST', '/v1/infer', b'{}', {**auth, 'Content-Type': 'text/plain'})[0], 415)

    def test_untrusted_certificate_and_plain_http_rejected(self):
        connection = http.client.HTTPSConnection('localhost', self.server.server_port, timeout=2)
        with self.assertRaises(ssl.SSLCertVerificationError):
            connection.request('GET', '/healthz')
        connection.close()
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        with self.assertRaises((ConnectionError, OSError, http.client.HTTPException)):
            connection.request('GET', '/healthz')
            connection.getresponse()
        connection.close()

    def test_live_gpu_contention_returns_503(self):
        self.service._gpu.acquire()
        try:
            status, result = self.request('POST', '/v1/infer', encode_json(request_fixture()),
                {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN})
            self.assertEqual((status, result['error']), (503, 'busy'))
        finally:
            self.service._gpu.release()

    def test_duplicate_content_length_and_auth_rejected(self):
        for duplicate, expected in (('Content-Length', 411), ('Authorization', 401)):
            connection = http.client.HTTPSConnection('localhost', self.server.server_port,
                                                    context=self.context, timeout=2)
            try:
                connection.putrequest('POST', '/v1/infer')
                connection.putheader('Content-Type', 'application/json')
                connection.putheader('Authorization', 'Bearer ' + TOKEN)
                connection.putheader('Content-Length', '2')
                connection.putheader(duplicate, '2' if duplicate == 'Content-Length' else 'Bearer ' + TOKEN)
                connection.endheaders(b'{}')
                self.assertEqual(connection.getresponse().status, expected)
            finally:
                connection.close()

    def test_slow_body_hits_total_deadline(self):
        audit = self.collect_audit()
        connection = http.client.HTTPSConnection('localhost', self.server.server_port,
                                                context=self.context, timeout=2)
        try:
            connection.putrequest('POST', '/v1/infer')
            connection.putheader('Content-Type', 'application/json')
            connection.putheader('Authorization', 'Bearer ' + TOKEN)
            connection.putheader('Content-Length', '100')
            connection.endheaders(b'{')
            started = time.monotonic()
            response = connection.getresponse()
            self.assertEqual(response.status, 408)
            self.assertLess(time.monotonic() - started, 1.2)
            self.assertEqual(self.service.backend.calls, [])
            self.assertTrue(audit.complete.wait(1.))
            row = next(item for item in audit.rows if item['event'] == 'request_stats')
            self.assertEqual((row['status'], row['phase']), (408, 'body'))
            self.assertEqual((row['declared_body_bytes'], row['body_bytes']), (100, 1))
            self.assertGreaterEqual(row['body_read_ms'], 400.)
            self.assertIsNone(row['json_decode_ms'])
            self.assertIsNone(row['service_ms'])
        finally:
            connection.close()

    def test_handler_slots_are_bounded_and_recover(self):
        # Saturate through semaphore ownership without relying on thread timing.
        for _ in range(4):
            self.assertTrue(self.server._slots.acquire(blocking=False))
        try:
            with self.assertRaises((ConnectionError, OSError, ssl.SSLError, http.client.HTTPException)):
                self.request('GET', '/healthz')
        finally:
            for _ in range(4):
                self.server._slots.release()
        self.assertEqual(self.request('GET', '/healthz')[0], 200)

    def test_separate_https_requests_share_one_model_thread(self):
        identities = []
        original = self.service.backend.infer
        def record(*args):
            identities.append(threading.get_ident())
            return original(*args)
        self.service.backend.infer = record
        session = str(uuid.uuid4())
        for sequence in range(1, 4):
            status, _ = self.request('POST', '/v1/infer',
                encode_json(request_fixture(sequence, session, sequence * 250_000_000)),
                {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN})
            self.assertEqual(status, 200)
        self.assertEqual(identities, [self.service._worker.ident] * 3)

    def test_success_diagnostics_cover_each_http_stage_without_data(self):
        audit = self.collect_audit()
        request = request_fixture()
        body = encode_json(request)
        status, _ = self.request('POST', '/v1/infer', body,
            {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN})
        self.assertEqual(status, 200)
        self.assertTrue(audit.complete.wait(1.))
        row = next(item for item in audit.rows if item['event'] == 'request_stats')
        self.assertEqual((row['status'], row['phase'], row['code']), (200, 'complete', 'ok'))
        self.assertEqual(row['body_bytes'], len(body))
        self.assertEqual(row['declared_body_bytes'], len(body))
        for name in ('body_read_ms', 'json_decode_ms', 'service_ms', 'elapsed_ms'):
            self.assertGreaterEqual(row[name], 0.)
        self.assertGreaterEqual(row['elapsed_ms'], row['body_read_ms'] + row['json_decode_ms'] + row['service_ms'])
        serialized = json.dumps(audit.rows)
        self.assertNotIn(TOKEN, serialized)
        self.assertNotIn(request['frames'][0]['jpeg_b64'], serialized)
        self.assertNotIn(request['session_id'], serialized)

    def test_failed_json_diagnostic_identifies_json_stage(self):
        audit = self.collect_audit()
        status, _ = self.request('POST', '/v1/infer', b'{"x":NaN}',
            {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN})
        self.assertEqual(status, 400)
        self.assertTrue(audit.complete.wait(1.))
        row = next(item for item in audit.rows if item['event'] == 'request_stats')
        self.assertEqual((row['phase'], row['code']), ('json', 'invalid_json'))
        self.assertEqual(row['body_bytes'], 9)
        self.assertIsNone(row['service_ms'])

    def test_busy_diagnostic_identifies_service_stage(self):
        audit = self.collect_audit()
        self.service._gpu.acquire()
        try:
            status, _ = self.request('POST', '/v1/infer', encode_json(request_fixture()),
                {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN})
            self.assertEqual(status, 503)
            self.assertTrue(audit.complete.wait(1.))
            row = next(item for item in audit.rows if item['event'] == 'request_stats')
            self.assertEqual((row['phase'], row['code']), ('service', 'busy'))
            self.assertGreaterEqual(row['service_ms'], 0.)
        finally:
            self.service._gpu.release()

    def test_model_deadline_diagnostic_distinguishes_full_body_from_slow_upload(self):
        audit = self.collect_audit()
        release = threading.Event()
        original = self.service.backend.infer
        def slow(*args):
            release.wait(2.)
            return original(*args)
        self.service.backend.infer = slow
        self.service.budget_seconds = .08
        body = encode_json(request_fixture())
        try:
            status, _ = self.request('POST', '/v1/infer', body,
                {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + TOKEN})
            self.assertEqual(status, 504)
            self.assertTrue(audit.complete.wait(1.))
            row = next(item for item in audit.rows if item['event'] == 'request_stats')
            self.assertEqual((row['phase'], row['code']), ('service', 'processing_deadline_exceeded'))
            self.assertEqual(row['body_bytes'], len(body))
            self.assertEqual(row['declared_body_bytes'], len(body))
            self.assertGreaterEqual(row['service_ms'], 40.)
            self.assertFalse(any(item['event'] == 'prediction' for item in audit.rows))
        finally:
            release.set()


class PersistentWorkerTests(unittest.TestCase):
    def service(self, backend=None, **kwargs):
        service = CampusInferenceService(backend or MockBackend(), TOKEN,
            image_decoder=lambda p: p['frames'], **kwargs)
        self.addCleanup(service.close)
        return service

    def assert_api_error(self, code, action):
        with self.assertRaises(APIError) as caught:
            action()
        self.assertEqual(caught.exception.code, code)

    def test_factory_warmup_and_requests_run_in_exact_same_thread(self):
        identities, callbacks = [], []
        def factory():
            identities.append(('load', threading.get_ident()))
            backend = MockBackend()
            original = backend.infer
            def record(frames, sequence):
                identities.append((sequence, threading.get_ident()))
                return original(frames, sequence)
            backend.infer = record
            backend.infer([], 0)  # Equivalent placement to NoMaDBackend warmup.
            return backend
        service = CampusInferenceService(None, TOKEN, backend_factory=factory,
            image_decoder=lambda p: p['frames'], on_result=lambda *args: callbacks.append(threading.get_ident()))
        self.addCleanup(service.close)
        session = str(uuid.uuid4())
        for sequence in range(1, 4):
            service.infer(request_fixture(sequence, session, sequence * 250_000_000))
        self.assertEqual([entry[0] for entry in identities], ['load', 0, 1, 2, 3])
        self.assertEqual({entry[1] for entry in identities}, {service._worker.ident})
        self.assertEqual(callbacks, [service._worker.ident] * 3)
        self.assertNotEqual(service._worker.ident, threading.get_ident())

    def test_factory_failure_does_not_leave_worker(self):
        before = {thread.ident for thread in threading.enumerate() if thread.name == 'campus-inference'}
        def broken_factory():
            raise RuntimeError('mock load failure')
        with self.assertRaisesRegex(RuntimeError, 'mock load failure'):
            CampusInferenceService(None, TOKEN, backend_factory=broken_factory)
        after = {thread.ident for thread in threading.enumerate() if thread.name == 'campus-inference'}
        self.assertEqual(after, before)

    def test_backend_exception_reaches_caller_and_worker_remains_usable(self):
        backend = MockBackend()
        original = backend.infer
        def fail_first(frames, sequence):
            if sequence == 1:
                raise RuntimeError('sensitive detail must not reach HTTP response')
            return original(frames, sequence)
        backend.infer = fail_first
        service = self.service(backend)
        request = request_fixture()
        self.assert_api_error('inference_failed', lambda: service.infer(request))
        response = service.infer(request_fixture(2, request['session_id'], 250_000_000))
        self.assertEqual(response['state'], 'running')
        self.assertTrue(service._worker.is_alive())

    def test_timeout_keeps_gpu_locked_no_queue_and_discards_late_result(self):
        entered, release = threading.Event(), threading.Event()
        callbacks = []
        backend = MockBackend()
        original = backend.infer
        def block_once(frames, sequence):
            if sequence == 1:
                entered.set()
                release.wait(2.)
            return original(frames, sequence)
        backend.infer = block_once
        service = self.service(backend, budget_seconds=.06,
                               on_result=lambda *args: callbacks.append(args))
        self.addCleanup(release.set)
        request = request_fixture()
        started = time.monotonic()
        self.assert_api_error('processing_deadline_exceeded', lambda: service.infer(request))
        self.assertLess(time.monotonic() - started, .3)
        self.assertTrue(entered.is_set())
        job = service._active_job
        self.assertIsNotNone(job)
        self.assertTrue(service._gpu.locked())
        for _ in range(5):
            self.assert_api_error('busy', lambda: service.infer(
                request_fixture(2, request['session_id'], 250_000_000)))
        self.assertIsNone(service._slot)
        release.set()
        self.assertTrue(job['done'].wait(1.))
        self.assertFalse(service._gpu.locked())
        self.assertEqual(callbacks, [])
        self.assertEqual(service.infer(request_fixture(2, request['session_id'], 250_000_000))['state'], 'running')
        self.assertEqual(backend.calls, [1, 2])

    def test_close_running_model_returns_false_without_unlock_and_prevents_delivery(self):
        entered, release = threading.Event(), threading.Event()
        outcomes, callbacks = [], []
        backend = MockBackend()
        original = backend.infer
        def blocking(*args):
            entered.set()
            release.wait(2.)
            return original(*args)
        backend.infer = blocking
        service = self.service(backend, on_result=lambda *args: callbacks.append(args))
        self.addCleanup(release.set)
        def caller():
            try:
                service.infer(request_fixture())
            except APIError as error:
                outcomes.append(error.code)
        thread = threading.Thread(target=caller)
        thread.start()
        self.assertTrue(entered.wait(1.))
        self.assertFalse(service.close(timeout=.01))
        self.assertTrue(service._gpu.locked())
        self.assertFalse(service.ready)
        thread.join(.3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(outcomes, ['not_ready'])
        self.assert_api_error('not_ready', lambda: service.infer(request_fixture()))
        release.set()
        self.assertTrue(service.close(timeout=1.))
        self.assertFalse(service._gpu.locked())
        self.assertEqual(callbacks, [])

    def test_timeout_during_decode_never_calls_model(self):
        release = threading.Event()
        backend = MockBackend()
        def slow_decode(request):
            release.wait(2.)
            return request['frames']
        service = CampusInferenceService(backend, TOKEN, image_decoder=slow_decode, budget_seconds=.05)
        self.addCleanup(service.close)
        self.addCleanup(release.set)
        self.assert_api_error('processing_deadline_exceeded', lambda: service.infer(request_fixture()))
        job = service._active_job
        release.set()
        self.assertTrue(job['done'].wait(1.))
        self.assertEqual(backend.calls, [])

    def test_callback_failure_transfers_to_caller(self):
        def fail(*args):
            raise RuntimeError('mock viewer failed')
        service = self.service(on_result=fail)
        self.assert_api_error('inference_failed', lambda: service.infer(request_fixture()))
        self.assertFalse(service._gpu.locked())

    def test_close_idle_is_idempotent_and_joins_worker(self):
        service = self.service()
        self.assertTrue(service.close())
        self.assertTrue(service.close())
        self.assertFalse(service._worker.is_alive())
        self.assertFalse(service.ready)

    def test_worker_threads_do_not_accumulate(self):
        before = {thread.ident for thread in threading.enumerate() if thread.name == 'campus-inference'}
        for _ in range(5):
            service = self.service()
            service.infer(request_fixture())
            self.assertTrue(service.close())
        after = {thread.ident for thread in threading.enumerate() if thread.name == 'campus-inference'}
        self.assertEqual(after, before)


if __name__ == '__main__':
    unittest.main()
