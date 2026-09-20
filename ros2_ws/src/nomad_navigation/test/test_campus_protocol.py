"""The robot-facing protocol can be tested without ROS or ML dependencies."""

import base64
import copy
import math
import subprocess
import sys
import unittest
from unittest.mock import patch
import uuid

from nomad_navigation.campus_protocol import (
    MAX_BODY_BYTES, MAX_JPEG_BYTES, ProtocolError, decode_json, encode_json,
    _validate_request_and_decode_jpegs, _validate_raw_request_and_get_jpegs, decode_jpeg_bytes,
    validate_request, validate_raw_request, validate_response,
)


def request_fixture(sequence=1, session_id=None, offset=0):
    return {'version': 1, 'session_id': session_id or str(uuid.uuid4()),
            'sequence': sequence, 'frame_id': 'camera_color_optical_frame',
            'observation_now_ns': 10_800_000_000 + offset,
            'frames': [{'stamp_ns': 10_000_000_000 + n * 250_000_000 + offset,
                        'jpeg_b64': base64.b64encode(b'\xff\xd8payload\xff\xd9').decode('ascii')}
                       for n in range(4)]}


def raw_request_fixture(sequence=1, session_id=None, offset=0):
    return {'version': 1, 'session_id': session_id or str(uuid.uuid4()),
            'sequence': sequence, 'frame_id': 'camera_color_optical_frame',
            'observation_now_ns': 10_800_000_000 + offset,
            'frames': [{'stamp_ns': 10_000_000_000 + n * 250_000_000 + offset,
                        'jpeg_bytes': b'\xff\xd8payload\xff\xd9'} for n in range(4)]}


def response_fixture(request):
    return {'version': 1, 'session_id': request['session_id'], 'sequence': request['sequence'],
            'observation_stamp_ns': request['frames'][-1]['stamp_ns'], 'frame_id': 'base_link',
            'state': 'running', 'waypoint': [0.1, 0.0],
            'trajectories': [[[0.1, 0.0] for _ in range(8)] for _ in range(8)],
            'inference_ms': 150.0, 'waypoint_scale': 0.05}


class ProtocolTests(unittest.TestCase):
    def test_import_does_not_load_ml(self):
        result = subprocess.run([sys.executable, '-c',
            'import sys; import nomad_navigation.campus_protocol; '
            'assert not any(n in sys.modules for n in ("torch", "numpy", "PIL", "rclpy"))'],
            capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_valid_round_trip(self):
        request = request_fixture()
        self.assertEqual(validate_request(decode_json(encode_json(request))), request)
        response = response_fixture(request)
        self.assertEqual(validate_response(response, session_id=request['session_id'],
            sequence=1, observation_stamp_ns=request['frames'][-1]['stamp_ns']), response)

    def test_internal_decoded_frames_are_bounded_bytes_and_public_contract_is_unchanged(self):
        request = request_fixture()
        original = copy.deepcopy(request)
        with patch('nomad_navigation.campus_protocol.decode_jpeg_bytes',
                   wraps=decode_jpeg_bytes) as decode:
            frames = _validate_request_and_decode_jpegs(request)
        self.assertIsInstance(frames, tuple)
        self.assertEqual(frames, (b'\xff\xd8payload\xff\xd9',) * 4)
        self.assertEqual(decode.call_count, 4)
        self.assertTrue(all(type(frame) is bytes and len(frame) <= MAX_JPEG_BYTES for frame in frames))
        self.assertEqual(request, original)
        self.assertIs(validate_request(request), request)

    def test_internal_decoding_preserves_all_request_rejection_boundaries(self):
        mutations = [
            lambda p: p.update(version=True),
            lambda p: p.update(session_id='not-a-uuid'),
            lambda p: p.update(sequence=0),
            lambda p: p.update(frame_id='base_link'),
            lambda p: p.update(extra='field'),
            lambda p: p['frames'].pop(),
            lambda p: p['frames'][0].update(stamp_ns=True),
            lambda p: p['frames'][2].update(stamp_ns=p['frames'][1]['stamp_ns']),
            lambda p: p['frames'][-1].update(extra='field'),
            lambda p: p['frames'][-1].update(jpeg_b64='***'),
            lambda p: p['frames'][-1].update(jpeg_b64='AAAA'),
            lambda p: p['frames'][-1].update(jpeg_b64=b'not text'),
            lambda p: p['frames'][-1].update(jpeg_b64='A' * (4 * ((MAX_JPEG_BYTES + 2) // 3) + 1)),
            lambda p: p['frames'][-1].update(jpeg_b64=base64.b64encode(
                b'\xff\xd8' + b'x' * (MAX_JPEG_BYTES - 3) + b'\xff\xd9').decode('ascii')),
            lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns'] + 1_000_000_001),
            lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns'] - 200_000_001),
            lambda p: p.update(telemetry={'controller_status': {'data': 'x' * 32768}}),
            lambda p: p.update(telemetry={'unknown': 'field'}),
        ]
        for mutate in mutations:
            request = request_fixture()
            mutate(request)
            with self.subTest(mutation=mutations.index(mutate)):
                with self.assertRaises(ProtocolError) as public_error:
                    validate_request(request)
                with self.assertRaises(ProtocolError) as internal_error:
                    _validate_request_and_decode_jpegs(request)
                self.assertEqual(str(public_error.exception), str(internal_error.exception))

    def test_raw_validation_reuses_immutable_bytes_without_any_base64_work(self):
        request = raw_request_fixture()
        original = copy.deepcopy(request)
        with patch('nomad_navigation.campus_protocol.base64.b64decode',
                   side_effect=AssertionError('raw validation must not decode Base64')) as decode, \
             patch('nomad_navigation.campus_protocol.base64.b64encode',
                   side_effect=AssertionError('raw validation must not encode Base64')) as encode, \
             patch('nomad_navigation.campus_protocol.decode_jpeg_bytes',
                   side_effect=AssertionError('wire decoder used for raw images')):
            frames = _validate_raw_request_and_get_jpegs(request)
            self.assertIs(validate_raw_request(request), request)
            encode.assert_not_called()
            decode.assert_not_called()
        self.assertEqual(request, original)
        self.assertIsInstance(frames, tuple)
        self.assertEqual(len(frames), 4)
        for decoded, frame in zip(frames, request['frames']):
            self.assertIs(decoded, frame['jpeg_bytes'])

    def test_raw_validation_does_not_import_pixel_decoders(self):
        script = (
            'import sys; from nomad_navigation.campus_protocol import validate_raw_request; '
            'request = {"version": 1, "session_id": "00000000-0000-0000-0000-000000000001", '
            '"sequence": 1, "frame_id": "camera_color_optical_frame", '
            '"observation_now_ns": 10800000000, '
            '"frames": [{"stamp_ns": 10000000000+n*250000000, '
            '"jpeg_bytes": bytes((255,216,255,217))} for n in range(4)]}; '
            'assert validate_raw_request(request) is request; '
            'assert not any(n in sys.modules for n in ("PIL", "cv2", "torch", "numpy", "rclpy"))')
        result = subprocess.run([sys.executable, '-c', script],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_raw_jpeg_size_and_immutable_type_boundaries(self):
        class BytesSubclass(bytes):
            pass

        invalid = (None, False, 123, '', '\xff\xd8\xff\xd9', [],
                   bytearray(b'\xff\xd8\xff\xd9'), memoryview(b'\xff\xd8\xff\xd9'),
                   BytesSubclass(b'\xff\xd8\xff\xd9'), b'', b'\xff\xd8', b'not-jpeg',
                   b'\xff\xd8no-end', b'no-start\xff\xd9',
                   b'\xff\xd8' + b'x' * (MAX_JPEG_BYTES - 3) + b'\xff\xd9')
        for index, jpeg in enumerate(invalid):
            request = raw_request_fixture()
            request['frames'][-1]['jpeg_bytes'] = jpeg
            with self.subTest(case=index):
                with self.assertRaises(ProtocolError):
                    validate_raw_request(request)
                with self.assertRaises(ProtocolError):
                    _validate_raw_request_and_get_jpegs(request)
        # This layer checks only the bounded envelope. Pixel validity remains
        # the preparer's responsibility, as on the legacy Base64 path.
        for jpeg in (b'\xff\xd8\xff\xd9',
                     b'\xff\xd8' + b'x' * (MAX_JPEG_BYTES - 4) + b'\xff\xd9'):
            request = raw_request_fixture()
            request['frames'][-1]['jpeg_bytes'] = jpeg
            self.assertIs(validate_raw_request(request), request)
            self.assertIs(_validate_raw_request_and_get_jpegs(request)[-1], jpeg)

    def test_raw_and_wire_metadata_reject_identical_malformed_fields(self):
        mutations = [
            lambda p: p.update(version=True), lambda p: p.update(version=2),
            lambda p: p.update(session_id='not-a-uuid'),
            lambda p: p.update(sequence=True), lambda p: p.update(sequence=0),
            lambda p: p.update(sequence=2**53),
            lambda p: p.update(observation_now_ns=False),
            lambda p: p.update(observation_now_ns=2**63),
            lambda p: p.update(frame_id='base_link'), lambda p: p.update(extra=1),
            lambda p: p['frames'].pop(), lambda p: p.update(frames=tuple(p['frames'])),
            lambda p: p['frames'][0].update(stamp_ns=True),
            lambda p: p['frames'][0].update(stamp_ns=0),
            lambda p: p['frames'][1].update(stamp_ns=p['frames'][0]['stamp_ns']+224_999_999),
            lambda p: p['frames'][-1].update(stamp_ns=p['frames'][-2]['stamp_ns']+750_000_001),
            lambda p: p['frames'][-1].update(unexpected=1),
            lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns']+1_000_000_001),
            lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns']-200_000_001),
            lambda p: p.update(telemetry={'controller_status': {'data': 'x'*32768}}),
            lambda p: p.update(telemetry={'gateway_status': []}),
            lambda p: p.update(telemetry={'unknown': 'field'}),
            lambda p: p.update(telemetry={'odom': {'stamp_ns': 10**10,
                'frame_id': 'odom', 'child_frame_id': 'base_link', 'linear_x': math.nan, 'angular_z': 0.}}),
        ]
        for index, mutate in enumerate(mutations):
            wire, raw = request_fixture(), raw_request_fixture()
            mutate(wire)
            mutate(raw)
            with self.subTest(case=index):
                with self.assertRaises(ProtocolError) as wire_error:
                    validate_request(wire)
                with self.assertRaises(ProtocolError) as raw_error:
                    validate_raw_request(raw)
                self.assertEqual(str(wire_error.exception), str(raw_error.exception))

    def test_raw_and_wire_image_representations_cannot_be_mixed_or_implicitly_sent(self):
        raw, wire = raw_request_fixture(), request_fixture()
        with self.assertRaises(ProtocolError):
            validate_request(raw)
        with self.assertRaises(ProtocolError):
            validate_raw_request(wire)
        with self.assertRaises(ProtocolError):
            encode_json(raw)
        for fixture, validator, other_fixture, other_key in (
                (raw_request_fixture, validate_raw_request, request_fixture, 'jpeg_b64'),
                (request_fixture, validate_request, raw_request_fixture, 'jpeg_bytes')):
            request = fixture()
            request['frames'][-1][other_key] = other_fixture()['frames'][-1][other_key]
            with self.assertRaises(ProtocolError):
                validator(request)
            request = fixture()
            request['frames'][-1] = other_fixture()['frames'][-1]
            with self.assertRaises(ProtocolError):
                validator(request)

    def test_raw_valid_boundary_stamps_and_telemetry_preserve_identity(self):
        for spacing in (225_000_000, 750_000_000):
            for age in (-200_000_000, 1_000_000_000):
                request = raw_request_fixture(sequence=2**53-1)
                for index, frame in enumerate(request['frames']):
                    frame['stamp_ns'] = 10**10 + index*spacing
                request['observation_now_ns'] = request['frames'][-1]['stamp_ns'] + age
                request['telemetry'] = {'controller_status': {'state': 'disarmed'},
                    'gateway_status': {'state': 'disarmed'}, 'odom': {'stamp_ns': 10**10,
                    'frame_id': 'odom', 'child_frame_id': 'base_link', 'linear_x': 0., 'angular_z': 0.}}
                self.assertIs(validate_raw_request(request), request)

    def test_invalid_json(self):
        for raw in (b'{"x":1,"x":2}', b'{"x":NaN}', b'{"x":Infinity}', b'[]', b'\xff', b'{' * 2000):
            with self.subTest(raw=raw[:40]), self.assertRaises(ProtocolError):
                decode_json(raw)
        with self.assertRaises(ProtocolError):
            decode_json(b' ' * (MAX_BODY_BYTES + 1))

    def test_invalid_request_identifiers(self):
        for field, value in [('version', True), ('version', 2), ('session_id', 'not-a-uuid'),
                             ('sequence', True), ('sequence', 0), ('sequence', 2**63),
                             ('frame_id', 'base_link'), ('observation_now_ns', False)]:
            request = request_fixture()
            request[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ProtocolError):
                validate_request(request)

    def test_request_spacing_freshness_and_encoding(self):
        mutations = [lambda p: p['frames'].pop(),
                     lambda p: p['frames'][1].update(stamp_ns=p['frames'][0]['stamp_ns']),
                     lambda p: p['frames'][1].update(stamp_ns=p['frames'][0]['stamp_ns'] + 220_000_000),
                     lambda p: p['frames'][3].update(stamp_ns=p['frames'][2]['stamp_ns'] + 751_000_000),
                     lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns'] + 1_000_000_001),
                     lambda p: p.update(observation_now_ns=p['frames'][-1]['stamp_ns'] - 200_000_001),
                     lambda p: p['frames'][0].update(jpeg_b64='AAAA'),
                     lambda p: p['frames'][0].update(jpeg_b64='***'),
                     lambda p: p.update(unexpected=1)]
        for mutate in mutations:
            request = request_fixture()
            mutate(request)
            with self.assertRaises(ProtocolError):
                validate_request(request)

    def test_response_rejects_wrong_identity_shape_scale_and_finite(self):
        request = request_fixture()
        mutations = [lambda p: p.update(sequence=2), lambda p: p.update(sequence=True),
                     lambda p: p.update(session_id=str(uuid.uuid4())),
                     lambda p: p.update(observation_stamp_ns=1), lambda p: p.update(state='stale'),
                     lambda p: p.update(frame_id='map'), lambda p: p.update(waypoint_scale=1.0),
                     lambda p: p.update(inference_ms=601), lambda p: p.update(inference_ms=math.nan),
                     lambda p: p['trajectories'].pop(), lambda p: p['trajectories'][0].pop(),
                     lambda p: p['trajectories'][0][0].append(1),
                     lambda p: p['trajectories'][0][0].__setitem__(0, True),
                     lambda p: p['trajectories'][0][0].__setitem__(0, math.inf),
                     lambda p: p['trajectories'][0][0].__setitem__(0, 10**1000),
                     lambda p: p['waypoint'].__setitem__(0, 2.0)]
        for mutate in mutations:
            response = response_fixture(request)
            mutate(response)
            with self.assertRaises(ProtocolError):
                validate_response(response, session_id=request['session_id'], sequence=1,
                                  observation_stamp_ns=request['frames'][-1]['stamp_ns'])

    def test_optional_telemetry_bounded(self):
        request = request_fixture()
        request['telemetry'] = {'controller_status': {'state': 'disarmed'},
            'gateway_status': {'state': 'disarmed'}, 'odom': {'stamp_ns': 10**10,
             'frame_id': 'odom', 'child_frame_id': 'base_link', 'linear_x': 0., 'angular_z': 0.}}
        self.assertIs(validate_request(request), request)
        invalid = copy.deepcopy(request)
        invalid['telemetry']['controller_status']['data'] = 'a' * 32768
        with self.assertRaises(ProtocolError):
            validate_request(invalid)
        invalid = copy.deepcopy(request)
        invalid['telemetry']['odom']['linear_x'] = math.nan
        with self.assertRaises(ProtocolError):
            validate_request(invalid)


if __name__ == '__main__':
    unittest.main()
