"""Bytes-only internal requests, cached encoding, unchanged wire boundary."""
import base64
import copy
import io
import time
from unittest.mock import Mock, patch

import pytest

from nomad_navigation.campus_client_core import CampusClientCore, HTTPSInference
from nomad_navigation.campus_image import UploadImagePreparer, compact_jpeg
from nomad_navigation.campus_protocol import (
    ProtocolError, decode_jpeg_bytes, validate_raw_request, validate_request,
)
from nomad_navigation.campus_upload_transport import CompactUploadTransport


def images():
    from PIL import Image
    result = []
    for i in range(4):
        output = io.BytesIO()
        Image.new('RGB', (640, 480), (20 + 30*i, 80, 120)).save(output, 'JPEG', quality=95)
        result.append(output.getvalue())
    return result


def raw_request(jpegs=None):
    core = CampusClientCore()
    for i, jpeg in enumerate(jpegs or images()):
        stamp = 10_000_000_000 + i*250_000_000
        assert core.add_camera(jpeg, stamp, 'camera_color_optical_frame', stamp, stamp/1e9)
    with patch('base64.b64encode', side_effect=AssertionError('internal Base64 encode')), \
            patch('base64.b64decode', side_effect=AssertionError('internal Base64 decode')):
        request = core.request(10_800_000_000, 10.8, {'controller_status': {'armed': False}})
    return request


def test_core_retains_immutable_bytes_without_base64_work_or_wire_widening():
    jpegs = images()
    request = raw_request(jpegs)
    assert validate_raw_request(request) is request
    for i, frame in enumerate(request['frames']):
        assert set(frame) == {'stamp_ns', 'jpeg_bytes'}
        assert frame['jpeg_bytes'] is jpegs[i]
    with pytest.raises(ProtocolError):
        validate_request(request)


def test_cold_and_hot_raw_paths_do_not_decode_base64_and_cache_encoding():
    request = raw_request()
    before = copy.deepcopy(request)
    preparer = UploadImagePreparer()
    encoder = base64.b64encode
    for expected_hits in (0, 4):
        with patch('base64.b64decode', side_effect=AssertionError('redundant image validation')), \
                patch('base64.b64encode', wraps=encoder) as encode, \
                patch('nomad_navigation.campus_image.compact_jpeg', wraps=compact_jpeg) as codec:
            output = preparer.prepare_request(request)
        assert encode.call_count == codec.call_count == 4-expected_hits
        assert preparer.last_metrics['cache_hits'] == expected_hits
        assert validate_request(output) is output
        assert output['telemetry'] == request['telemetry']
        assert all(set(frame) == {'stamp_ns', 'jpeg_b64'} for frame in output['frames'])
        assert request == before


def test_raw_and_legacy_paths_generate_identical_wire_images():
    request = raw_request()
    legacy = copy.deepcopy(request)
    for frame in legacy['frames']:
        frame['jpeg_b64'] = base64.b64encode(frame.pop('jpeg_bytes')).decode('ascii')
    assert UploadImagePreparer().prepare_request(request) == UploadImagePreparer().prepare_request(legacy)
    output = UploadImagePreparer().prepare_request(request)
    for source, encoded in zip(request['frames'], output['frames']):
        assert decode_jpeg_bytes(encoded['jpeg_b64']) == compact_jpeg(source['jpeg_bytes'])


def test_cache_reuse_keeps_current_identity_and_cannot_be_mutated_through_output():
    request = raw_request()
    preparer = UploadImagePreparer()
    first = preparer.prepare_request(request)
    expected_images = [frame['jpeg_b64'] for frame in first['frames']]
    first['frames'][0]['jpeg_b64'] = 'corrupted'
    first['telemetry']['controller_status']['armed'] = True
    request['sequence'] += 1
    request['observation_now_ns'] += 250_000_000
    for frame in request['frames']:
        frame['stamp_ns'] += 250_000_000
    with patch('base64.b64encode', side_effect=AssertionError('cached encoding')), \
            patch('nomad_navigation.campus_image.compact_jpeg', side_effect=AssertionError('cache miss')):
        second = preparer.prepare_request(request)
    assert [f['jpeg_b64'] for f in second['frames']] == expected_images
    assert [f['stamp_ns'] for f in second['frames']] == [f['stamp_ns'] for f in request['frames']]
    assert second['sequence'] == 2
    assert second['telemetry']['controller_status']['armed'] is False


@pytest.mark.parametrize('mutation', [
    lambda p: p['frames'][1].update(jpeg_b64='extra'),
    lambda p: p['frames'][0].update(jpeg_bytes=bytearray(p['frames'][0]['jpeg_bytes'])),
    lambda p: p['frames'][0].update(jpeg_bytes=b'bad'),
    lambda p: p['frames'][1].update(stamp_ns=p['frames'][0]['stamp_ns']),
    lambda p: p.update(session_id='bad'),
])
def test_raw_bad_boundary_is_rejected_before_codec_or_cache(mutation):
    request = raw_request()
    mutation(request)
    preparer = UploadImagePreparer()
    with patch('nomad_navigation.campus_image.compact_jpeg') as codec:
        with pytest.raises(ProtocolError):
            preparer.prepare_request(request)
    codec.assert_not_called()
    assert not preparer._cache


def test_wire_sender_still_validates_generated_images_once_before_connecting():
    compact = UploadImagePreparer().prepare_request(raw_request())
    with patch('nomad_navigation.campus_client_core.read_token', return_value='test-token'), \
            patch('nomad_navigation.campus_client_core.ssl.create_default_context'):
        transport = HTTPSInference('https://127.0.0.1:8443', 'ca', 'token')
    with patch('nomad_navigation.campus_protocol.decode_jpeg_bytes', wraps=decode_jpeg_bytes) as decode, \
            patch('nomad_navigation.campus_client_core.DeadlineHTTPSConnection',
                  side_effect=ConnectionError('stop before network')):
        with pytest.raises(ConnectionError):
            transport.infer(compact)
    assert decode.call_count == 4
    compact['frames'][0]['jpeg_b64'] = 'invalid'
    with patch('nomad_navigation.campus_client_core.DeadlineHTTPSConnection') as factory:
        with pytest.raises(ProtocolError):
            transport.infer(compact)
    factory.assert_not_called()


def test_upload_uses_real_raw_preparer_and_original_deadline():
    request = raw_request()
    underlying = Mock()
    transport = CompactUploadTransport(underlying)
    transport.warmup()
    started = time.monotonic()
    transport.infer(request, started=started)
    wire = underlying.infer.call_args.args[0]
    assert validate_request(wire) is wire
    assert underlying.infer.call_args.kwargs == {'started': started}
    assert wire['frames'][-1]['stamp_ns'] == request['frames'][-1]['stamp_ns']
    assert transport.snapshot()['metrics']['original_jpeg_bytes']['mean'] == sum(
        len(frame['jpeg_bytes']) for frame in request['frames'])
