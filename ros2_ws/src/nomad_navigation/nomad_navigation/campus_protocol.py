"""Versioned campus API contract; deliberately imports only the Python stdlib.

All observation timestamps belong to the robot.  The server never compares them
with its own wall clock.  The robot must still enforce end-to-end freshness.
"""

import base64
import binascii
import json
import math
import uuid

PROTOCOL_VERSION = 1
CONTEXT_FRAMES = 4
NUM_SAMPLES = 8
TRAJECTORY_STEPS = 8
WAYPOINT_INDEX = 2
WAYPOINT_SCALE = 0.05
MAX_BODY_BYTES = 4 * 1024 * 1024
MAX_JPEG_BYTES = 768 * 1024
INPUT_FRAME = 'camera_color_optical_frame'
OUTPUT_FRAME = 'base_link'
MIN_FRAME_GAP_NS = 225_000_000
MAX_FRAME_GAP_NS = 750_000_000
IMAGE_TIMEOUT_NS = 1_000_000_000
FUTURE_TOLERANCE_NS = 200_000_000
MAX_SEQUENCE = 2**53 - 1
MAX_STAMP_NS = 2**63 - 1


class ProtocolError(ValueError):
    """A request/response cannot safely participate in navigation."""


def _fail(message):
    raise ProtocolError(message)


def _integer(value, name, maximum=MAX_STAMP_NS):
    if type(value) is not int or not 0 < value <= maximum:
        _fail('Invalid ' + name)
    return value


def _finite(value, name):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        _fail('Invalid ' + name)
    return value


def _keys(value, required, optional=()):
    if not isinstance(value, dict) or not set(required) <= value.keys():
        _fail('Missing object fields')
    if not value.keys() <= set(required) | set(optional):
        _fail('Unexpected object fields')


def _session(value):
    if not isinstance(value, str) or len(value) != 36:
        _fail('Invalid session_id')
    try:
        if str(uuid.UUID(value)) != value:
            _fail('session_id must be a canonical UUID')
    except (ValueError, AttributeError):
        _fail('Invalid session_id')


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            _fail('Duplicate JSON field')
        result[key] = value
    return result


def decode_json(raw):
    """Decode bounded UTF-8 JSON, rejecting duplicate keys and NaN/Infinity."""
    if not isinstance(raw, bytes) or len(raw) > MAX_BODY_BYTES:
        _fail('JSON body is too large or not bytes')
    try:
        result = json.loads(raw.decode('utf-8'), object_pairs_hook=_pairs,
                            parse_constant=lambda _: _fail('Nonfinite JSON number'))
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ProtocolError('Invalid JSON body') from exc
    if not isinstance(result, dict):
        _fail('JSON body must be an object')
    return result


def encode_json(payload):
    try:
        raw = json.dumps(payload, allow_nan=False, separators=(',', ':'),
                         ensure_ascii=True).encode('utf-8')
    except (ValueError, TypeError, RecursionError) as exc:
        raise ProtocolError('Cannot encode JSON body') from exc
    if len(raw) > MAX_BODY_BYTES:
        _fail('JSON body is too large')
    return raw


def decode_jpeg_bytes(encoded):
    """Validate canonical-ish bounded base64 without importing an image library."""
    if not isinstance(encoded, str) or not encoded or len(encoded) > 4 * ((MAX_JPEG_BYTES + 2) // 3):
        _fail('Invalid JPEG encoding length')
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ProtocolError('Invalid JPEG base64') from exc
    if len(raw) > MAX_JPEG_BYTES or not raw.startswith(b'\xff\xd8') or not raw.endswith(b'\xff\xd9'):
        _fail('Invalid JPEG envelope')
    return raw


def validate_telemetry(value):
    _keys(value, (), ('controller_status', 'gateway_status', 'odom'))
    for name in ('controller_status', 'gateway_status'):
        if name in value:
            if not isinstance(value[name], dict) or len(encode_json(value[name])) > 32768:
                _fail('Invalid telemetry status')
    if 'odom' in value:
        odom = value['odom']
        _keys(odom, ('stamp_ns', 'frame_id', 'child_frame_id', 'linear_x', 'angular_z'))
        _integer(odom['stamp_ns'], 'odometry stamp')
        for name in ('frame_id', 'child_frame_id'):
            if not isinstance(odom[name], str) or not 0 < len(odom[name]) <= 128:
                _fail('Invalid odometry frame')
        _finite(odom['linear_x'], 'odometry speed')
        _finite(odom['angular_z'], 'odometry angular speed')
    return value


def _validate_request_jpegs(payload, image_field, read_jpeg):
    """Shared metadata checks for the two explicitly separate image forms."""
    _keys(payload, ('version', 'session_id', 'sequence', 'frame_id',
                    'observation_now_ns', 'frames'), ('telemetry',))
    if type(payload['version']) is not int or payload['version'] != PROTOCOL_VERSION:
        _fail('Unsupported protocol version')
    _session(payload['session_id'])
    _integer(payload['sequence'], 'sequence', MAX_SEQUENCE)
    _integer(payload['observation_now_ns'], 'observation_now_ns')
    if payload['frame_id'] != INPUT_FRAME:
        _fail('Unexpected input frame')
    frames = payload['frames']
    if not isinstance(frames, list) or len(frames) != CONTEXT_FRAMES:
        _fail('Expected four context frames')
    previous = None
    decoded = []
    for frame in frames:
        _keys(frame, ('stamp_ns', image_field))
        stamp = _integer(frame['stamp_ns'], 'frame stamp')
        if previous is not None and not MIN_FRAME_GAP_NS <= stamp - previous <= MAX_FRAME_GAP_NS:
            _fail('Invalid context spacing/order')
        previous = stamp
        decoded.append(read_jpeg(frame[image_field]))
    age = payload['observation_now_ns'] - frames[-1]['stamp_ns']
    if not -FUTURE_TOLERANCE_NS <= age <= IMAGE_TIMEOUT_NS:
        _fail('Stale or future latest observation')
    if 'telemetry' in payload:
        validate_telemetry(payload['telemetry'])
    return tuple(decoded)


def _validate_request_and_decode_jpegs(payload):
    """Run every wire-request check and return the validated JPEG bytes.

    Internal preparation callers can reuse these bytes instead of decoding
    the same original Base64 twice. There is no trusted/prevalidated shortcut:
    identity, timestamps, frame bounds, JPEG envelopes and telemetry are all
    checked before the immutable, bounded four-frame tuple is returned.
    Actual JPEG format/dimensions/pixels still require the image decoder.
    """
    return _validate_request_jpegs(payload, 'jpeg_b64', decode_jpeg_bytes)


def _validate_raw_jpeg(jpeg):
    # Exact immutable bytes, not bytearray/memoryview or a bytes subclass with
    # overridden inspection methods. Validation never copies/encodes pixels.
    if (type(jpeg) is not bytes or not 4 <= len(jpeg) <= MAX_JPEG_BYTES
            or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9')):
        _fail('Invalid raw JPEG envelope')
    return jpeg


def _validate_raw_request_and_get_jpegs(payload):
    """Validate bounded internal JPEG bytes without any Base64/image codec.

    Raw frames have exactly stamp_ns and jpeg_bytes. They are an internal
    representation, not another version of the HTTPS JSON contract. All
    metadata and telemetry restrictions match the wire request. A successful
    envelope check does not replace the later JPEG format/dimension decoder.
    """
    return _validate_request_jpegs(payload, 'jpeg_bytes', _validate_raw_jpeg)


def validate_raw_request(payload):
    """Strictly validate an internal raw request and return the same object."""
    _validate_raw_request_and_get_jpegs(payload)
    return payload


def validate_request(payload):
    """Strictly validate a request, preserving the public identity contract."""
    _validate_request_and_decode_jpegs(payload)
    return payload


def validate_response(payload, *, session_id, sequence, observation_stamp_ns):
    _keys(payload, ('version', 'session_id', 'sequence', 'observation_stamp_ns',
                    'frame_id', 'state', 'waypoint', 'trajectories',
                    'inference_ms', 'waypoint_scale'))
    if type(payload['version']) is not int or payload['version'] != PROTOCOL_VERSION:
        _fail('Unsupported protocol version')
    _session(payload['session_id'])
    _integer(payload['sequence'], 'sequence', MAX_SEQUENCE)
    _integer(payload['observation_stamp_ns'], 'observation_stamp_ns')
    if (payload['session_id'] != session_id or payload['sequence'] != sequence
            or payload['observation_stamp_ns'] != observation_stamp_ns):
        _fail('Response does not match the request')
    if payload['frame_id'] != OUTPUT_FRAME or payload['state'] != 'running':
        _fail('Response is not a running base_link prediction')
    if _finite(payload['waypoint_scale'], 'waypoint scale') != WAYPOINT_SCALE:
        _fail('Unexpected waypoint scale')
    if not 0 <= _finite(payload['inference_ms'], 'inference time') <= 600:
        _fail('Inference time exceeds budget')
    trajectories = payload['trajectories']
    if not isinstance(trajectories, list) or len(trajectories) != NUM_SAMPLES:
        _fail('Incorrect trajectory sample count')
    for path in trajectories:
        if not isinstance(path, list) or len(path) != TRAJECTORY_STEPS:
            _fail('Incorrect trajectory step count')
        for xy in path:
            if not isinstance(xy, list) or len(xy) != 2:
                _fail('Incorrect trajectory dimensions')
            for value in xy:
                _finite(value, 'trajectory value')
    waypoint = payload['waypoint']
    if not isinstance(waypoint, list) or len(waypoint) != 2:
        _fail('Incorrect waypoint dimensions')
    for i, value in enumerate(waypoint):
        _finite(value, 'waypoint value')
        if not math.isclose(value, trajectories[0][WAYPOINT_INDEX][i], rel_tol=1e-6, abs_tol=1e-6):
            _fail('Waypoint does not match the selected trajectory')
    return payload
