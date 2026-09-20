"""Bounded HTTP diagnostics do not change the navigation transport contract."""
import base64
import copy
import json
import threading
import time
from unittest.mock import Mock, patch

import pytest

from nomad_navigation.campus_client_core import (
    FatalClientError, HTTPSInference, METRIC_WINDOW_LIMIT, REQUEST_DEADLINE_SEC,
)
from nomad_navigation.campus_protocol import ProtocolError, encode_json
from nomad_navigation.campus_upload_transport import CompactUploadTransport


def payload():
    return dict(version=1, session_id='12345678-1234-1234-1234-123456789abc',
                sequence=1, frame_id='camera_color_optical_frame',
                observation_now_ns=10_750_000_000,
                frames=[dict(stamp_ns=stamp,
                             jpeg_b64=base64.b64encode(b'\xff\xd8test\xff\xd9').decode())
                        for stamp in (10_000_000_000, 10_250_000_000,
                                      10_500_000_000, 10_750_000_000)])


class Response:
    status = 200

    def __init__(self, request):
        paths = [[[.1, .01] for _ in range(8)] for _ in range(8)]
        self.result = dict(version=1, session_id=request['session_id'],
                           sequence=request['sequence'], frame_id='base_link',
                           observation_stamp_ns=request['frames'][-1]['stamp_ns'],
                           state='running', waypoint=paths[0][2], trajectories=paths,
                           inference_ms=100., waypoint_scale=.05)
        self.raw = encode_json(self.result)
        self.position = 0

    def getheaders(self):
        return [('Content-Length', str(len(self.raw))), ('Content-Type', 'application/json')]

    def getheader(self, name, default=None):
        return dict(self.getheaders()).get(name, default)

    def read1(self, count):
        block = self.raw[self.position:self.position+count]
        self.position += len(block)
        return block


class Connection:
    def __init__(self, request):
        self.response = Response(request)
        self.sock = Mock()
        self.deadline = None
        self.cancelled = False
        self.connect = Mock()
        self.request = Mock()
        self.close = Mock()

    def remaining(self):
        remaining = self.deadline-time.monotonic()
        if remaining <= 0 or self.cancelled:
            raise TimeoutError('deadline')
        return remaining

    def getresponse(self):
        return self.response

    def cancel(self):
        self.cancelled = True


@pytest.fixture
def transport():
    with patch('nomad_navigation.campus_client_core.read_token', return_value='secret-token'), \
            patch('nomad_navigation.campus_client_core.ssl.create_default_context'):
        return HTTPSInference('https://127.0.0.1:8443', 'private-ca-file', 'private-token-file')


def run(transport, request, connection, *, started=None):
    def factory(host, port, context, deadline):
        connection.deadline = deadline
        return connection
    with patch('nomad_navigation.campus_client_core.DeadlineHTTPSConnection', side_effect=factory), \
            patch('nomad_navigation.campus_client_core.threading.Timer') as timer:
        result = transport.infer(request, started=started)
    return result, timer


def test_all_success_phases_are_numeric_bounded_and_contain_no_sensitive_data(transport):
    request = payload()
    original = copy.deepcopy(request)
    connection = Connection(request)
    result, timer = run(transport, request, connection)
    assert request == original
    assert result == connection.response.result
    connection.connect.assert_called_once_with()
    connection.request.assert_called_once()
    assert connection.request.call_args.kwargs['headers']['Connection'] == 'close'
    assert connection.cancelled
    connection.close.assert_called_once_with()
    timer.return_value.cancel.assert_called_once_with()
    snapshot = transport.snapshot()
    assert snapshot['counts'] == dict(attempts=1, succeeded=1)
    assert snapshot['metrics_window_limit'] == 4096
    assert set(snapshot['metrics']) == {
        'encode_ms', 'connect_ms', 'send_ms', 'headers_wait_ms',
        'body_read_ms', 'validate_ms', 'total_transport_ms'}
    for value in snapshot['metrics'].values():
        assert value['count'] == 1
        assert value['min'] >= 0
    encoded = json.dumps(snapshot)
    for secret in ('secret-token', '127.0.0.1', 'private-ca-file', 'jpeg_b64', 'session_id'):
        assert secret not in encoded
    snapshot['counts']['attempts'] = 100
    assert transport.snapshot()['counts']['attempts'] == 1


def test_failed_connect_is_counted_and_closed_without_retry(transport):
    request = payload()
    connection = Connection(request)
    connection.connect.side_effect = ConnectionError('private error data')
    with pytest.raises(ConnectionError):
        run(transport, request, connection)
    connection.connect.assert_called_once_with()
    connection.request.assert_not_called()
    connection.close.assert_called_once_with()
    assert connection.cancelled
    snapshot = transport.snapshot()
    assert snapshot['counts'] == dict(attempts=1, failed=1)
    assert snapshot['metrics']['connect_ms']['count'] == 1
    assert snapshot['metrics']['send_ms']['count'] == 0
    assert snapshot['metrics']['total_transport_ms']['count'] == 1
    assert 'private error data' not in json.dumps(snapshot)


def test_invalid_request_records_encode_failure_without_socket(transport):
    with patch('nomad_navigation.campus_client_core.DeadlineHTTPSConnection') as factory:
        with pytest.raises(ProtocolError):
            transport.infer({})
    factory.assert_not_called()
    snapshot = transport.snapshot()
    assert snapshot['counts'] == dict(attempts=1, failed=1)
    assert snapshot['metrics']['encode_ms']['count'] == 1
    assert snapshot['metrics']['connect_ms']['count'] == 0


def test_expired_original_deadline_never_connects_or_sends(transport):
    request = payload()
    connection = Connection(request)
    started = time.monotonic()-1.
    with pytest.raises(TimeoutError):
        run(transport, request, connection, started=started)
    assert connection.deadline == started+REQUEST_DEADLINE_SEC
    connection.connect.assert_not_called()
    connection.request.assert_not_called()
    assert transport.snapshot()['counts'] == dict(attempts=1, failed=1)


def test_prepare_time_cannot_be_renewed_by_explicit_connect(transport):
    request = payload()
    connection = Connection(request)
    clock = [10.]
    connection.connect.side_effect = lambda: clock.__setitem__(0, 10.11)
    with patch('nomad_navigation.campus_client_core.time.monotonic', side_effect=lambda: clock[0]):
        with pytest.raises(TimeoutError):
            run(transport, request, connection, started=9.3)
    assert connection.deadline == pytest.approx(10.1)
    connection.connect.assert_called_once_with()
    connection.request.assert_not_called()
    assert connection.cancelled
    assert REQUEST_DEADLINE_SEC == .8


def test_cancel_remains_available_during_explicit_connect(transport):
    request = payload()
    connection = Connection(request)
    entered, release = threading.Event(), threading.Event()
    caught = []
    def connect():
        entered.set()
        assert release.wait(.5)
    connection.connect.side_effect = connect
    def invoke():
        try:
            run(transport, request, connection)
        except Exception as error:
            caught.append(error)
    worker = threading.Thread(target=invoke)
    worker.start()
    try:
        assert entered.wait(.5)
        transport.cancel()
    finally:
        release.set()
        worker.join(.5)
    assert not worker.is_alive()
    assert len(caught) == 1 and isinstance(caught[0], TimeoutError)
    connection.request.assert_not_called()
    assert transport._connection is None


def test_response_identity_failure_keeps_validation_timing_and_remains_fatal(transport):
    request = payload()
    connection = Connection(request)
    connection.response.result['sequence'] += 1
    connection.response.raw = encode_json(connection.response.result)
    with pytest.raises(FatalClientError):
        run(transport, request, connection)
    snapshot = transport.snapshot()
    assert snapshot['counts'] == dict(attempts=1, failed=1)
    assert snapshot['metrics']['validate_ms']['count'] == 1
    assert connection.cancelled


def test_metric_windows_do_not_grow_and_snapshot_cannot_change_underlying_values(transport):
    for values in transport._metrics.values():
        values.extend(range(METRIC_WINDOW_LIMIT+100))
    snapshot = transport.snapshot()
    assert all(value['count'] == 4096 for value in snapshot['metrics'].values())
    snapshot['metrics']['encode_ms']['min'] = -1.
    assert transport.snapshot()['metrics']['encode_ms']['min'] == 100.


def test_compact_snapshot_adds_only_http_diagnostics_without_sending(transport):
    preparer = Mock()
    compact = CompactUploadTransport(transport, preparer)
    with patch.object(transport, 'infer') as infer:
        snapshot = compact.snapshot()
    assert snapshot['http_transport'] == transport.snapshot()
    assert snapshot['http_transport']['counts'] == {}
    assert snapshot['deadline_sec'] == .8
    preparer.prepare_request.assert_not_called()
    infer.assert_not_called()
