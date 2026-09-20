"""Hardware-free tests: compaction cannot extend leases or create requests."""
import base64
import copy
import threading
import time
from unittest.mock import Mock, patch

import pytest

from nomad_navigation.campus_client_core import FatalClientError, OneRequestWorker
from nomad_navigation.campus_protocol import ProtocolError
from nomad_navigation.campus_upload_transport import CompactUploadTransport


def payload():
    return {'session_id': 'test-session', 'sequence': 7, 'observation_now_ns': 10_000_000_000,
            'frames': [{'stamp_ns': stamp, 'jpeg_b64': base64.b64encode(b'\xff\xd8original\xff\xd9').decode()}
                       for stamp in (9_250_000_000, 9_500_000_000, 9_750_000_000, 10_000_000_000)]}


class FakePreparer:
    def prepare_request(self, request):
        compact = copy.deepcopy(request)
        for frame in compact['frames']:
            frame['jpeg_b64'] = base64.b64encode(b'\xff\xd8x\xff\xd9').decode()
        return compact


def test_compacts_with_original_start_stamp_session_and_sequence():
    underlying = Mock()
    transport = CompactUploadTransport(underlying, FakePreparer())
    source = payload()
    before = copy.deepcopy(source)
    started = time.monotonic()
    result = transport.infer(source, started=started)
    assert result is underlying.infer.return_value
    compact = underlying.infer.call_args.args[0]
    assert underlying.infer.call_args.kwargs == {'started': started}
    assert source == before
    assert compact['sequence'] == source['sequence']
    assert compact['session_id'] == source['session_id']
    assert [f['stamp_ns'] for f in compact['frames']] == [f['stamp_ns'] for f in source['frames']]
    snapshot = transport.snapshot()
    assert snapshot['counts'] == {'attempts': 1, 'prepared': 1}
    assert snapshot['metrics']['upload_jpeg_bytes']['mean'] < snapshot['metrics']['original_jpeg_bytes']['mean']


def test_preparation_budget_is_part_of_existing_deadline():
    underlying = Mock()
    preparer = FakePreparer()
    transport = CompactUploadTransport(underlying, preparer)
    with patch('nomad_navigation.campus_upload_transport.time.monotonic', side_effect=[10., 10., 10.8, 10.8]):
        with pytest.raises(TimeoutError):
            transport.infer(payload(), started=10.)
    underlying.infer.assert_not_called()


def test_already_expired_request_never_decodes_or_sends():
    underlying, preparer = Mock(), Mock()
    transport = CompactUploadTransport(underlying, preparer)
    with pytest.raises(TimeoutError):
        transport.infer(payload(), started=time.monotonic()-1)
    preparer.prepare_request.assert_not_called()
    underlying.infer.assert_not_called()


def test_invalid_image_is_fatal_without_fallback():
    underlying, preparer = Mock(), Mock()
    preparer.prepare_request.side_effect = ProtocolError('invalid image')
    transport = CompactUploadTransport(underlying, preparer)
    with pytest.raises(FatalClientError):
        transport.infer(payload())
    underlying.infer.assert_not_called()
    assert transport.snapshot()['counts']['preparation_failures'] == 1


def test_cancel_forwards_and_preprocessing_stays_in_single_worker():
    underlying = Mock()
    main_thread = threading.get_ident()
    called_threads = []
    preparer = FakePreparer()
    original = preparer.prepare_request
    def prepare(request):
        called_threads.append(threading.get_ident())
        return original(request)
    preparer.prepare_request = prepare
    transport = CompactUploadTransport(underlying, preparer)
    worker = OneRequestWorker(transport)
    assert worker.submit(payload(), time.monotonic())
    worker._thread.join(timeout=1)
    assert called_threads and called_threads[0] != main_thread
    assert worker.take()[2] is None
    assert worker.close()
    underlying.cancel.assert_called()


def test_metrics_are_bounded_and_snapshot_is_detached():
    transport = CompactUploadTransport(Mock(), FakePreparer())
    transport._metrics['prepare_ms'].extend(range(5000))
    first = transport.snapshot()
    assert first['metrics']['prepare_ms']['count'] == 4096
    first['counts']['attempts'] = 999
    assert transport.snapshot()['counts'] == {}


def test_warmup_cannot_send_an_api_request_or_extend_a_lease():
    underlying, preparer = Mock(), Mock()
    transport = CompactUploadTransport(underlying, preparer)
    transport.warmup()
    preparer.warmup.assert_called_once_with()
    underlying.infer.assert_not_called()
    assert transport.snapshot()['counts'] == {}
    assert transport.snapshot()['warmup_ms'] >= 0.
    assert transport.snapshot()['deadline_sec'] == .8
