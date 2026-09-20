"""Compact camera uploads inside the single request worker, within its deadline."""
from collections import Counter, deque
import threading
import time

from .campus_client_core import FatalClientError, REQUEST_DEADLINE_SEC, metric_summary
from .campus_protocol import ProtocolError, decode_jpeg_bytes, encode_json


class CompactUploadTransport:
    """No ROS callbacks, image timestamps, retries, or motion authority here."""

    def __init__(self, transport, preparer=None):
        if preparer is None:
            from .campus_image import UploadImagePreparer
            preparer = UploadImagePreparer()
        self.transport = transport
        self.preparer = preparer
        self._lock = threading.Lock()
        self._counts = Counter()
        self._warmup_ms = None
        self._metrics = {name: deque(maxlen=4096) for name in (
            'prepare_ms', 'original_jpeg_bytes', 'upload_jpeg_bytes', 'request_body_bytes')}

    def cancel(self):
        self.transport.cancel()

    def warmup(self):
        """Load codecs before creating camera subscriptions, never during a lease."""
        started = time.monotonic()
        self.preparer.warmup()
        with self._lock:
            self._warmup_ms = (time.monotonic()-started)*1000

    @staticmethod
    def _remaining(started):
        if time.monotonic() - started >= REQUEST_DEADLINE_SEC:
            raise TimeoutError('Image preparation exceeded the existing request deadline')

    def infer(self, request, *, started=None):
        started = time.monotonic() if started is None else started
        with self._lock:
            self._counts['attempts'] += 1
        self._remaining(started)
        prepared_at = time.monotonic()
        try:
            compact = self.preparer.prepare_request(request)
        except (ProtocolError, ValueError, OSError) as error:
            with self._lock:
                self._counts['preparation_failures'] += 1
            # Never fall back to an invalid/full-size frame after a failed decode.
            raise FatalClientError('Camera image preparation failed validation') from error
        prepared_ms = (time.monotonic() - prepared_at) * 1000
        image_metrics = getattr(self.preparer, 'last_metrics', None)
        if isinstance(image_metrics, dict) and image_metrics.get('complete') is True:
            # Reuse counts already obtained from validated JPEGs; do not decode
            # all original base64 frames a second time just for telemetry.
            original_bytes = image_metrics['source_jpeg_bytes']
            upload_bytes = image_metrics['encoded_jpeg_bytes']
        else:
            original_bytes = sum(len(f['jpeg_bytes']) if 'jpeg_bytes' in f
                                 else len(decode_jpeg_bytes(f['jpeg_b64']))
                                 for f in request['frames'])
            upload_bytes = sum(len(decode_jpeg_bytes(f['jpeg_b64'])) for f in compact['frames'])
        values = {
            'prepare_ms': prepared_ms,
            'original_jpeg_bytes': original_bytes,
            'upload_jpeg_bytes': upload_bytes,
            'request_body_bytes': len(encode_json(compact)),
        }
        self._remaining(started)
        with self._lock:
            self._counts['prepared'] += 1
            for name, value in values.items():
                self._metrics[name].append(value)
        # Preserve the original worker deadline: preprocessing cannot buy time.
        return self.transport.infer(compact, started=started)

    def snapshot(self):
        with self._lock:
            result = dict(mode='bounded_compact_jpeg', deadline_sec=REQUEST_DEADLINE_SEC,
                          warmup_ms=self._warmup_ms,
                          metrics_window_limit=4096, counts=dict(self._counts),
                          metrics={name: metric_summary(values) for name, values in self._metrics.items()})
        snapshot = getattr(self.transport, 'snapshot', None)
        if callable(snapshot):
            http_transport = snapshot()
            if isinstance(http_transport, dict):
                result['http_transport'] = http_transport
        return result
