"""Bounded HTTPS transport and robot-clock guards; standard library only.

No ROS, torch, numpy, command publishing, arming or proxy discovery here.
"""
from collections import Counter, deque
from contextlib import contextmanager
import errno
import http.client
import ipaddress
import math
from pathlib import Path
import socket
import ssl
import stat
import threading
import time
from urllib.parse import urlsplit
import uuid

from nomad_navigation.campus_protocol import (
    MAX_BODY_BYTES, ProtocolError, decode_json, encode_json,
    validate_raw_request, validate_request, validate_response,
)

REQUEST_DEADLINE_SEC = .8
SOURCE_TIMEOUT_NS = 1_000_000_000
FUTURE_NS = 200_000_000
SENSOR_GAP_NS = 750_000_000
SAMPLE_SPACING_NS = 225_000_000
WAYPOINT_TIMEOUT_SEC = .6
MAX_JPEG_BYTES = 768 * 1024
METRIC_WINDOW_LIMIT = 4096
HISTORY_MAX_FRAMES = 96
HISTORY_MAX_BYTES = 64 * 1024 * 1024
HISTORY_WINDOW_NS = 3 * SENSOR_GAP_NS
CONTEXT_TARGET_GAP_NS = 250_000_000
FAULT_TIMING_LIMIT = 32
TRANSIENT_PAUSE_REASONS = frozenset((
    'stale_result', 'waypoint_timeout', 'request_timeout', 'network_error',
    'camera_gap', 'camera_discontinuity', 'context_invalidated'))


def metric_summary(values):
    """Nearest-rank p95 over a bounded window; missing data is null, never NaN."""
    finite = sorted(float(value) for value in values
                    if type(value) in (int, float) and math.isfinite(value))
    if not finite:
        return dict(count=0, min=None, mean=None, p95=None, max=None)
    scale = max(abs(finite[0]), abs(finite[-1]))
    normalized_mean = math.fsum(value/scale/len(finite) for value in finite) if scale else 0.
    mean = max(-1., min(1., normalized_mean))*scale
    return dict(count=len(finite), min=finite[0], mean=mean,
                p95=finite[math.ceil(.95*len(finite))-1], max=finite[-1])


class FatalClientError(RuntimeError):
    """Identity/configuration failures require a deliberate process restart."""


class HTTPResponseError(ConnectionError):
    """A server answered HTTP; retain its numeric status, never its body/token."""
    def __init__(self, status):
        if type(status) is not int or not 100 <= status <= 599:
            raise ValueError('Invalid HTTP response status')
        self.status = status
        super().__init__('Inference service returned HTTP %d' % status)


def server_address(value):
    parsed = urlsplit(value)
    if (parsed.scheme != 'https' or not parsed.hostname or parsed.username is not None
            or parsed.password is not None or parsed.path not in ('', '/')
            or parsed.query or parsed.fragment):
        raise ValueError('Expected https://<numeric server IP>:<port> without credentials/path')
    # Numeric IPs eliminate unbounded libc DNS resolution and proxy interception.
    address = ipaddress.ip_address(parsed.hostname)
    if address.is_unspecified or address.is_multicast:
        raise ValueError('The server must have a unicast address')
    return str(address), parsed.port or 443


def read_token(path):
    target = Path(path)
    info = target.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_size > 4096:
        raise ValueError('Token must be a small regular file')
    if __import__('os').name == 'posix' and info.st_mode & 0o077:
        raise ValueError('Token file permissions must be 0600 or stricter')
    token = target.read_text(encoding='ascii').strip()
    if not 32 <= len(token) <= 512 or any(not (c.isalnum() or c in '-_') for c in token):
        raise ValueError('Token must contain 32..512 URL-safe ASCII characters')
    return token


class DeadlineHTTPSConnection(http.client.HTTPSConnection):
    """Every socket stage shares one deadline, including TLS and body reads."""

    def __init__(self, host, port, context, deadline):
        self.deadline = deadline
        self.cancelled = threading.Event()
        self._active_socket = None
        super().__init__(host, port, context=context, timeout=REQUEST_DEADLINE_SEC)

    def remaining(self):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0 or self.cancelled.is_set():
            raise TimeoutError('HTTPS request exceeded total deadline')
        return remaining

    def connect(self):
        family = socket.AF_INET6 if ':' in self.host else socket.AF_INET
        raw = socket.socket(family, socket.SOCK_STREAM)
        self.sock = raw  # Cancellation also covers TCP connect, not only HTTP.
        self._active_socket = raw
        raw.settimeout(self.remaining())
        raw.connect((self.host, self.port))
        # Match HTTPConnection.connect(): bypassing its DNS-capable connect
        # must not also drop TCP_NODELAY for separate TLS/HTTP writes.
        try:
            raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError as error:
            if error.errno != errno.ENOPROTOOPT:
                raise
        raw.settimeout(self.remaining())
        wrapped = self._context.wrap_socket(raw, server_hostname=self.host,
                                            do_handshake_on_connect=False)
        self.sock = wrapped
        self._active_socket = wrapped
        wrapped.settimeout(self.remaining())
        wrapped.do_handshake()
        wrapped.settimeout(self.remaining())

    def cancel(self):
        self.cancelled.set()
        # getresponse() can detach self.sock when the server says Connection:
        # close, while its buffered response still owns a live file descriptor.
        current = self._active_socket
        if current is not None:
            try:
                current.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                current.close()
            except OSError:
                pass


class HTTPSInference:
    def __init__(self, server_url, ca_file, token_file):
        self.host, self.port = server_address(server_url)
        self.token = read_token(token_file)
        self.context = ssl.create_default_context(cafile=str(ca_file))
        self.context.minimum_version = ssl.TLSVersion.TLSv1_2
        self._lock = threading.Lock()
        self._connection = None
        self._counts = Counter()
        self._metrics = {name: deque(maxlen=METRIC_WINDOW_LIMIT) for name in (
            'encode_ms', 'connect_ms', 'send_ms', 'headers_wait_ms',
            'body_read_ms', 'validate_ms', 'total_transport_ms')}

    @staticmethod
    @contextmanager
    def _measure(timings, name):
        # Diagnostic time is separate from the existing monotonic deadline.
        # Include a failed stage, but never request/response data or errors.
        started = time.perf_counter()
        try:
            yield
        finally:
            timings[name] = (time.perf_counter() - started) * 1000

    def snapshot(self):
        with self._lock:
            counts = dict(self._counts)
            metrics = {name: tuple(values) for name, values in self._metrics.items()}
        return dict(metrics_window_limit=METRIC_WINDOW_LIMIT, counts=counts,
                    metrics={name: metric_summary(values) for name, values in metrics.items()})

    def cancel(self):
        with self._lock:
            connection = self._connection
        if connection is not None:
            connection.cancel()

    def infer(self, request, *, started=None):
        measured_at = time.perf_counter()
        timings = {}
        succeeded = False
        with self._lock:
            self._counts['attempts'] += 1
        try:
            started = time.monotonic() if started is None else started
            deadline = started + REQUEST_DEADLINE_SEC
            with self._measure(timings, 'encode_ms'):
                body = encode_json(validate_request(request))
                if len(body) > MAX_BODY_BYTES:
                    raise ProtocolError('Request body exceeds the protocol limit')
            connection = DeadlineHTTPSConnection(self.host, self.port, self.context, deadline)
            with self._lock:
                self._connection = connection
            timer = threading.Timer(max(0.0, deadline-time.monotonic()), connection.cancel)
            timer.daemon = True
            timer.start()
            try:
                connection.remaining()
                # Split the normal implicit connect from request() for timing;
                # use the same connection, identity checks and total deadline.
                with self._measure(timings, 'connect_ms'):
                    connection.connect()
                with self._measure(timings, 'send_ms'):
                    connection.sock.settimeout(connection.remaining())
                    connection.request('POST', '/v1/infer', body=body, headers={
                        'Authorization': 'Bearer ' + self.token,
                        'Content-Type': 'application/json', 'Accept': 'application/json',
                        'Connection': 'close',
                    })
                with self._measure(timings, 'headers_wait_ms'):
                    connection.sock.settimeout(connection.remaining())
                    response = connection.getresponse()
                    if response.status in (401, 403):
                        raise FatalClientError('API authentication rejected; verify token offline')
                    if response.status != 200:
                        raise HTTPResponseError(response.status)
                    headers = response.getheaders()
                    lengths = [v for k, v in headers if k.lower() == 'content-length']
                    if (len(lengths) != 1 or not lengths[0].isdigit()
                            or not 0 < int(lengths[0]) <= MAX_BODY_BYTES
                            or response.getheader('Transfer-Encoding') is not None
                            or response.getheader('Content-Encoding') not in (None, 'identity')
                            or response.getheader('Content-Type', '').split(';')[0] != 'application/json'):
                        raise FatalClientError('Unexpected API response framing')
                with self._measure(timings, 'body_read_ms'):
                    length = int(lengths[0])
                    chunks = []
                    received = 0
                    while received < length:
                        connection.remaining()
                        chunk = response.read1(min(65536, length-received))
                        if not chunk:
                            raise ConnectionError('Truncated inference response')
                        received += len(chunk)
                        chunks.append(chunk)
                    connection.remaining()
                with self._measure(timings, 'validate_ms'):
                    result = validate_response(decode_json(b''.join(chunks)),
                                               session_id=request['session_id'],
                                               sequence=request['sequence'],
                                               observation_stamp_ns=request['frames'][-1]['stamp_ns'])
                    connection.remaining()
                succeeded = True
                return result
            except ssl.SSLCertVerificationError as error:
                raise FatalClientError('Server TLS certificate/identity verification failed') from error
            except ProtocolError as error:
                raise FatalClientError('Server response failed protocol/identity validation') from error
            finally:
                timer.cancel()
                connection.cancel()
                connection.close()
                with self._lock:
                    if self._connection is connection:
                        self._connection = None
        finally:
            timings['total_transport_ms'] = (time.perf_counter() - measured_at) * 1000
            with self._lock:
                self._counts['succeeded' if succeeded else 'failed'] += 1
                for name, value in timings.items():
                    if math.isfinite(value) and value >= 0:
                        self._metrics[name].append(value)


class OneRequestWorker:
    """Never queue or spawn a second request while the first remains alive."""
    def __init__(self, transport):
        self.transport = transport
        self._thread = None
        self._result = None
        self._lock = threading.Lock()
        self.closed = False

    @property
    def busy(self):
        return self._thread is not None and self._thread.is_alive()

    def submit(self, request, started):
        if self.closed or self.busy or self._result is not None:
            return False
        def execute():
            try:
                result = self.transport.infer(request, started=started)
                item = (request, result, None, time.monotonic())
            except Exception as error:
                item = (request, None, error, time.monotonic())
            with self._lock:
                self._result = item
        self._thread = threading.Thread(target=execute, name='nomad-https', daemon=True)
        self._thread.start()
        return True

    def take(self):
        if self.busy:
            return None
        with self._lock:
            item, self._result = self._result, None
        return item

    def close(self):
        self.closed = True
        self.transport.cancel()
        if self._thread is not None:
            self._thread.join(timeout=REQUEST_DEADLINE_SEC + .1)
        return not self.busy


class CampusClientCore:
    """Latest-context policy; all freshness uses original robot timestamps."""
    def __init__(self, session_id=None, *, latch_faults=False, network_pause_resume=False):
        if type(latch_faults) is not bool:
            raise ValueError('latch_faults must be an explicit boolean')
        self.latch_faults = latch_faults
        if type(network_pause_resume) is not bool:
            raise ValueError('network_pause_resume must be an explicit boolean')
        self.network_pause_resume = network_pause_resume
        self.pause_reason = None
        self.session_id = session_id or str(uuid.uuid4())
        # Raw compressed frames are retained, not irrevocably downsampled on
        # arrival. Each request anchors its 4 Hz context at the latest image.
        self.frames = deque()
        self.history_bytes = 0
        self.sequence = 0
        self.generation = 0
        self.state = 'waiting_context'
        self.fatal = False
        self.last_now = None
        self.last_steady = None
        self.last_seen_stamp = 0
        self.last_arrival = None
        self.last_requested_stamp = 0
        self.pending = None
        self.last_result_stamp = None
        self.last_result_at = None
        self.accepted_results = 0
        self.last_error_type = None
        self.last_http_status = None
        self.rejected_results = 0
        self.rejection_counts = Counter()
        self.fault_events = Counter()
        self.accepted_metrics = {
            name: deque(maxlen=METRIC_WINDOW_LIMIT)
            for name in ('request_elapsed_ms', 'worker_elapsed_ms', 'source_age_ms',
                         'observation_age_at_request_ms', 'inference_ms')
        }
        self._accepted_metric_summary = None
        self.valid_camera_samples = 0
        self.camera_callback_age_ms = deque(maxlen=METRIC_WINDOW_LIMIT)
        self._camera_metric_summary = None
        self.fault_timing = deque(maxlen=FAULT_TIMING_LIMIT)
        self.fault_timing_total = 0

    def _append_history(self, stamp_ns, jpeg):
        size = len(jpeg)
        # Evict BEFORE appending, so every retained-history state obeys all
        # three limits. Immutable JPEG bytes are shared with a selected view.
        while self.frames and (len(self.frames) >= HISTORY_MAX_FRAMES
                               or self.history_bytes + size > HISTORY_MAX_BYTES
                               or stamp_ns-self.frames[0][0] > HISTORY_WINDOW_NS):
            _, old = self.frames.popleft()
            self.history_bytes -= len(old)
        self.frames.append((stamp_ns, bytes(jpeg)))
        self.history_bytes += size

    def select_context(self, now_ns):
        """Latest nonfuture anchor; prefer 250 ms predecessors without dead ends.

        A bounded memoized search checks reachability for exactly four ordered
        frames. It never substitutes a duplicated frame, relaxes the protocol's
        225..750 ms spacing, or falls back to a stale anchor when the newest
        eligible anchor has insufficient history.
        """
        eligible = tuple(frame for frame in self.frames if frame[0] <= now_ns)
        if len(eligible) < 4 or now_ns-eligible[-1][0] > SOURCE_TIMEOUT_NS:
            return None
        cache = {}

        def prefix(index, count):
            key = (index, count)
            if key in cache:
                return cache[key]
            if count == 1:
                return (index,)
            stamp = eligible[index][0]
            candidates = [earlier for earlier in range(index)
                          if SAMPLE_SPACING_NS <= stamp-eligible[earlier][0] <= SENSOR_GAP_NS]
            candidates.sort(key=lambda earlier: (
                abs(stamp-eligible[earlier][0]-CONTEXT_TARGET_GAP_NS),
                -eligible[earlier][0]))
            for earlier in candidates:
                result = prefix(earlier, count-1)
                if result is not None:
                    cache[key] = result + (index,)
                    return cache[key]
            cache[key] = None
            return None

        indices = prefix(len(eligible)-1, 4)
        return tuple(eligible[index] for index in indices) if indices is not None else None

    def _record_fault_timing(self, reason, origin, *, pending=None, completed_at=None):
        """Bounded metadata only: no images, response bodies, paths or tokens."""
        def finite(value):
            try:
                return float(value) if type(value) in (int, float) and math.isfinite(value) else None
            except OverflowError:
                return None

        def elapsed_ms(end, start, divisor=1.):
            if finite(end) is None or finite(start) is None:
                return None
            return finite((end-start)*1000/divisor)

        selected = self.pending if pending is None else pending
        now, steady = self.last_now, self.last_steady
        valid_now = now if type(now) is int and 0 < now <= 2**63-1 else None
        self.fault_timing_total += 1
        self.fault_timing.append(dict(
            reason=reason, origin=origin, sequence=self.sequence, generation=self.generation,
            now_ns=valid_now, steady_sec=finite(steady),
            last_result_stamp_ns=self.last_result_stamp,
            previous_source_age_ms=elapsed_ms(now, self.last_result_stamp, 1e9),
            previous_acceptance_age_ms=elapsed_ms(steady, self.last_result_at),
            pending_sequence=selected['sequence'] if selected else None,
            pending_stamp_ns=selected['stamp_ns'] if selected else None,
            pending_elapsed_ms=elapsed_ms(steady, selected['started']) if selected else None,
            incoming_source_age_ms=elapsed_ms(now, selected['stamp_ns'], 1e9) if selected else None,
            worker_elapsed_ms=elapsed_ms(completed_at, selected['started']) if selected else None,
            completed_result_wait_ms=elapsed_ms(steady, completed_at),
        ))

    def _reject_result(self, reason):
        # One outcome per completed request, separate from repeated watchdog
        # checks and state transitions. It never changes the state machine.
        self.rejected_results += 1
        self.rejection_counts[reason] += 1

    def _record_accepted(self, pending, response, completed_at, now_ns, steady):
        self._accepted_metric_summary = None
        values = dict(request_elapsed_ms=(steady-pending['started'])*1000,
                      worker_elapsed_ms=(completed_at-pending['started'])*1000,
                      source_age_ms=(now_ns-pending['stamp_ns'])/1_000_000,
                      observation_age_at_request_ms=pending['observation_age_at_request_ms'],
                      inference_ms=response['inference_ms'])
        for name, value in values.items():
            if type(value) in (int, float) and math.isfinite(value):
                self.accepted_metrics[name].append(float(value))

    def fail(self, reason, *, fatal=False, clear=False, origin='state_guard',
             pending=None, completed_at=None):
        recoverable = self.network_pause_resume and reason in TRANSIENT_PAUSE_REASONS and not fatal
        if not self.fatal:
            previous_reason = self.pause_reason if self.state == 'network_wait' else self.state
            if reason != 'stopped' and (previous_reason != reason or origin == 'incoming_result'):
                self._record_fault_timing(reason, origin, pending=pending, completed_at=completed_at)
            if previous_reason != reason and reason != 'stopped':
                self.fault_events[reason] += 1
            self.state = 'network_wait' if recoverable else reason
            self.pause_reason = reason if recoverable else None
        # Default formal output remains fail-closed. The explicit speed-profile
        # pause policy keeps only enumerated transient waits alive; the local
        # controller independently expires data and requires new predictions.
        self.fatal = self.fatal or fatal or (
            self.latch_faults and not recoverable and reason not in ('waiting_context', 'stopped'))
        self.last_result_stamp = self.last_result_at = None
        if clear:
            self.frames.clear()
            self.history_bytes = 0
            self.generation += 1

    def clock(self, now_ns, steady):
        if (type(now_ns) is not int or now_ns <= 0 or not math.isfinite(steady) or steady < 0
                or (self.last_now is not None and now_ns < self.last_now)
                or (self.last_steady is not None and steady < self.last_steady)):
            self.fail('clock_rollback', fatal=True, clear=True)
        self.last_now, self.last_steady = now_ns, steady
        return not self.fatal

    def add_camera(self, jpeg, stamp_ns, frame_id, now_ns, steady):
        if not self.clock(now_ns, steady):
            return False
        if (type(stamp_ns) is not int or stamp_ns <= 0 or frame_id != 'camera_color_optical_frame'
                or not -FUTURE_NS <= now_ns-stamp_ns <= SOURCE_TIMEOUT_NS
                or not 4 <= len(jpeg) <= MAX_JPEG_BYTES
                or jpeg[:2] != b'\xff\xd8' or jpeg[-2:] != b'\xff\xd9'):
            self.fail('invalid_camera', clear=True)
            return False
        if stamp_ns <= self.last_seen_stamp:
            self.fail('camera_replay', clear=True)
            return False
        discontinuity = (self.last_seen_stamp and stamp_ns-self.last_seen_stamp > SENSOR_GAP_NS)
        if self.last_arrival is not None and steady-self.last_arrival > .75:
            discontinuity = True
        self.last_seen_stamp, self.last_arrival = stamp_ns, steady
        if discontinuity or (self.frames and stamp_ns-self.frames[-1][0] > SENSOR_GAP_NS):
            self.fail('camera_discontinuity', clear=True)
            if self.fatal:
                return False
        self._append_history(stamp_ns, jpeg)
        # All timestamp-valid retained callbacks are now eligible for a future
        # request-aligned context, including high-rate camera observations.
        self.valid_camera_samples += 1
        self.camera_callback_age_ms.append((now_ns-stamp_ns)/1_000_000)
        self._camera_metric_summary = None
        if len(self.frames) < 4:
            self.state = 'waiting_context'
        return True

    def watchdog(self, now_ns, steady):
        if not self.clock(now_ns, steady):
            return
        if (self.last_arrival is not None and steady-self.last_arrival > .75):
            self.fail('camera_gap', clear=bool(self.frames))
        if self.pending is not None and steady-self.pending['started'] >= REQUEST_DEADLINE_SEC:
            self.fail('request_timeout', origin='watchdog_request')
        if self.last_result_stamp is not None:
            if not -FUTURE_NS <= now_ns-self.last_result_stamp <= SOURCE_TIMEOUT_NS:
                self.fail('stale_result', origin='watchdog_previous_prediction')
            elif steady-self.last_result_at > WAYPOINT_TIMEOUT_SEC:
                self.fail('waypoint_timeout', origin='watchdog_previous_prediction')

    def request(self, now_ns, steady, telemetry=None):
        self.watchdog(now_ns, steady)
        if self.fatal or self.pending is not None:
            return None
        context = self.select_context(now_ns)
        if context is None or context[-1][0] <= self.last_requested_stamp:
            return None
        self.sequence += 1
        payload = dict(version=1, session_id=self.session_id, sequence=self.sequence,
                       frame_id='camera_color_optical_frame', observation_now_ns=now_ns,
                       # Internal work items retain immutable JPEG bytes. Only
                       # the final compact upload needs the JSON/Base64 format.
                       frames=[dict(stamp_ns=stamp, jpeg_bytes=jpeg)
                               for stamp, jpeg in context])
        if telemetry:
            payload['telemetry'] = telemetry
        validate_raw_request(payload)
        self.last_requested_stamp = context[-1][0]
        self.pending = dict(sequence=self.sequence, stamp_ns=self.last_requested_stamp,
                            started=steady, generation=self.generation,
                            context_stamps_ns=tuple(stamp for stamp, _ in context),
                            observation_age_at_request_ms=(now_ns-self.last_requested_stamp)/1_000_000)
        return payload

    def complete(self, response, error, completed_at, now_ns, steady):
        pending, self.pending = self.pending, None
        if not self.clock(now_ns, steady) or pending is None:
            self._reject_result(self.state if self.fatal else 'unexpected_completion')
            return None
        if error is not None:
            self.last_error_type = type(error).__name__
            self.last_http_status = error.status if isinstance(error, HTTPResponseError) else None
            # Keep the original network_error state but classify an elapsed
            # deadline/TimeoutError separately for the performance audit.
            if isinstance(error, (FatalClientError, ProtocolError)):
                rejection = 'identity_or_protocol_error'
            elif isinstance(error, HTTPResponseError):
                # Receipt of 408/504 is evidence of a responding HTTP server,
                # not proof of a Wi-Fi disconnect. Control still fails closed
                # under the unchanged network_error state below.
                rejection = 'http_%d' % error.status
            elif (isinstance(error, TimeoutError)
                  or completed_at-pending['started'] >= REQUEST_DEADLINE_SEC
                  or steady-pending['started'] >= REQUEST_DEADLINE_SEC):
                rejection = 'request_timeout'
            else:
                rejection = 'network_error'
            self._reject_result(rejection)
            hard_error = isinstance(error, (FatalClientError, ProtocolError))
            if self.network_pause_resume:
                hard_error = hard_error or (isinstance(error, HTTPResponseError)
                    and error.status not in (408, 429, 500, 502, 503, 504))
                hard_error = hard_error or not isinstance(error, (OSError, HTTPResponseError))
            self.fail('identity_or_protocol_error' if hard_error else 'network_error', fatal=hard_error,
                      origin='incoming_result', pending=pending, completed_at=completed_at)
            return None
        try:
            validate_response(response, session_id=self.session_id, sequence=pending['sequence'],
                              observation_stamp_ns=pending['stamp_ns'])
        except (ValueError, TypeError, KeyError):
            self._reject_result('identity_or_protocol_error')
            self.fail('identity_or_protocol_error', fatal=True, origin='incoming_result',
                      pending=pending, completed_at=completed_at)
            return None
        if (completed_at-pending['started'] >= REQUEST_DEADLINE_SEC
                or steady-pending['started'] >= REQUEST_DEADLINE_SEC):
            self._reject_result('request_timeout')
            self.fail('request_timeout', origin='incoming_result', pending=pending, completed_at=completed_at)
            return None
        if pending['generation'] != self.generation:
            self._reject_result('context_invalidated')
            self.fail('context_invalidated', origin='incoming_result', pending=pending, completed_at=completed_at)
            return None
        if (self.last_arrival is None or steady-self.last_arrival > .75
                or not -FUTURE_NS <= now_ns-pending['stamp_ns'] <= SOURCE_TIMEOUT_NS):
            self._reject_result('stale_result')
            self.fail('stale_result', origin='incoming_result', pending=pending, completed_at=completed_at)
            return None
        self.last_result_stamp, self.last_result_at = pending['stamp_ns'], steady
        self.state = 'running'
        self.pause_reason = None
        self.accepted_results += 1
        self._record_accepted(pending, response, completed_at, now_ns, steady)
        return response

    def snapshot(self):
        # Status is published at 20 Hz; sort only when a new accepted result
        # changes the bounded window, not on every watchdog heartbeat.
        if self._accepted_metric_summary is None:
            self._accepted_metric_summary = {
                name: metric_summary(values) for name, values in self.accepted_metrics.items()}
        if self._camera_metric_summary is None:
            self._camera_metric_summary = metric_summary(self.camera_callback_age_ms)
        return dict(state=self.state, session_id=self.session_id, sequence=self.sequence,
                    fatal=self.fatal, latch_faults=self.latch_faults,
                    network_pause_resume=self.network_pause_resume, pause_reason=self.pause_reason,
                    accepted_results=self.accepted_results,
                    last_error_type=self.last_error_type, last_http_status=self.last_http_status,
                    auto_arm=False,
                    rejected_results=self.rejected_results,
                    rejection_counts=dict(self.rejection_counts),
                    fault_events=dict(self.fault_events),
                    accepted_metrics_window_limit=METRIC_WINDOW_LIMIT,
                    context_sampling='request_aligned_latest_250ms',
                    history_frames=len(self.frames), history_bytes=self.history_bytes,
                    history_limits=dict(frames=HISTORY_MAX_FRAMES, bytes=HISTORY_MAX_BYTES,
                                        span_ns=HISTORY_WINDOW_NS),
                    fault_timing_total=self.fault_timing_total, fault_timing_limit=FAULT_TIMING_LIMIT,
                    fault_timing=[dict(event) for event in self.fault_timing],
                    valid_camera_samples=self.valid_camera_samples,
                    camera_age_sampling='all_timestamp_valid_history_callbacks',
                    camera_callback_age_ms=dict(self._camera_metric_summary),
                    accepted_metrics={name: dict(values)
                                      for name, values in self._accepted_metric_summary.items()},
                    trajectory_layout=dict(samples=8, steps=8, waypoint_index=2,
                                           scale_applied=.05, frame_id='base_link'))
