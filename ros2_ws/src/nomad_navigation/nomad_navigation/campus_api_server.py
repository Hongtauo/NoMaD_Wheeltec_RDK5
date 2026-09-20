"""Bounded, authenticated HTTPS NoMaD inference; never controls ROS or motors.

Run with ``python -m nomad_navigation.campus_api_server --help``.  TLS is
mandatory; provision the CA/certificate out of band and never use insecure TLS.
"""

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import hmac
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
import json
import math
from pathlib import Path
import signal
import socket
from socketserver import ThreadingMixIn
import ssl
import threading
import time
import uuid

from .campus_protocol import (
    CONTEXT_FRAMES, MAX_BODY_BYTES, NUM_SAMPLES, PROTOCOL_VERSION,
    TRAJECTORY_STEPS, WAYPOINT_INDEX, WAYPOINT_SCALE, ProtocolError,
    _validate_request_and_decode_jpegs, decode_json, encode_json, validate_response,
)


class APIError(Exception):
    def __init__(self, status, code):
        self.status, self.code = status, code
        super().__init__(code)


class AuditLog:
    """Bounded-size records: no request bodies, tokens, image pixels or credentials."""

    def __init__(self, output_root):
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.run_dir = Path(output_root) / ('server__' + stamp + '__' + uuid.uuid4().hex[:8])
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self._lock = threading.Lock()
        self._stream = (self.run_dir / 'events.jsonl').open('x', encoding='utf-8', buffering=1)
        self.count = 0
        self._closed = False

    def write(self, event, **fields):
        # No caller-provided request dictionary is ever merged into a log record.
        timings = {'body_read_ms', 'json_decode_ms', 'service_ms', 'elapsed_ms'}
        allowed = {'sequence', 'status', 'code', 'inference_ms', 'runtime', 'session_hash',
                   'declared_body_bytes', 'body_bytes', 'phase'} | timings
        if not set(fields) <= allowed:
            raise ValueError('Unsafe audit fields')
        for name in timings:
            value = fields.get(name)
            if value is not None and (type(value) not in (int, float)
                                      or not math.isfinite(value) or not 0 <= value <= 2**53):
                raise ValueError('Unsafe audit timing')
        for name, maximum in (('declared_body_bytes', 9_999_999_999), ('body_bytes', MAX_BODY_BYTES)):
            value = fields.get(name)
            if value is not None and (type(value) is not int or not 0 <= value <= maximum):
                raise ValueError('Unsafe audit byte count')
        if 'phase' in fields and fields['phase'] not in {
                'request_target', 'authorization', 'headers', 'body', 'json',
                'service', 'response', 'complete'}:
            raise ValueError('Unsafe audit phase')
        with self._lock:
            # HTTP handlers are daemon threads; a late final diagnostic after
            # server shutdown must not write to an already closed stream.
            if self._closed or self.count >= 1_000_000:
                return False
            record = {'utc': datetime.now(timezone.utc).isoformat(), 'event': event, **fields}
            self._stream.write(json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n')
            self.count += 1
            return True

    def close(self):
        with self._lock:
            if not self._closed:
                self._closed = True
                self._stream.close()


def decode_images(payload):
    """Independently validate a request and decode its bounded JPEG images."""
    return _decode_validated_jpegs(_validate_request_and_decode_jpegs(payload))


def _decode_validated_jpegs(jpegs):
    """Decode only the immutable bytes produced by complete request validation.

    This private worker path does not accept a request or a validation flag.
    Actual JPEG format, dimensions and pixels still receive the same checks.
    """
    import numpy as np
    from PIL import Image

    result = []
    for raw in jpegs:
        try:
            with Image.open(io.BytesIO(raw)) as source:
                if (source.format != 'JPEG' or source.width < 16 or source.height < 16
                        or source.width > 1920 or source.height > 1920
                        or source.width * source.height > 1920 * 1080):
                    raise ProtocolError('Unsupported JPEG dimensions/format')
                source.load()
                result.append(np.asarray(source.convert('RGB')).copy())
        except (OSError, ValueError, Image.DecompressionBombError) as exc:
            raise ProtocolError('Unable to decode JPEG') from exc
    return result


class CampusInferenceService:
    """Dependency-injected, one-model service with no GPU waiting queue.

    A failed attempt consumes its sequence.  Sessions cannot steal an active
    lease, and retired IDs are never resurrected by evicting replay history.
    Restarting a client requires a new UUID and a five-second idle lease gap.
    """

    def __init__(self, backend, token, *, image_decoder=decode_images, on_result=None,
                 audit=None, clock=time.monotonic, budget_seconds=0.6,
                 session_lease_seconds=5.0, max_sessions=128, backend_factory=None):
        if not isinstance(token, str) or not 32 <= len(token) <= 256 or not token.isascii() or any(c.isspace() for c in token):
            raise ValueError('Bearer token must be 32..256 ASCII non-whitespace characters')
        if (backend is None) == (backend_factory is None):
            raise ValueError('Supply one backend or one backend_factory')
        if backend is not None:
            self._validate_backend(backend)
        if not 0 < budget_seconds <= 0.6 or not 1 <= max_sessions <= 1024 or session_lease_seconds <= 0:
            raise ValueError('Invalid service bounds')
        self.backend, self._token = backend, token
        self.image_decoder, self.on_result, self.audit = image_decoder, on_result, audit
        self.clock, self.budget_seconds = clock, budget_seconds
        self.session_lease_seconds, self.max_sessions = session_lease_seconds, max_sessions
        self._gpu = threading.Lock()
        self._sessions = {}
        self._active = None
        self._last_activity = -float('inf')
        self.ready = False
        self._condition = threading.Condition()
        self._slot = None
        self._active_job = None
        self._closing = False
        self._startup_done = threading.Event()
        self._startup_error = None
        self._worker = threading.Thread(target=self._worker_loop, args=(backend_factory,),
                                        name='campus-inference', daemon=True)
        self._worker.start()
        try:
            # Factory loading and its warmup run in the very same thread that
            # owns subsequent inference. No real backend is warmed per request.
            if not self._startup_done.wait(120):
                raise RuntimeError('Inference worker startup deadline exceeded')
            if self._startup_error is not None:
                raise self._startup_error
        except BaseException:
            self.close(timeout=1.)
            raise

    @staticmethod
    def _validate_backend(backend):
        if (backend.context_frames != CONTEXT_FRAMES or backend.num_samples != NUM_SAMPLES
                or backend.config['len_traj_pred'] != TRAJECTORY_STEPS
                or backend.config['image_size'] != [96, 96]):
            raise ValueError('Backend is not the fixed 4-frame / 8x8 / 96x96 protocol model')

    def _worker_loop(self, backend_factory):
        try:
            if backend_factory is not None:
                self.backend = backend_factory()
            self._validate_backend(self.backend)
            with self._condition:
                self.ready = not self._closing
        except BaseException as exc:
            self._startup_error = exc
            return
        finally:
            self._startup_done.set()
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._slot is not None or self._closing)
                    if self._slot is None:
                        return
                    job, self._slot = self._slot, None
                try:
                    job['result'] = self._execute(job)
                except BaseException as exc:
                    # Transfer an exception to the originating HTTP handler;
                    # even a backend SystemExit must not strand the GPU lock.
                    job['error'] = exc if isinstance(exc, APIError) else APIError(500, 'inference_failed')
                finally:
                    with self._condition:
                        self._active_job = None
                        # Only this completion path releases a dispatched job's
                        # GPU lock. HTTP timeout/close cannot make it available.
                        self._gpu.release()
                        job['done'].set()
        finally:
            self.ready = False

    def close(self, timeout=2.):
        """Cancel delivery and join boundedly; False means work is still alive.

        CUDA/Python backend calls cannot safely be killed. If one is stuck its
        lock remains owned, no further job can run, and callers see not-ready.
        """
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError('Invalid worker close timeout')
        with self._condition:
            self.ready = False
            self._closing = True
            if self._active_job is not None:
                self._active_job['cancelled'].set()
            self._condition.notify_all()
        if threading.current_thread() is self._worker:
            return False
        self._worker.join(timeout=timeout)
        return not self._worker.is_alive()

    def authenticate(self, value):
        return isinstance(value, str) and hmac.compare_digest(value.encode('utf-8'), ('Bearer ' + self._token).encode('ascii'))

    def _reserve(self, request):
        session, sequence = request['session_id'], request['sequence']
        now = self.clock()
        latest = request['frames'][-1]['stamp_ns']
        observation_now = request['observation_now_ns']
        if session != self._active:
            if session in self._sessions:
                raise APIError(409, 'retired_session')
            if now - self._last_activity < self.session_lease_seconds:
                raise APIError(409, 'session_busy')
            if len(self._sessions) >= self.max_sessions:
                raise APIError(503, 'session_capacity_restart_required')
            # Sequence 1 can be lost in transit, so the first observed sequence
            # may be any positive integer; it is still never retried.
            self._active = session
            self._sessions[session] = (0, 0, 0)
        old_sequence, old_stamp, old_now = self._sessions[session]
        if sequence <= old_sequence or latest <= old_stamp or observation_now < old_now:
            raise APIError(409, 'replayed_or_nonmonotonic_observation')
        self._sessions[session] = (sequence, latest, observation_now)
        self._last_activity = now

    def _check_budget(self, started):
        if self.clock() - started > self.budget_seconds:
            raise APIError(504, 'processing_deadline_exceeded')

    def infer(self, request, *, started=None):
        started = self.clock() if started is None else started
        try:
            # Validation, decoded bytes and eventual identity/reservation all
            # belong to this detached snapshot, never a caller-owned mapping.
            # Copying/validation remain inside the original processing budget.
            request = copy.deepcopy(request)
            jpegs = _validate_request_and_decode_jpegs(request)
        except (ProtocolError, RecursionError) as exc:
            raise APIError(400, 'invalid_request') from exc
        if not self.ready:
            raise APIError(503, 'not_ready')
        if not self._gpu.acquire(blocking=False):
            raise APIError(503, 'busy')
        dispatched = False
        job = None
        try:
            self._check_budget(started)
            remaining = self.budget_seconds - (self.clock() - started)
            job = {'request': request, 'jpegs': jpegs, 'started': started,
                   'deadline': time.monotonic() + remaining,
                   'done': threading.Event(), 'cancelled': threading.Event(),
                   'result': None, 'error': None}
            with self._condition:
                if self._closing or not self.ready or not self._worker.is_alive():
                    raise APIError(503, 'not_ready')
                if self._slot is not None or self._active_job is not None:
                    raise APIError(503, 'busy')
                self._slot = self._active_job = job
                dispatched = True
                self._condition.notify()
            while not job['done'].is_set():
                self._check_job(job)
                job['done'].wait(min(.05, max(0., job['deadline'] - time.monotonic())))
            self._check_job(job)
            if job['error'] is not None:
                raise job['error']
            return job['result']
        finally:
            if dispatched:
                # After a deadline/close, discard eventual results. The worker
                # still owns the GPU lock until its backend actually returns.
                if not job['done'].is_set():
                    job['cancelled'].set()
            else:
                self._gpu.release()

    def _check_job(self, job):
        if self._closing or not self.ready:
            raise APIError(503, 'not_ready')
        if job['cancelled'].is_set() or time.monotonic() >= job['deadline']:
            raise APIError(504, 'processing_deadline_exceeded')
        self._check_budget(job['started'])

    def _execute(self, job):
        request, started = job['request'], job['started']
        try:
            self._check_job(job)
            self._reserve(request)
            if self.image_decoder is decode_images:
                frames = _decode_validated_jpegs(job['jpegs'])
            else:
                # Injected decoders retain their one-payload interface and
                # execute on the same sole inference worker as before.
                frames = self.image_decoder(request)
            self._check_job(job)
            actions, inference_ms = self.backend.infer(frames, request['sequence'])
            # A slow backend is a deadline failure, not malformed model data.
            # Check before validate_response's inference_ms contract bound.
            self._check_job(job)
            # No numpy is required by mocked backends.  The real backend emits
            # ndarray; convert before strict finite/shape validation below.
            raw = actions.tolist() if hasattr(actions, 'tolist') else actions
            if not isinstance(raw, list) or len(raw) != NUM_SAMPLES:
                raise ProtocolError('Invalid model sample count')
            for path in raw:
                if not isinstance(path, list) or len(path) != TRAJECTORY_STEPS:
                    raise ProtocolError('Invalid model trajectory count')
                for xy in path:
                    if not isinstance(xy, list) or len(xy) != 2:
                        raise ProtocolError('Invalid model trajectory coordinates')
                    if any(type(v) not in (int, float) or not math.isfinite(v) for v in xy):
                        raise ProtocolError('Invalid model trajectory value')
            trajectories = [[[float(value) * WAYPOINT_SCALE for value in xy] for xy in path] for path in raw]
            response = {
                'version': PROTOCOL_VERSION, 'session_id': request['session_id'],
                'sequence': request['sequence'],
                'observation_stamp_ns': request['frames'][-1]['stamp_ns'],
                'frame_id': 'base_link', 'state': 'running',
                'waypoint': trajectories[0][WAYPOINT_INDEX], 'trajectories': trajectories,
                'inference_ms': float(inference_ms), 'waypoint_scale': WAYPOINT_SCALE,
            }
            validate_response(response, session_id=request['session_id'], sequence=request['sequence'],
                              observation_stamp_ns=request['frames'][-1]['stamp_ns'])
            self._check_job(job)
            if self.on_result is not None:
                # Preview callbacks must be nonblocking and never control ROS.
                self.on_result(request, response, frames)
            self._check_job(job)
            if self.audit:
                self.audit.write('prediction', sequence=request['sequence'], inference_ms=response['inference_ms'],
                                 session_hash=hashlib.sha256(request['session_id'].encode()).hexdigest()[:16])
            self._check_job(job)
            return response
        except APIError:
            raise
        except ProtocolError as exc:
            raise APIError(422, 'invalid_image_or_model_result') from exc
        except Exception as exc:
            # Detailed exception strings can contain input data: never echo them.
            raise APIError(500, 'inference_failed') from exc


class CampusRequestHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'
    server_version = 'NoMaDCampus/1'
    sys_version = ''
    # StreamRequestHandler applies TCP_NODELAY to the accepted TLS socket.
    # Small JSON replies must not wait for an ACK of the preceding headers.
    disable_nagle_algorithm = True

    def log_message(self, _format, *args):
        pass  # Base handler logs attacker-controlled URLs/headers; disable it.

    def handle(self):
        # A total socket deadline bounds TLS/header/body slow clients; the GPU
        # itself cannot be cancelled and retains its lock until infer completes.
        timer = threading.Timer(2.0, self._expire_connection)
        timer.daemon = True
        timer.start()
        try:
            super().handle()
        finally:
            timer.cancel()

    def _expire_connection(self):
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def _send_json(self, status, payload):
        raw = encode_json(payload)
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Connection', 'close')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(raw)
        self.close_connection = True

    def do_GET(self):
        if self.path != '/healthz':
            self._send_json(404, {'error': 'not_found'})
            return
        self._send_json(200 if self.server.service.ready else 503,
                        {'version': PROTOCOL_VERSION, 'ready': self.server.service.ready})

    def do_POST(self):
        started = self.server.service.clock()
        # These phases start after TLS and HTTP headers have been received.
        # No body/header/path content is ever copied into the diagnostic.
        diagnostic_started = time.monotonic()
        diagnostic = dict(phase='request_target', status=500, code='request_failed',
                          declared_body_bytes=None, body_bytes=0, body_read_ms=None,
                          json_decode_ms=None, service_ms=None, elapsed_ms=None)
        try:
            self._handle_post(diagnostic, started)
        finally:
            diagnostic['elapsed_ms'] = (time.monotonic() - diagnostic_started) * 1000
            if self.server.service.audit:
                self.server.service.audit.write('request_stats', **diagnostic)

    def _handle_post(self, diagnostic, started):
        service = self.server.service
        status, code = 500, 'request_failed'
        try:
            if self.path != '/v1/infer':
                raise APIError(404, 'not_found')
            diagnostic['phase'] = 'authorization'
            auth = self.headers.get_all('Authorization', [])
            if len(auth) != 1 or not service.authenticate(auth[0]):
                raise APIError(401, 'unauthorized')
            diagnostic['phase'] = 'headers'
            if self.headers.get_all('Transfer-Encoding') or self.headers.get_all('Expect'):
                raise APIError(400, 'unsupported_transfer')
            content_types = self.headers.get_all('Content-Type', [])
            if len(content_types) != 1 or content_types[0].split(';', 1)[0].strip().lower() != 'application/json':
                raise APIError(415, 'expected_json')
            lengths = self.headers.get_all('Content-Length', [])
            if len(lengths) != 1 or not lengths[0].isdigit() or len(lengths[0]) > 10:
                raise APIError(411, 'content_length_required')
            length = int(lengths[0])
            diagnostic['declared_body_bytes'] = length
            if not 0 < length <= MAX_BODY_BYTES:
                raise APIError(413, 'body_too_large')
            raw = bytearray()
            diagnostic['phase'] = 'body'
            stage_started = time.monotonic()
            try:
                while len(raw) < length:
                    remaining = service.budget_seconds - (service.clock() - started)
                    if remaining <= 0:
                        raise APIError(408, 'request_deadline_exceeded')
                    self.connection.settimeout(remaining)
                    # read1 exposes already received chunks; read(N) can hide
                    # their count when it subsequently times out awaiting N.
                    block = self.rfile.read1(min(65536, length - len(raw)))
                    if not block:
                        raise APIError(400, 'incomplete_body')
                    raw.extend(block)
                    diagnostic['body_bytes'] = len(raw)
            finally:
                diagnostic['body_read_ms'] = (time.monotonic() - stage_started) * 1000
            diagnostic['phase'] = 'json'
            stage_started = time.monotonic()
            try:
                request = decode_json(bytes(raw))
            finally:
                diagnostic['json_decode_ms'] = (time.monotonic() - stage_started) * 1000
            diagnostic['phase'] = 'service'
            stage_started = time.monotonic()
            try:
                response = service.infer(request, started=started)
                service._check_budget(started)
            finally:
                diagnostic['service_ms'] = (time.monotonic() - stage_started) * 1000
            diagnostic.update(phase='response', status=200, code='ok')
            self._send_json(200, response)
            diagnostic['phase'] = 'complete'
            return
        except APIError as exc:
            status, code = exc.status, exc.code
        except ProtocolError:
            status, code = 400, 'invalid_json'
        except (TimeoutError, socket.timeout):
            status, code = 408, 'request_deadline_exceeded'
        except (BrokenPipeError, ConnectionError, OSError):
            diagnostic.update(status=499, code='connection_closed')
            return
        except Exception:
            status, code = 500, 'request_failed'
        diagnostic.update(status=status, code=code)
        if service.audit:
            service.audit.write('rejected', status=status, code=code)
        try:
            self._send_json(status, {'version': PROTOCOL_VERSION, 'state': 'unavailable', 'error': code})
        except (ConnectionError, OSError):
            pass


class BoundedHTTPSServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    block_on_close = False
    request_queue_size = 4

    def __init__(self, address, service, tls_context, *, max_handlers=4):
        if not isinstance(tls_context, ssl.SSLContext) or not 1 <= max_handlers <= 8:
            raise ValueError('A TLS server context and 1..8 handlers are required')
        if tls_context.protocol != ssl.PROTOCOL_TLS_SERVER:
            raise ValueError('Only a TLS server context is accepted')
        if tls_context.minimum_version < ssl.TLSVersion.TLSv1_2:
            raise ValueError('TLS 1.2 or later is required')
        self.service, self.tls_context = service, tls_context
        self._slots = threading.BoundedSemaphore(max_handlers)
        super().__init__(address, CampusRequestHandler)

    def process_request(self, request, client_address):
        if not self._slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self._slots.release()
            raise

    def process_request_thread(self, request, client_address):
        secured = request
        try:
            request.settimeout(1.0)
            secured = self.tls_context.wrap_socket(request, server_side=True)
            self.finish_request(secured, client_address)
        except (OSError, ssl.SSLError):
            pass
        finally:
            self.shutdown_request(secured)
            self._slots.release()

    def handle_error(self, request, client_address):
        pass  # No traceback with payloads/headers to an unmanaged log.

    def server_close(self):
        super().server_close()
        if not self.service.close():
            raise RuntimeError('Inference worker did not exit; GPU work was not cancelled or unlocked')


def create_server(address, service, *, cert=None, key=None, tls_context=None, max_handlers=4):
    if tls_context is None:
        if cert is None or key is None:
            raise ValueError('TLS certificate and key are mandatory')
        tls_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        tls_context.minimum_version = ssl.TLSVersion.TLSv1_2
        tls_context.load_cert_chain(str(cert), str(key))
    return BoundedHTTPSServer(address, service, tls_context, max_handlers=max_handlers)


def load_backend(project_root, device='cuda'):
    from .nomad_core import NoMaDBackend
    root = Path(project_root).resolve(strict=True)
    return NoMaDBackend({
        'model_config': str(root / 'visualnav-transformer/train/config/nomad.yaml'),
        'checkpoint': str(root / 'visualnav-transformer/deployment/model_weights/nomad.pth'),
        'action_stats_path': str(root / 'visualnav-transformer/train/vint_train/data/data_config.yaml'),
        'device': device, 'num_samples': NUM_SAMPLES, 'seed': 0, 'warmup': True,
    })


def main(argv=None, *, on_result=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8443)
    parser.add_argument('--cert', required=True)
    parser.add_argument('--key', required=True)
    parser.add_argument('--token-file', required=True)
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--device', choices=('cuda', 'cpu'), default='cuda',
                        help='CPU is explicit diagnostic mode; it retains the 0.6s deadline')
    args = parser.parse_args(argv)
    token_path = Path(args.token_file)
    if token_path.stat().st_size > 257:
        parser.error('Token file is too large')
    token = token_path.read_text(encoding='ascii').strip()
    # Verify TLS before allocating the GPU; no plain HTTP fallback exists.
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(args.cert, args.key)
    audit = AuditLog(args.output_root)
    server = service = None
    try:
        service = CampusInferenceService(None, token, audit=audit, on_result=on_result,
            backend_factory=lambda: load_backend(args.project_root, args.device))
        server = create_server((args.bind, args.port), service, tls_context=context)
        audit.write('ready', runtime=service.backend.runtime_info)
        print(json.dumps({'ready': True, 'version': PROTOCOL_VERSION, 'port': server.server_port,
                          'run_dir': str(audit.run_dir)}), flush=True)
        def stop(_signum, _frame):
            service.ready = False
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        server.serve_forever(poll_interval=0.2)
    finally:
        try:
            if server:
                server.server_close()
            elif service and not service.close():
                raise RuntimeError('Inference worker did not exit during startup cleanup')
        finally:
            audit.close()


if __name__ == '__main__':
    main()
