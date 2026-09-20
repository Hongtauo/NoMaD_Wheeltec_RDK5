"""Bounded, credential-free, latest-only file channel for independent RViz.

Server callbacks only swap a memory slot; a background thread serializes files.
The viewer opens only the public preview file, never the API token or TLS key.
"""
import base64
import json
import math
import os
from pathlib import Path
import stat
import threading
import time
import uuid
from contextlib import contextmanager


MAX_SNAPSHOT_BYTES = 2 * 1024 * 1024
MAX_JPEG_BYTES = 768 * 1024
MAX_AGE_SEC = 1.0
PREDICTION_LABEL = 'SERVER PREDICTIONS ONLY'
PREVIEW_PATH = Path(__file__).resolve().parents[2] / 'outputs/campus_api/preview/latest.json'


def validate_snapshot(data):
    if not isinstance(data, dict) or data.get('schema') != 1 or type(data.get('schema')) is not int:
        raise ValueError('Invalid preview schema')
    base = {'schema', 'writer_id', 'published_ns', 'kind'}
    if not isinstance(data.get('writer_id'), str):
        raise ValueError('Invalid preview writer identity')
    uuid.UUID(data['writer_id'])
    if type(data.get('published_ns')) is not int or data['published_ns'] <= 0:
        raise ValueError('Invalid preview publication timestamp')
    if data.get('kind') in ('waiting', 'stopped'):
        if set(data) != base:
            raise ValueError('Unexpected preview state fields')
        return data
    if data.get('kind') != 'prediction' or set(data) != base | {'frame', 'prediction'}:
        raise ValueError('Invalid preview kind/fields')
    frame, prediction = data['frame'], data['prediction']
    if not isinstance(frame, dict) or set(frame) != {'stamp_ns', 'jpeg_b64'}:
        raise ValueError('Invalid preview frame')
    if type(frame['stamp_ns']) is not int or frame['stamp_ns'] <= 0:
        raise ValueError('Invalid preview source timestamp')
    encoded = frame['jpeg_b64']
    if not isinstance(encoded, str) or not 4 <= len(encoded) <= 4 * ((MAX_JPEG_BYTES + 2) // 3):
        raise ValueError('Preview JPEG size is invalid')
    raw = base64.b64decode(encoded, validate=True)
    if len(raw) > MAX_JPEG_BYTES or not raw.startswith(b'\xff\xd8') or not raw.endswith(b'\xff\xd9'):
        raise ValueError('Invalid preview JPEG envelope')
    if not isinstance(prediction, dict) or set(prediction) != {'sequence', 'inference_ms', 'waypoint', 'trajectories'}:
        raise ValueError('Invalid preview prediction fields')
    if type(prediction['sequence']) is not int or prediction['sequence'] < 1:
        raise ValueError('Invalid preview sequence')
    if (type(prediction['inference_ms']) not in (int, float)
            or not math.isfinite(prediction['inference_ms']) or not 0 <= prediction['inference_ms'] <= 600):
        raise ValueError('Invalid preview inference time')
    paths = prediction['trajectories']
    if not isinstance(paths, list) or len(paths) != 8:
        raise ValueError('Invalid preview trajectory count')
    for path in paths:
        if not isinstance(path, list) or len(path) != 8:
            raise ValueError('Invalid preview trajectory length')
        for point in path:
            _point(point)
    _point(prediction['waypoint'])
    return data


def _point(value):
    if (not isinstance(value, list) or len(value) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError('Invalid preview coordinate')


class SnapshotWriter:
    def __init__(self, path):
        if os.name != 'posix':
            raise RuntimeError('Preview writer requires Linux directory locks and file permissions')
        self.path = Path(path).absolute()
        if self.path != PREVIEW_PATH.absolute() or self.path.resolve() != self.path:
            raise ValueError('Preview writes are restricted to the dedicated preview/latest.json path')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.parent.resolve() != self.path.parent:
            raise ValueError('Preview directory must not be a symlink')
        self._directory_fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        self.writer_id = str(uuid.uuid4())
        # The directory lock serializes all writers. A fixed temporary name
        # bounds even repeated crash/restart residue to latest + one temp.
        self._temporary = self.path.with_name('.' + self.path.name + '.tmp')
        self._condition = threading.Condition()
        self._closing = False
        self.error = None
        self._pending = self._state('waiting')
        self._thread = threading.Thread(target=self._run, name='campus-preview-file', daemon=True)
        self._thread.start()

    def _state(self, kind):
        return dict(schema=1, writer_id=self.writer_id, published_ns=time.time_ns(), kind=kind)

    def submit(self, request, response, _decoded_frames):
        # Only explicitly public display fields are copied. In particular no
        # authorization, server credentials, telemetry or request dictionary.
        try:
            frame = request['frames'][-1]
            item = self._state('prediction')
            item['frame'] = {'stamp_ns': frame['stamp_ns'], 'jpeg_b64': frame['jpeg_b64']}
            item['prediction'] = {
                'sequence': response['sequence'], 'inference_ms': response['inference_ms'],
                'waypoint': list(response['waypoint']),
                'trajectories': [[list(point) for point in path] for path in response['trajectories']]}
            with self._condition:
                if not self._closing:
                    self._pending = item
                    self._condition.notify()
        except Exception as error:
            self.error = type(error).__name__  # No arbitrary values/credentials in diagnostics.

    def _write(self, item):
        validate_snapshot(item)
        payload = json.dumps(item, separators=(',', ':'), allow_nan=False).encode('utf-8')
        if len(payload) > MAX_SNAPSHOT_BYTES:
            raise ValueError('Preview snapshot exceeds its size bound')
        with self._directory_lock():
            # Recover only this dedicated, ordinary temporary entry after an
            # interrupted writer. Do not truncate a symlink/hardlink target.
            try:
                old = self._temporary.lstat()
            except FileNotFoundError:
                old = None
            if old is not None:
                if (not stat.S_ISREG(old.st_mode) or old.st_uid != os.getuid()
                        or old.st_nlink != 1 or old.st_size > MAX_SNAPSHOT_BYTES):
                    raise ValueError('Unsafe stale preview temporary file')
                self._temporary.unlink()
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_NONBLOCK
            descriptor = os.open(self._temporary, flags, 0o600)
            try:
                with os.fdopen(descriptor, 'wb') as stream:
                    if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                        raise ValueError('Preview temporary file must be regular')
                    os.fchmod(stream.fileno(), 0o600)
                    if stat.S_IMODE(os.fstat(stream.fileno()).st_mode) != 0o600:
                        raise ValueError('Preview filesystem cannot enforce mode 0600')
                    stream.write(payload)
                os.replace(self._temporary, self.path)
            finally:
                # Still holding the directory lock: another writer cannot yet
                # own this temporary name, including after a failed write.
                self._temporary.unlink(missing_ok=True)

    @contextmanager
    def _directory_lock(self):
        # Lock the directory inode: no extra lock/image-history file. Writers
        # and cleanup cooperate so check-owner + unlink cannot delete a newly
        # published snapshot belonging to the next service session.
        import fcntl
        fcntl.flock(self._directory_fd, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(self._directory_fd, fcntl.LOCK_UN)

    def _remove_own_files(self):
        with self._directory_lock():
            try:
                descriptor = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
                with os.fdopen(descriptor, 'rb') as stream:
                    metadata = os.fstat(stream.fileno())
                    if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > MAX_SNAPSHOT_BYTES:
                        raise ValueError('Unsafe preview cleanup target')
                    data = json.loads(stream.read(MAX_SNAPSHOT_BYTES + 1))
                if isinstance(data, dict) and data.get('writer_id') == self.writer_id:
                    self.path.unlink()
            except FileNotFoundError:
                pass

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._pending is not None or self._closing)
                    if self._closing:
                        return
                    item, self._pending = self._pending, None
                if item is not None:
                    try:
                        self._write(item)
                        self.error = None
                    except Exception as error:
                        if self.error != type(error).__name__:
                            print('Campus preview file unavailable (' + type(error).__name__
                                  + '); inference remains independent.', flush=True)
                        self.error = type(error).__name__
        finally:
            try:
                self._remove_own_files()
            except Exception as error:
                self.error = type(error).__name__
            finally:
                os.close(self._directory_fd)

    def close(self):
        with self._condition:
            if not self._closing:
                self._pending = None
                self._closing = True
                self._condition.notify()
        self._thread.join(timeout=1.)  # Viewer/disk trouble never blocks service shutdown indefinitely.
        return not self._thread.is_alive()


class SnapshotReader:
    def __init__(self, path):
        self.path = Path(path)
        self._signature = None
        self._cached = None

    def latest(self, *, now_ns=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0)
        try:
            descriptor = os.open(self.path, flags)
        except FileNotFoundError:
            self._signature = self._cached = None
            return None
        with os.fdopen(descriptor, 'rb') as stream:
            metadata = os.fstat(stream.fileno())
            if not stat.S_ISREG(metadata.st_mode) or not 0 < metadata.st_size <= MAX_SNAPSHOT_BYTES:
                raise ValueError('Preview file type/size is invalid')
            signature = (metadata.st_ino, metadata.st_mtime_ns, metadata.st_size)
            if signature != self._signature:
                payload = stream.read(MAX_SNAPSHOT_BYTES + 1)
                if len(payload) > MAX_SNAPSHOT_BYTES:
                    raise ValueError('Preview file exceeds its size bound')
                self._cached = validate_snapshot(json.loads(payload))
                self._signature = signature
        if not 0 <= now_ns - self._cached['published_ns'] <= int(MAX_AGE_SEC * 1e9):
            return None  # Restarted viewer never presents an old saved frame as live.
        return self._cached
