"""Bounded camera upload preparation; run in the HTTPS worker, not ROS callbacks.

Keep the entire camera view and its orientation.  A 640x480 frame becomes
320x240, NOT a cropped square; the server still performs NoMaD's unchanged
96x96 BICUBIC preprocessing.  The extra resize/JPEG pass is lossy and is not
claimed to be numerically identical to feeding the original JPEG to NoMaD.

Importing this module needs only stdlib + campus_protocol. Image codecs load
only during warmup or actual preparation. Nothing publishes ROS, changes a
timestamp, extends a deadline, or retries a request.
"""

import base64
from collections import OrderedDict
import copy
from dataclasses import dataclass
import hashlib
import io
import time
import warnings

from .campus_protocol import (
    MAX_JPEG_BYTES, ProtocolError, _validate_request_and_decode_jpegs,
    _validate_raw_request_and_get_jpegs,
)

DEFAULT_MAX_EDGE = 320
DEFAULT_JPEG_QUALITY = 85
MAX_INPUT_EDGE = 1920
MAX_INPUT_PIXELS = 1920 * 1080
MIN_IMAGE_EDGE = 16
MAX_CACHE_ENTRIES = 8
DEFAULT_BACKEND = 'opencv'
BACKENDS = ('opencv', 'pillow_draft', 'pillow')


class ImagePreparationError(ProtocolError):
    """Input cannot be prepared without violating the existing image contract."""


def _settings(max_edge, quality, backend=DEFAULT_BACKEND):
    # Restrict configuration to reviewed upload modes, not arbitrary model or
    # field-of-view changes. Both modes leave the server's 96x96 input intact.
    if type(max_edge) is not int or max_edge != DEFAULT_MAX_EDGE:
        raise ValueError('Only the reviewed 320-pixel upload edge is supported')
    if type(quality) is not int or quality not in (75, 85):
        raise ValueError('JPEG upload quality must be 75 or 85')
    if backend not in BACKENDS:
        raise ValueError('Unknown JPEG preparation backend')


def _jpeg_envelope(jpeg):
    if (not isinstance(jpeg, bytes) or not 4 <= len(jpeg) <= MAX_JPEG_BYTES
            or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9')):
        raise ImagePreparationError('Invalid or oversized source JPEG')


def _opencv_encode(jpeg, source_size, target, quality):
    """Decode reduced JPEG DCT coefficients before allocating full RGB pixels."""
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise ImagePreparationError('OpenCV upload backend is unavailable') from exc
    width, height = source_size
    reduction = 1
    for candidate in (8, 4, 2):
        reduced = ((width + candidate - 1) // candidate, (height + candidate - 1) // candidate)
        if reduced[0] >= target[0] and reduced[1] >= target[1]:
            reduction = candidate
            break
    flags = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
             4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}[reduction]
    # Default OpenCV decoding may honor EXIF orientation; the existing NoMaD
    # Pillow decoder deliberately does not. Never silently rotate a camera.
    flags |= cv2.IMREAD_IGNORE_ORIENTATION
    try:
        bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), flags)
        expected = ((height + reduction - 1) // reduction,
                    (width + reduction - 1) // reduction, 3)
        if (bgr is None or bgr.dtype != np.uint8 or bgr.shape != expected
                or bgr.shape[0] * bgr.shape[1] > MAX_INPUT_PIXELS):
            raise ImagePreparationError('Unexpected decoded JPEG dimensions/type')
        if (bgr.shape[1], bgr.shape[0]) != target:
            # Only shrink; reduced decode was selected to remain >= target.
            # Area interpolation retains the whole image without a crop.
            bgr = cv2.resize(bgr, target, interpolation=cv2.INTER_AREA)
        if bgr.shape != (target[1], target[0], 3) or bgr.dtype != np.uint8:
            raise ImagePreparationError('Unexpected resized JPEG dimensions/type')
        ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality,
                                               cv2.IMWRITE_JPEG_OPTIMIZE, 0,
                                               cv2.IMWRITE_JPEG_PROGRESSIVE, 0])
        if not ok or encoded is None or encoded.dtype != np.uint8:
            raise ImagePreparationError('JPEG encoder failed')
        if encoded.size > MAX_JPEG_BYTES:
            raise ImagePreparationError('Encoded JPEG exceeds byte limit')
        return encoded.tobytes()
    except cv2.error as exc:
        raise ImagePreparationError('OpenCV JPEG decode/resize/encode failed') from exc


def compact_jpeg(jpeg, *, max_edge=DEFAULT_MAX_EDGE, quality=DEFAULT_JPEG_QUALITY,
                 backend=DEFAULT_BACKEND):
    """Return bounded RGB JPEG bytes with no crop/rotation/padding/upscaling.

    Dimensions and compressed bytes are checked before pixel allocation.
    Aspect ratio is preserved up to integer-pixel rounding. EXIF orientation
    is deliberately not applied, matching the server's existing decoder.
    No source metadata is passed into the re-encoded JPEG.
    """
    _settings(max_edge, quality, backend)
    _jpeg_envelope(jpeg)
    from PIL import Image

    try:
        with warnings.catch_warnings():
            warnings.simplefilter('error', Image.DecompressionBombWarning)
            with Image.open(io.BytesIO(jpeg)) as source:
                width, height = source.size
                if (source.format != 'JPEG' or min(width, height) < MIN_IMAGE_EDGE
                        or max(width, height) > MAX_INPUT_EDGE
                        or width * height > MAX_INPUT_PIXELS):
                    raise ImagePreparationError('Unsupported source JPEG dimensions/format')
                if max(width, height) > max_edge:
                    target = (max(1, round(width * max_edge / max(width, height))),
                              max(1, round(height * max_edge / max(width, height))))
                else:
                    target = source.size
                if min(target) < MIN_IMAGE_EDGE:
                    # Stretching or cropping to repair extreme aspect ratios
                    # would alter the camera geometry; fail instead.
                    raise ImagePreparationError('Aspect ratio cannot fit the upload image limits')
                if backend == 'opencv':
                    # The Pillow object is used only as a bounded header parser;
                    # do not call source.load() on the fast ARM upload path.
                    result = _opencv_encode(jpeg, source.size, target, quality)
                else:
                    if backend == 'pillow_draft':
                        source.draft('RGB', target)
                    source.load()
                    rgb = source.convert('RGB')
                    if rgb.size != target:
                        rgb = rgb.resize(target, Image.Resampling.BICUBIC)
                    output = io.BytesIO()
                    rgb.save(output, format='JPEG', quality=quality, subsampling=2,
                             optimize=False, progressive=False)
                    result = output.getvalue()
    except (OSError, ValueError, Image.DecompressionBombError, Image.DecompressionBombWarning) as exc:
        if isinstance(exc, ImagePreparationError):
            raise
        raise ImagePreparationError('JPEG decode/resize/encode failed') from exc
    _jpeg_envelope(result)
    return result


@dataclass(frozen=True)
class _CompactImage:
    """Owned codec output and its once-encoded wire representation."""
    jpeg: bytes
    jpeg_b64: str


class UploadImagePreparer:
    """Single-request-worker helper with a bounded cache and no pending queue.

    The cache stores at most eight compact JPEGs and their Base64 strings,
    keyed by source-byte SHA256, not timestamps. Reused context frames need
    neither pixel decode/re-encode nor another Base64 encode.
    Stamp/sequence/session identity is copied from each request, never cached.
    This object is intentionally not shared across concurrent workers.
    """

    def __init__(self, *, max_edge=DEFAULT_MAX_EDGE, quality=DEFAULT_JPEG_QUALITY,
                 max_cache_entries=MAX_CACHE_ENTRIES, backend=DEFAULT_BACKEND):
        _settings(max_edge, quality, backend)
        if type(max_cache_entries) is not int or not 0 <= max_cache_entries <= MAX_CACHE_ENTRIES:
            raise ValueError('Image cache entries must be an integer in [0, 8]')
        self.max_edge, self.quality = max_edge, quality
        self.backend = backend
        self.max_cache_entries = max_cache_entries
        self._cache = OrderedDict()
        self._last_metrics = None
        self._warmup_metrics = None

    @property
    def last_metrics(self):
        return dict(self._last_metrics) if self._last_metrics is not None else None

    def warmup(self):
        """Load codecs BEFORE ROS node creation/sampling; no observation exists.

        A local blank 640x480 JPEG exercises the real half-size decode path.
        This does not fill the real-frame cache or update request diagnostics.
        """
        if self._warmup_metrics is None:
            started = time.perf_counter()
            from PIL import Image
            stream = io.BytesIO()
            Image.new('RGB', (640, 480)).save(stream, format='JPEG', quality=85)
            compact_jpeg(stream.getvalue(), max_edge=self.max_edge,
                         quality=self.quality, backend=self.backend)
            self._warmup_metrics = dict(backend=self.backend,
                warmup_ms=(time.perf_counter() - started) * 1000, complete=True)
        return dict(self._warmup_metrics)

    def prepare_request(self, request):
        """Copy a valid request and replace JPEGs only; never renew its age.

        Caller must keep the ORIGINAL request-start monotonic timestamp when
        invoking HTTPSInference.infer(..., started=started). Preparation time
        is part of the unchanged 0.8s end-to-end budget, not an extra allowance.
        """
        started = time.perf_counter()
        metrics = dict(prepare_ms=0., source_jpeg_bytes=0, encoded_jpeg_bytes=0,
                       cache_hits=0, cache_misses=0, cache_entries=len(self._cache),
                       complete=False)
        try:
            frames = request.get('frames') if isinstance(request, dict) else None
            raw_input = (isinstance(frames, list) and bool(frames)
                         and isinstance(frames[0], dict) and 'jpeg_bytes' in frames[0])
            # Production uses bytes all the way from the camera history. The
            # legacy wire-dict adapter remains strict for standalone callers.
            decoded = (_validate_raw_request_and_get_jpegs(request) if raw_input
                       else _validate_request_and_decode_jpegs(request))
            prepared = copy.deepcopy(request)
            for jpeg, target in zip(decoded, prepared['frames']):
                metrics['source_jpeg_bytes'] += len(jpeg)
                key = (self.backend, self.max_edge, self.quality, hashlib.sha256(jpeg).digest())
                cached = self._cache.get(key)
                if cached is not None:
                    if not isinstance(cached, _CompactImage):
                        raise ImagePreparationError('Invalid owned image cache entry')
                    self._cache.move_to_end(key)
                    metrics['cache_hits'] += 1
                else:
                    metrics['cache_misses'] += 1
                    encoded = compact_jpeg(jpeg, max_edge=self.max_edge,
                                           quality=self.quality, backend=self.backend)
                    cached = _CompactImage(encoded, base64.b64encode(encoded).decode('ascii'))
                    if self.max_cache_entries:
                        self._cache[key] = cached
                        while len(self._cache) > self.max_cache_entries:
                            self._cache.popitem(last=False)
                metrics['encoded_jpeg_bytes'] += len(cached.jpeg)
                if raw_input:
                    del target['jpeg_bytes']
                target['jpeg_b64'] = cached.jpeg_b64
            # We just constructed these outputs with the bounded codec. Do
            # not decode them again here; HTTPS validates the wire boundary.
            metrics['complete'] = True
            return prepared
        finally:
            metrics['prepare_ms'] = (time.perf_counter() - started) * 1000
            metrics['cache_entries'] = len(self._cache)
            self._last_metrics = metrics
