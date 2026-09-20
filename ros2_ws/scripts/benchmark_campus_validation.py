#!/usr/bin/env python3
"""CPU-only upload validation/caching benchmark, with no network/ROS/model.

Uses synthetic 640x480 JPEGs unless --jpeg inputs are supplied (read-only).
Reports actual production helper timings and independent stage timings; these
are not ARM estimates unless this script is run on ARM. No production code is
patched, no deadlines are changed, and no credentials/images are printed.
"""

import argparse
import base64
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import statistics
import time

from nomad_navigation.campus_image import UploadImagePreparer, compact_jpeg
from nomad_navigation.campus_protocol import (
    MAX_JPEG_BYTES, decode_jpeg_bytes, encode_json, validate_raw_request, validate_request,
)


def distribution(values):
    ordered = sorted(values)
    return dict(count=len(values), mean=statistics.mean(values), min=ordered[0],
                p95=ordered[math.ceil(.95 * len(values)) - 1], max=ordered[-1])


def synthetic_jpegs(count):
    import numpy as np
    from PIL import Image
    y, x = np.indices((480, 640), dtype=np.int32)
    random = np.random.default_rng(20260919)
    images = []
    for index in range(count):
        checker = (((x + index * 3) // 23 + y // 19) % 2) * 60
        base = np.stack(((x * 2 + y + checker + index * 5) % 256,
                         (y * 3 + checker + index * 7) % 256,
                         (x + y * 2 + checker + index * 11) % 256), axis=-1)
        noise = random.integers(-12, 13, size=base.shape, dtype=np.int16)
        rgb = np.clip(base + noise, 0, 255).astype(np.uint8)
        output = io.BytesIO()
        Image.fromarray(rgb).save(output, 'JPEG', quality=90)
        images.append(output.getvalue())
    return images


def request_for(images, index=0, *, raw=True):
    return dict(version=1, session_id='79acf50b-1c66-4b32-a445-f27e655d53a6',
                sequence=index + 1, frame_id='camera_color_optical_frame',
                observation_now_ns=10_800_000_000 + index * 250_000_000,
                frames=[dict(stamp_ns=10_000_000_000 + (index + n) * 250_000_000,
                             **({'jpeg_bytes': jpeg} if raw else
                                {'jpeg_b64': base64.b64encode(jpeg).decode('ascii')}))
                        for n, jpeg in enumerate(images)])


def measure(call, iterations):
    call()  # exclude one local scheduling/cache cold call
    values = []
    for _ in range(iterations):
        started = time.perf_counter()
        call()
        values.append((time.perf_counter() - started) * 1000)
    return distribution(values)


def context_case(images, iterations, backend, mode):
    preparer = UploadImagePreparer(backend=backend)
    preparer.warmup()
    preparer.prepare_request(request_for(images[:4]))
    samples = []
    for index in range(iterations):
        if mode == 'all_cached':
            selected = images[:4]
        elif mode == 'three_cached_one_new':
            selected = images[index + 1:index + 5]
        else:
            # Three disjoint groups rotate through the production 8-item LRU.
            start = 4 * ((index + 1) % 3)
            selected = images[start:start + 4]
        source = request_for(selected, index + 1)
        result = preparer.prepare_request(source)
        metrics = preparer.last_metrics
        assert metrics['complete'] is True
        assert [row['stamp_ns'] for row in result['frames']] == [row['stamp_ns'] for row in source['frames']]
        samples.append(metrics)
    return dict(prepare_ms=distribution([row['prepare_ms'] for row in samples]),
                cache_hits=sum(row['cache_hits'] for row in samples),
                cache_misses=sum(row['cache_misses'] for row in samples),
                source_jpeg_bytes=distribution([row['source_jpeg_bytes'] for row in samples]),
                encoded_jpeg_bytes=distribution([row['encoded_jpeg_bytes'] for row in samples]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, choices=range(3, 51), default=12)
    parser.add_argument('--backend', choices=('opencv', 'pillow_draft', 'pillow'), default='opencv')
    parser.add_argument('--jpeg', type=Path, action='append', default=[])
    args = parser.parse_args(argv)
    if args.jpeg and (len(args.jpeg) != 4 or any(path.stat().st_size > MAX_JPEG_BYTES for path in args.jpeg)):
        parser.error('Supply exactly four bounded read-only JPEG files, or no --jpeg arguments')
    synthetic = synthetic_jpegs(max(12, args.iterations + 4))
    originals = [path.read_bytes() for path in args.jpeg] if args.jpeg else synthetic[:4]
    preparer = UploadImagePreparer(backend=args.backend)
    warmup = preparer.warmup()
    source = request_for(originals)
    legacy_source = request_for(originals, raw=False)
    compact = preparer.prepare_request(source)
    encoded = [decode_jpeg_bytes(frame['jpeg_b64']) for frame in compact['frames']]
    original_b64 = [frame['jpeg_b64'] for frame in legacy_source['frames']]

    calls = {
        'raw_validate_four_ms': lambda: validate_raw_request(source),
        'original_validate_four_ms': lambda: validate_request(legacy_source),
        'original_explicit_decode_four_ms': lambda: [decode_jpeg_bytes(value) for value in original_b64],
        'original_sha256_four_ms': lambda: [hashlib.sha256(value).digest() for value in originals],
        'original_b64encode_four_ms': lambda: [base64.b64encode(value).decode('ascii') for value in originals],
        'metadata_deepcopy_ms': lambda: copy.deepcopy(source),
        'compact_validate_four_ms': lambda: validate_request(compact),
        'compact_b64encode_four_ms': lambda: [base64.b64encode(value).decode('ascii') for value in encoded],
        'compact_json_encode_ms': lambda: encode_json(compact),
        'original_codec_one_ms': lambda: compact_jpeg(originals[0], backend=args.backend),
        'original_codec_four_ms': lambda: [compact_jpeg(value, backend=args.backend) for value in originals],
    }
    timings = {name: measure(call, args.iterations) for name, call in calls.items()}
    # Production internal requests now carry bytes without a Base64 round trip.
    # The isolated legacy decode timing measures removed, not remaining work.
    # A/B preparation measurements are still needed to demonstrate the saving.
    report = dict(
        platform=platform.platform(), machine=platform.machine(), python=platform.python_version(),
        backend=args.backend, iterations=args.iterations, warmup=warmup,
        no_ros_camera_network_or_model=True,
        stage_input='four read-only JPEG files' if args.jpeg else 'synthetic 640x480 texture JPEG90',
        source_jpeg_bytes=sum(map(len, originals)), encoded_jpeg_bytes=sum(map(len, encoded)),
        request_body_bytes=len(encode_json(compact)), stage_ms=timings,
        current_calls_per_request=dict(source_base64_decode_passes=0,
                                       source_base64_encode_passes=0,
                                       compact_base64_decode_passes=1,
                                       server_base64_decode_passes=1,
                                       compact_full_json_encode_passes=2),
        internal_raw_jpeg_enabled=True,
        removed_stage_isolation_mean_ms=timings['original_explicit_decode_four_ms']['mean'],
        remaining_duplicate_json_mean_ms_estimate=timings['compact_json_encode_ms']['mean'],
        estimate_scope='Isolated stage timings only, not end-to-end savings; all request boundary validations retained',
        synthetic_cache_contexts={mode: context_case(synthetic, args.iterations, args.backend, mode)
                                  for mode in ('all_cached', 'three_cached_one_new', 'all_new')},
        caveat='Independent stage timings may overlap production work; cache cases are synthetic. Do not add percentiles or infer ARM performance from x86.')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
