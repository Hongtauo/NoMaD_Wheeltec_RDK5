#!/usr/bin/env python3
"""CPU-only upload codec benchmark; no ROS, camera, network, or model access.

Default input is deterministic 640x480 synthetic texture encoded as JPEG95.
Optional --jpeg paths are read-only. Prints one JSON report to stdout and does
not write artifacts; callers may capture it in their timestamped report folder.
"""

import argparse
import io
import json
import math
from pathlib import Path
import platform
import statistics
import threading
import time
import uuid

from nomad_navigation.campus_image import UploadImagePreparer, compact_jpeg


def distribution(values):
    values = sorted(values)
    return dict(count=len(values), mean=statistics.mean(values), min=values[0],
                p95=values[math.ceil(.95 * len(values)) - 1], max=values[-1])


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
        Image.fromarray(rgb).save(output, 'JPEG', quality=95)
        images.append(output.getvalue())
    return images


def make_request(images, index, session):
    return dict(version=1, session_id=session, sequence=index + 1,
                frame_id='camera_color_optical_frame',
                observation_now_ns=10_050_000_000 + (index + 3) * 250_000_000,
                frames=[dict(stamp_ns=10_000_000_000 + (index + n) * 250_000_000,
                             jpeg_bytes=images[(index + n) % len(images)])
                        for n in range(4)])


def pixel_error(original, uploaded):
    """Compare final 96x96 RGB pixels, not model actions or navigation safety."""
    import numpy as np
    from PIL import Image
    def decoded(raw):
        with Image.open(io.BytesIO(raw)) as image:
            return np.asarray(image.convert('RGB').resize((96, 96), Image.Resampling.BICUBIC),
                              dtype=np.float32)
    difference = decoded(original) - decoded(uploaded)
    mse = float(np.mean(difference ** 2))
    return dict(mae_rgb_0_255=float(np.mean(np.abs(difference))),
                psnr_db=10 * math.log10(255 ** 2 / mse) if mse else None)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=12, choices=range(3, 101))
    parser.add_argument('--quality', type=int, choices=(75, 85), default=85)
    parser.add_argument('--jpeg', type=Path, action='append', default=[])
    args = parser.parse_args(argv)
    backends = ('pillow', 'pillow_draft', 'opencv')
    preparers = {name: UploadImagePreparer(backend=name, quality=args.quality) for name in backends}
    warmups = {name: preparer.warmup() for name, preparer in preparers.items()}
    if args.jpeg:
        if len(args.jpeg) > 100:
            parser.error('At most 100 read-only JPEG inputs are allowed')
        # Per-file bytes are still bounded by compact_jpeg before pixel decode.
        from nomad_navigation.campus_protocol import MAX_JPEG_BYTES
        if any(path.stat().st_size > MAX_JPEG_BYTES for path in args.jpeg):
            parser.error('JPEG input exceeds protocol byte limit')
        images = [path.read_bytes() for path in args.jpeg]
    else:
        images = synthetic_jpegs(args.iterations + 3)
    session = str(uuid.uuid4())
    measurements = {}
    for backend in backends:
        durations, output_sizes, errors = [], [], []
        for index in range(args.iterations):
            raw = images[index % len(images)]
            started = time.perf_counter()
            encoded = compact_jpeg(raw, quality=args.quality, backend=backend)
            durations.append((time.perf_counter() - started) * 1000)
            output_sizes.append(len(encoded))
            errors.append(pixel_error(raw, encoded))
        requests, call_times = [], []
        preparer = preparers[backend]
        for index in range(args.iterations):
            request = make_request(images, index, session)
            failures = []
            def once():
                try:
                    preparer.prepare_request(request)
                except BaseException as error:
                    failures.append(error)
            started = time.perf_counter()
            thread = threading.Thread(target=once)
            thread.start()
            thread.join()
            if failures:
                raise failures[0]
            call_times.append((time.perf_counter() - started) * 1000)
            requests.append(preparer.last_metrics)
        source_bytes = sum(len(images[index % len(images)]) for index in range(args.iterations))
        finite_psnr = [row['psnr_db'] for row in errors if row['psnr_db'] is not None]
        measurements[backend] = dict(
            warmup=warmups[backend], single_image_ms=distribution(durations),
            source_bytes=source_bytes, encoded_bytes=sum(output_sizes),
            reduction_fraction=1 - sum(output_sizes) / source_bytes,
            final_96_mae=distribution([row['mae_rgb_0_255'] for row in errors]),
            final_96_psnr_db_min=min(finite_psnr) if finite_psnr else None,
            first_context=requests[0],
            sliding_context_ms=distribution([row['prepare_ms'] for row in requests[1:]]),
            fresh_request_thread_ms=distribution(call_times), context_samples=requests)
    import cv2
    from PIL import __version__ as pillow_version
    print(json.dumps(dict(platform=platform.platform(), machine=platform.machine(),
                          opencv=cv2.__version__, pillow=pillow_version,
                          input='read-only JPEG files' if args.jpeg else 'synthetic 640x480 texture, JPEG95',
                          iterations=args.iterations, quality=args.quality, max_edge=320,
                          no_ros_camera_network_or_model=True, measurements=measurements), indent=2))


if __name__ == '__main__':
    main()
