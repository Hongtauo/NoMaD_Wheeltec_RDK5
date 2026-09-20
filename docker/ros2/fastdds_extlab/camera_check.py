"""Image-only bounded compatibility check, with unique output per run."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

from camera_metrics import summarize_camera_metrics

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--reliability', choices=('best_effort', 'reliable'), default='best_effort')
    p.add_argument('--output-root', default='/opt/extlab/results')
    args = p.parse_args()
    run = Path(args.output_root) / ('camera__' + args.reliability + '__' +
           datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '__' + uuid.uuid4().hex[:8])
    run.mkdir(parents=True, exist_ok=False)
    rclpy.init(); n = rclpy.create_node('extlab_camera_check')
    frames = []; decode_errors = []; dimensions = set(); last_encoded = None
    started = time.monotonic()

    def receive(msg):
        nonlocal last_encoded
        now = time.monotonic()
        received_unix_s = time.time()
        header_unix_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1_000_000_000
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if img is None:
            decode_errors.append(now - started); return
        dimensions.add(tuple(img.shape))
        frames.append({'t': now - started, 'stamp_sec': msg.header.stamp.sec,
                       'stamp_nanosec': msg.header.stamp.nanosec, 'bytes': len(msg.data),
                       'received_unix_s': received_unix_s,
                       'header_age_s': received_unix_s - header_unix_s,
                       'frame_id': msg.header.frame_id, 'format': msg.format})
        if len(frames) == 1:
            if not cv2.imwrite(str(run / 'first_frame.png'), img):
                raise RuntimeError('Could not write first frame')
        last_encoded = bytes(msg.data)

    qos = QoSProfile(depth=5, durability=DurabilityPolicy.VOLATILE,
                     reliability=ReliabilityPolicy.RELIABLE if args.reliability == 'reliable'
                     else ReliabilityPolicy.BEST_EFFORT)
    sub = n.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', receive, qos)
    libraries = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                        if any(s in line for s in ('libfastrtps.so', 'librmw_fastrtps', 'libfastcdr.so'))})
    print(json.dumps({'output_dir': str(run), 'loaded_libraries': libraries}), flush=True)
    end = started + args.seconds; next_report = started + 10
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)
        if time.monotonic() >= next_report:
            print(json.dumps({'elapsed': round(time.monotonic()-started, 1), 'frames': len(frames),
                              'publishers': n.count_publishers('/camera/color/image_raw/compressed')}), flush=True)
            next_report += 10
    gaps = [b['t']-a['t'] for a,b in zip(frames, frames[1:])]
    result = {'seconds': time.monotonic()-started, 'reliability': args.reliability,
              'frames': len(frames), 'decode_errors': len(decode_errors), 'dimensions': sorted(dimensions),
              'first_frame_s': frames[0]['t'] if frames else None,
              'last_frame_s': frames[-1]['t'] if frames else None,
              'active_hz': (len(frames)-1)/(frames[-1]['t']-frames[0]['t']) if len(frames)>1 else None,
              'max_gap_s': max(gaps) if gaps else None,
              'camera_publishers': n.count_publishers('/camera/color/image_raw/compressed'),
              'loaded_libraries': libraries, 'output_dir': str(run)}
    result.update(summarize_camera_metrics(frames, result['seconds']))
    if last_encoded is not None:
        cv2.imwrite(str(run / 'last_frame.png'), cv2.imdecode(np.frombuffer(last_encoded, np.uint8), cv2.IMREAD_COLOR))
    (run / 'frames.json').write_text(json.dumps(frames, indent=2), encoding='utf-8')
    (run / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result), flush=True)
    n.destroy_node(); rclpy.shutdown()
    if not frames: raise SystemExit(1)


if __name__ == '__main__':
    main()
