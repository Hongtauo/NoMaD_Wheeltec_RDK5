#!/usr/bin/env python3
"""Read-only ROS/system clock continuity check; no /clock publisher or time changes."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time

import rclpy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120.0)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 600:
        parser.error('--seconds must be between 1 and 600')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('Set a dedicated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    root = Path(__file__).resolve().parents[2]
    output = root / 'outputs/time_sync_checks' / (
        'system-clock__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    output.mkdir(parents=True, exist_ok=False)
    print(f'Clock check: {output}', flush=True)
    rclpy.init()
    node = rclpy.create_node('system_clock_check')
    rows = []
    try:
        assert not node.get_parameter('use_sim_time').value
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            rows.append((time.monotonic_ns(), time.time_ns(), node.get_clock().now().nanoseconds))
            time.sleep(0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    with (output / 'samples.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['monotonic_ns', 'system_ns', 'ros_ns'])
        writer.writerows(rows)
    deltas = [(b[0]-a[0], b[1]-a[1], b[2]-a[2]) for a, b in zip(rows, rows[1:])]
    hashes = {}
    for name in ('controller_core.py', 'controller_node.py'):
        path = root / 'ros2_ws/src/nomad_navigation/nomad_navigation' / name
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    backward = sum(system < 0 or ros < 0 for _, system, ros in deltas)
    report = {'status': 'complete' if backward == 0 else 'failed',
              'use_sim_time': False, 'samples': len(rows),
              'duration_seconds': (rows[-1][0]-rows[0][0])/1e9,
              'backward_steps': backward,
              'max_abs_ros_vs_monotonic_step_error_ms': max(abs(ros-mono) for mono, _, ros in deltas)/1e6,
              'max_abs_system_vs_monotonic_step_error_ms': max(abs(system-mono) for mono, system, _ in deltas)/1e6,
              'controller_sha256': hashes}
    (output / 'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
    return int(backward != 0)


if __name__ == '__main__':
    raise SystemExit(main())
