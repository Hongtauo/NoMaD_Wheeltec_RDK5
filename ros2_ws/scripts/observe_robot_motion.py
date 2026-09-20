#!/usr/bin/env python3
"""Bounded read-only robot-local status/odometry audit; no services/publishers."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, choices=range(10, 61), default=30)
    args = parser.parse_args()
    from nomad_navigation.robot_identity import require_robot_identity, validate_api_environment
    require_robot_identity()
    validate_api_environment(os.environ, platform.machine())
    import rclpy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    output = Path('/home/wheeltec/nomad_live_control/runs') / (
        'motion_observation__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = dict(read_only=True, automatic_arm=False, seconds=args.seconds, rows=[], errors=[])
    rclpy.init(args=[])
    node = rclpy.create_node('nomad_readonly_motion_audit')
    last = {}

    def save(topic, fields):
        steady = time.monotonic()
        if steady-last.get(topic, 0.) < .04:
            return
        last[topic] = steady
        if len(report['rows']) >= 6000:
            raise RuntimeError('Observation row limit exceeded')
        report['rows'].append(dict(topic=topic, monotonic_sec=steady, time_ns=time.time_ns(), **fields))

    def status(topic, msg):
        try:
            if len(msg.data) > 32768:
                raise ValueError('Oversized status')
            value = json.loads(msg.data)
            names = ('armed', 'motion_paused', 'network_pause_resume', 'reason', 'model_state',
                     'resume_candidates', 'linear_x', 'angular_z', 'state', 'v', 'w', 'fault',
                     'source_waypoint_stamp_ns')
            save(topic, {name: value[name] for name in names if name in value})
        except (ValueError, TypeError):
            if len(report['errors']) < 32:
                report['errors'].append('invalid_' + topic)

    node.create_subscription(String, '/nomad/robot_control/status', lambda m: status('control', m),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/nomad/robot_gateway/status', lambda m: status('gateway', m), 10)
    node.create_subscription(Odometry, '/odom', lambda m: save('odom', dict(
        stamp_ns=m.header.stamp.sec * 10**9 + m.header.stamp.nanosec,
        linear_x=m.twist.twist.linear.x, angular_z=m.twist.twist.angular.z)),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    print(json.dumps(dict(output=str(output), read_only=True)), flush=True)
    try:
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        report['status'] = 'completed'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps(dict(report=str(output/'report.json'), rows=len(report['rows']))), flush=True)


if __name__ == '__main__':
    main()
