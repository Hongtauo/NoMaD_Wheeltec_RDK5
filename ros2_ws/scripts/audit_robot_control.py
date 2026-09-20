#!/usr/bin/env python3
"""Read-only, bounded onboard command/odometry audit; never arms or publishes."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=30, choices=range(5, 601))
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    output = args.output_root / ('audit__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('robot_control_audit', namespace='/nomad')
    rows = []
    def receive(kind, data):
        rows.append({'kind': kind, 'monotonic': time.monotonic(), 'ros_ns': node.get_clock().now().nanoseconds, **data})
    subs = [
        node.create_subscription(Twist, '/cmd_vel', lambda m: receive('cmd_vel', {'v': m.linear.x, 'w': m.angular.z}), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', lambda m: receive('odom', {'stamp': m.header.stamp.sec*10**9+m.header.stamp.nanosec,
            'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z, 'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y}), qos_profile_sensor_data),
        node.create_subscription(String, '/nomad/robot_control/status', lambda m: receive('control', {'data': m.data}), 10),
        node.create_subscription(String, '/nomad/robot_gateway/status', lambda m: receive('gateway', {'data': m.data}), 10),
    ]
    print(str(output), flush=True)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        with (output / 'events.jsonl').open('x') as f:
            for row in rows:
                f.write(json.dumps(row) + '\n')
        cmds = [row for row in rows if row['kind'] == 'cmd_vel']
        odom = [row for row in rows if row['kind'] == 'odom']
        summary = {'duration_requested': args.seconds, 'counts': {k: sum(r['kind'] == k for r in rows) for k in ('cmd_vel', 'odom', 'control', 'gateway')},
                   'nonzero_cmd_count': sum(bool(r['v'] or r['w']) for r in cmds),
                   'max_command_v': max((abs(r['v']) for r in cmds), default=None),
                   'max_command_w': max((abs(r['w']) for r in cmds), default=None),
                   'max_measured_v': max((abs(r['v']) for r in odom), default=None),
                   'max_measured_w': max((abs(r['w']) for r in odom), default=None),
                   'last_command': cmds[-1] if cmds else None, 'last_odom': odom[-1] if odom else None}
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
