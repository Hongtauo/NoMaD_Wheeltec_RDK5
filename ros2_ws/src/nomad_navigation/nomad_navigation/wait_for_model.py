"""Bounded model-readiness barrier for launch; publishes no messages."""
import json
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wait_for_nomad')
    node.declare_parameter('timeout_sec', 90.0)
    node.declare_parameter('status_topic', '/nomad/status')
    result = 1
    state = {'ready': False, 'fault': False}
    def receive(msg):
        try:
            value = json.loads(msg.data)
            state['ready'] = value['state'] in ('waiting_for_images', 'running') and bool(value.get('run_dir'))
            state['fault'] = value['state'] == 'faulted'
        except (ValueError, KeyError, TypeError):
            pass
    node.create_subscription(String, node.get_parameter('status_topic').value, receive,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-started < node.get_parameter('timeout_sec').value:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state['ready']:
                node.get_logger().info('Model is ready; video may start')
                result = 0
                break
            if state['fault']:
                break
        if result:
            node.get_logger().error('Model readiness failed or timed out; video will not start')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
