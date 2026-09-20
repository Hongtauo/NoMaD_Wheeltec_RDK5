"""Minimal log-only node for validating package discovery and ROS 2 lifecycle."""

import math
import signal

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def validate_heartbeat_period(value):
    """Reject nonpositive/nonfinite periods before creating a ROS timer."""
    # 转为浮点数
    period = float(value)
    # 拒绝无穷大和负数
    if not math.isfinite(period) or period <= 0:
        raise ValueError('heartbeat_period_sec must be finite and greater than zero')
    return period


class SmokeNode(Node):
    """
        Emit readiness and heartbeat logs, without model loading or motion topics.
        “心跳”就是周期性日志，用来确认节点仍在运行，没有发布心跳话题消息
    """

    def __init__(self, **kwargs):
        super().__init__('nomad_smoke', **kwargs)
        try:
            # declare_parameter为这个节点“nomad_smoke”声明一个叫做“heartbeat_period_sec”的参数，默认值是2.0
            self.declare_parameter(
                'heartbeat_period_sec', 2.0,
                ParameterDescriptor(
                    description='Positive heartbeat period in seconds; set at startup.',
                    read_only=True, # True表示参数声明后不能在运行中修改，但可以启动时覆盖
                ),
            )
            # declare_parameter声明一个叫做“greeting”的参数，默认值是“nomad_navigation is ready”
            self.declare_parameter(
                'greeting', 'nomad_navigation is ready',
                ParameterDescriptor(description='Startup message.', read_only=True),
            )
            # get_parameter()读取参数，在这表示读取名为“heartbeat_period_sec”的值，也就是2.0，赋值给period
            period = validate_heartbeat_period(
                self.get_parameter('heartbeat_period_sec').value)

            self._heartbeat_count = 0
            # 创建一个计时器_timer，为定时器注册一个回调函数_on_heartbeat，_timer的第一个参数控制定时器执行的时间间隔，第二个参数为定时执行的函数
            self._timer = self.create_timer(period, self._on_heartbeat)
            self.get_logger().info(
                f'{self.get_parameter("greeting").value}; '
                f'node={self.get_fully_qualified_name()}; '
                'log-only: no model loaded, no motion commands')
        except Exception:
            self.destroy_node()
            raise

    # 回调函数，当被执行的时候，_heartbeat_count自增1，并打印日志，将_heartbeat_count打印出来
    def _on_heartbeat(self):
        self._heartbeat_count += 1
        self.get_logger().info(f'heartbeat={self._heartbeat_count}')


def main(args=None):
    """Run until interrupted and release both the node and ROS context."""
    rclpy.init(args=args)
    node = None
    try:
        node = SmokeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT can reach both launch and its child; launch may
        # forward it again. Do not let a second SIGINT interrupt cleanup.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_handler)


if __name__ == '__main__':
    main()
