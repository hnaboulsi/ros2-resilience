"""A ROS 2 node wrapping :class:`WatchdogEvaluator` with recovery publication."""

from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from ros2_resilience.health.watchdog import WatchdogEvaluator

__all__ = ["WatchdogNode", "main"]


class WatchdogNode(Node):
    """Detect stale/under-arriving traffic and latch a stop command."""

    def __init__(self, **kwargs) -> None:
        super().__init__("watchdog", **kwargs)
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/watchdog/status")
        self.declare_parameter("recovery_topic", "/watchdog/recovery_published")
        self.declare_parameter("max_age_sec", 0.3)
        self.declare_parameter("expected_rate_hz", 20.0)
        self.declare_parameter("loss_window_sec", 2.0)
        self.declare_parameter("max_loss_fraction", 0.8)
        self.declare_parameter("check_period_sec", 0.01)

        self._evaluator = WatchdogEvaluator(
            max_age_sec=self.get_parameter("max_age_sec").value,
            expected_rate_hz=self.get_parameter("expected_rate_hz").value,
            loss_window_sec=self.get_parameter("loss_window_sec").value,
            max_loss_fraction=self.get_parameter("max_loss_fraction").value,
        )

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._command_publisher = self.create_publisher(
            Twist, self.get_parameter("command_topic").value, 10
        )
        self._status_publisher = self.create_publisher(
            DiagnosticStatus, self.get_parameter("status_topic").value, 10
        )
        self._recovery_publisher = self.create_publisher(
            DiagnosticStatus, self.get_parameter("recovery_topic").value, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("input_topic").value, self._on_input, qos
        )
        self.create_timer(self.get_parameter("check_period_sec").value, self._on_check)

    def _on_input(self, msg: Odometry) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._evaluator.record_arrival(self.get_clock().now().nanoseconds, stamp_ns=stamp_ns)

    def _on_check(self) -> None:
        status = self._evaluator.check(self.get_clock().now().nanoseconds)
        if not status.violated:
            return

        diagnostic = DiagnosticStatus()
        diagnostic.name = "watchdog"
        diagnostic.level = DiagnosticStatus.ERROR
        diagnostic.message = status.reason.value if status.reason is not None else ""
        diagnostic.values = [
            KeyValue(key="age_sec", value=str(status.age_sec)),
            KeyValue(key="loss_fraction", value=str(status.loss_fraction)),
        ]
        self._status_publisher.publish(diagnostic)

        stop = Twist()
        self._command_publisher.publish(stop)
        self._recovery_publisher.publish(diagnostic)


def main(args: list[str] | None = None) -> None:
    """Run a standalone WatchdogNode."""
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
