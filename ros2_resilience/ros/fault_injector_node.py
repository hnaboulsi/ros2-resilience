"""A ROS 2 node wrapping :class:`FaultInjector` around one odometry stream."""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Int64

from ros2_resilience.faults.injector import FaultConfig, FaultInjector

__all__ = ["FaultInjectorNode", "main"]

_POLL_PERIOD_SEC = 0.005


class FaultInjectorNode(Node):
    """Forward one odometry stream through a single, reconfigurable fault.

    Reconfiguration happens via :meth:`apply_config`, called directly by an
    in-process orchestrator (the experiment runner) rather than through a
    remote ROS parameter/service call: every node in this Phase-1 harness
    runs in one process, so a direct, typed method call is the real
    equivalent of what a parameter service would do across processes,
    without the complexity of an async client for a purely local call.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("fault_injector", **kwargs)
        self.declare_parameter("input_topic", "/odom_raw")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("config_applied_topic", "/fault_injector/config_applied")
        self.declare_parameter("counters_topic", "/fault_injector/counters")

        self._injector = FaultInjector[Odometry](FaultConfig())
        self._revision = 0
        self._last_ns = time.monotonic_ns()

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._output_publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, qos
        )
        self._applied_publisher = self.create_publisher(
            Int64, self.get_parameter("config_applied_topic").value, 10
        )
        self._counters_publisher = self.create_publisher(
            DiagnosticStatus, self.get_parameter("counters_topic").value, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("input_topic").value, self._on_input, qos
        )
        self.create_timer(_POLL_PERIOD_SEC, self._on_poll)

    @property
    def applied_revision(self) -> int:
        """Return the most recently applied configuration revision."""
        return self._revision

    def apply_config(self, config: FaultConfig, *, revision: int) -> None:
        """Atomically apply ``config`` and publish the new revision."""
        now = self._now_ns()
        flushed = self._injector.configure(config, now_ns=now)
        for message in flushed:
            self._output_publisher.publish(message)
        self._revision = revision
        applied = Int64()
        applied.data = revision
        self._applied_publisher.publish(applied)
        self._publish_counters()

    def _on_input(self, msg: Odometry) -> None:
        now = self._now_ns()
        for output in self._injector.receive(msg, now_ns=now):
            self._output_publisher.publish(output)
        self._publish_counters()

    def _on_poll(self) -> None:
        now = self._now_ns()
        for output in self._injector.poll(now_ns=now):
            self._output_publisher.publish(output)

    def _publish_counters(self) -> None:
        counters = self._injector.counters
        status = DiagnosticStatus()
        status.name = "fault_injector"
        status.message = "counters"
        status.values = [
            KeyValue(key="received", value=str(counters.received)),
            KeyValue(key="forwarded", value=str(counters.forwarded)),
            KeyValue(key="dropped", value=str(counters.dropped)),
        ]
        self._counters_publisher.publish(status)

    def _now_ns(self) -> int:
        now = time.monotonic_ns()
        self._last_ns = max(now, self._last_ns)
        return self._last_ns


def main(args: list[str] | None = None) -> None:
    """Run a standalone FaultInjectorNode with no dynamic reconfiguration."""
    rclpy.init(args=args)
    node = FaultInjectorNode()
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
