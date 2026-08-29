"""A ROS 2 node wrapping :class:`RobotModel` in real publishers/subscribers."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from ros2_resilience.robot.model import RobotModel

__all__ = ["RobotNode", "main"]


class RobotNode(Node):
    """Publish planar odometry for a robot driven by ``/cmd_vel`` commands."""

    def __init__(self, **kwargs) -> None:
        super().__init__("robot", **kwargs)
        self.declare_parameter("output_topic", "/odom_raw")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("initial_linear_velocity_mps", 0.5)
        self.declare_parameter("initial_angular_velocity_rad_s", 0.0)

        rate_hz = self.get_parameter("publish_rate_hz").value
        self._model = RobotModel(
            initial_linear_x=self.get_parameter("initial_linear_velocity_mps").value,
            initial_angular_z=self.get_parameter("initial_angular_velocity_rad_s").value,
        )
        self._dt_sec = 1.0 / rate_hz

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, qos
        )
        self.create_subscription(
            Twist, self.get_parameter("command_topic").value, self._on_command, 10
        )
        self.create_timer(self._dt_sec, self._on_timer)

    def _on_command(self, msg: Twist) -> None:
        self._model.set_command(linear_x=msg.linear.x, angular_z=msg.angular.z)

    def _on_timer(self) -> None:
        pose = self._model.step(self._dt_sec)
        velocity = self._model.velocity

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.orientation.z = math.sin(pose.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose.theta / 2.0)
        msg.twist.twist.linear.x = velocity.linear_x
        msg.twist.twist.angular.z = velocity.angular_z
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Run a standalone RobotNode."""
    rclpy.init(args=args)
    node = RobotNode()
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
