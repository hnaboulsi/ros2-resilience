"""A real launch_testing integration test: three separate OS processes.

Unlike ``test_scenarios_end_to_end.py`` (which drives real ROS nodes as
Python objects inside one process via ``run_trial``), this test launches
``robot``, ``fault_injector``, and ``watchdog`` as three independent OS
processes through ``ros2 launch`` machinery and observes them purely over
real topics, with the default (fault-disabled) configuration. It proves the
installed console-script executables actually run and talk to each other,
not just the in-process node classes.

Run explicitly inside a sourced ROS 2 Jazzy environment, e.g.::

    launch_test tests/integration/test_launch_multiprocess.py
"""

from __future__ import annotations

import time
import unittest

import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles

import launch


@pytest.mark.launch_test
def generate_test_description():
    """Launch the three nodes with their default (fault-disabled) topics."""
    robot = launch_ros.actions.Node(package="ros2_resilience", executable="robot", name="robot")
    injector = launch_ros.actions.Node(
        package="ros2_resilience", executable="fault_injector", name="fault_injector"
    )
    watchdog = launch_ros.actions.Node(
        package="ros2_resilience", executable="watchdog", name="watchdog"
    )
    return (
        launch.LaunchDescription([robot, injector, watchdog, launch_testing.actions.ReadyToTest()]),
        {"robot": robot, "injector": injector, "watchdog": watchdog},
    )


class TestResilienceGraphComesUp(unittest.TestCase):
    """Active tests: the launched processes must behave like a real system."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_observer")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_odometry_flows_through_robot_and_injector(self) -> None:
        received: list[Odometry] = []
        qos = QoSPresetProfiles.SENSOR_DATA.value
        sub = self.node.create_subscription(Odometry, "/odom", received.append, qos)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and len(received) < 5:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        assert len(received) >= 5, f"expected odometry on /odom, got {len(received)} messages"

    def test_no_watchdog_violation_on_a_healthy_stream(self) -> None:
        violations: list[DiagnosticStatus] = []
        sub = self.node.create_subscription(
            DiagnosticStatus, "/watchdog/status", violations.append, 10
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        assert violations == []


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):
    """After shutdown, every launched process must have exited cleanly."""

    def test_exit_codes(self, proc_info) -> None:
        launch_testing.asserts.assertExitCodes(proc_info)
