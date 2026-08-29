"""Manually launch the robot/injector/watchdog graph for one scenario.

This is a convenience launcher for interactive observation (``ros2 topic
echo``, ``rqt``, etc.) with the fault applied for the whole session. It does
not run the phased CREATED -> ... -> FINISHED lifecycle, activation timing,
or assertion evaluation -- that automated, repeatable orchestration lives in
the ``run_scenario`` console script (``ros2_resilience.ros.experiment_runner_node``),
which is what CI and repeated-trial batches use.

Usage::

    ros2 launch ros2_resilience resilience_trial.launch.py \\
        scenario:=scenarios/dropout_burst.yaml
"""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from ros2_resilience.core.scenario import FaultKind
from ros2_resilience.core.validation import load_scenario


def _launch_setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    scenario_path = LaunchConfiguration("scenario").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    scenario = load_scenario(scenario_path)

    robot_node = Node(
        package="ros2_resilience",
        executable="robot",
        namespace=namespace,
        parameters=[
            {
                "output_topic": scenario.system.input_topic,
                "command_topic": scenario.system.command_topic,
                "publish_rate_hz": scenario.system.robot.publish_rate_hz,
                "initial_linear_velocity_mps": scenario.system.robot.initial_linear_velocity_mps,
                "initial_angular_velocity_rad_s": (
                    scenario.system.robot.initial_angular_velocity_rad_s
                ),
            }
        ],
    )
    injector_node = Node(
        package="ros2_resilience",
        executable="fault_injector",
        namespace=namespace,
        parameters=[
            {
                "input_topic": scenario.system.input_topic,
                "output_topic": scenario.system.output_topic,
            }
        ],
    )
    watchdog_node = Node(
        package="ros2_resilience",
        executable="watchdog",
        namespace=namespace,
        parameters=[
            {
                "input_topic": scenario.system.output_topic,
                "command_topic": scenario.system.command_topic,
                "max_age_sec": scenario.watchdog.max_age_sec,
                "expected_rate_hz": scenario.watchdog.expected_rate_hz,
                "loss_window_sec": scenario.watchdog.loss_window_sec,
                "max_loss_fraction": scenario.watchdog.max_loss_fraction,
                "check_period_sec": scenario.watchdog.check_period_sec,
            }
        ],
    )

    if scenario.fault.type is not FaultKind.NONE:
        print(
            f"NOTE: fault '{scenario.fault.type.value}' is defined in {scenario_path} but this "
            "launch file starts every node with faults disabled. Use 'run_scenario' for an "
            "automated trial, or apply the fault by hand once nodes are up."
        )

    return [robot_node, injector_node, watchdog_node]


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for one manually-observed scenario."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario", description="Path to a resilience-contract YAML file"
            ),
            DeclareLaunchArgument("namespace", default_value="", description="ROS namespace"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
