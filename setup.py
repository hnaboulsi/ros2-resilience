import glob

from setuptools import find_packages, setup

package_name = "ros2_resilience"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("tests", "tests.*")),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*.launch.py")),
        (f"share/{package_name}/scenarios", glob.glob("scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hassan Naboulsi",
    maintainer_email="hassannaboulsi@gmail.com",
    description="Closed-loop resilience regression testing for ROS 2 systems.",
    license="Apache-2.0",
    python_requires=">=3.12",
    extras_require={"dev": ["pytest>=8,<10", "ruff>=0.9,<1"]},
    entry_points={
        "console_scripts": [
            "fault_injector = ros2_resilience.ros.fault_injector_node:main",
            "robot = ros2_resilience.ros.robot_node:main",
            "watchdog = ros2_resilience.ros.watchdog_node:main",
            "run_scenario = ros2_resilience.ros.experiment_runner_node:main",
        ],
    },
)
