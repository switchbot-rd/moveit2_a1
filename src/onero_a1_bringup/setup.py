from glob import glob
import os

from setuptools import find_packages, setup

package_name = "onero_a1_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml", "LICENSE"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Onero Robotics",
    maintainer_email="dev@onerobot.com",
    description="Launch files and ros2_control configuration for the Onero A1 arm",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "topic_bridge = onero_a1_bringup.topic_bridge:main",
            "health_watchdog = onero_a1_bringup.health_watchdog:main",
            'servo_manager = onero_a1_bringup.servo_manager:main',
            "wait_for_joint_states = onero_a1_bringup.wait_for_joint_states:main",
            "load_environment = onero_a1_bringup.environment:main",
            "woan_gripper_controller = onero_a1_bringup.woan_gripper:main",
        ],
    },
)
