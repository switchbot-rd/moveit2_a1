from glob import glob
from setuptools import find_packages, setup

EXAMPLES = (
    "joint_goal", "named_pose", "pose_goal", "cartesian_path",
    "orientation_constraint", "planning_scene", "collision_check",
    "gripper", "arm_gripper_task", "fixed_pick_place", "record_startup", "return_startup", "freedrive",
    "dual_joint_goal", "dual_cartesian_square", "dual_gripper",
)

setup(
    name="onero_a1_examples", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/onero_a1_examples"]),
        ("share/onero_a1_examples", ["package.xml", "README.md", "SHOWCASE.md", "LICENSE"]),
        ("share/onero_a1_examples/onero_a1_examples", glob("onero_a1_examples/*.py")),
        ("share/onero_a1_examples/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Onero Robotics", maintainer_email="dev@onerobot.com",
    description="Annotated MoveIt 2 usage examples for the Onero A1 adapter",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        f"{name} = onero_a1_examples.{name}:main" for name in EXAMPLES
    ]},
)
