"""Isolated MoveIt model/scene validation server; no controllers or hardware."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from onero_a1_bringup.dual_config import moveit_parameters


def setup(context):
    snapshot = Path(LaunchConfiguration('snapshot').perform(context)).resolve()
    config = (MoveItConfigsBuilder('onero_a1', package_name='onero_a1_moveit_config')
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    params = moveit_parameters(config, 'woan_picker')
    params.update(robot_description=(snapshot/'robot.urdf').read_text(),
                  robot_description_semantic=(snapshot/'robot.srdf').read_text(),
                  allow_trajectory_execution=False,
                  publish_robot_description=True, publish_robot_description_semantic=True)
    return [Node(package='moveit_ros_move_group', executable='move_group', parameters=[params], output='screen'),
            Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
                '--frame-id', 'world', '--child-frame-id', 'base_link'], output='log')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('snapshot'), OpaqueFunction(function=setup)])
