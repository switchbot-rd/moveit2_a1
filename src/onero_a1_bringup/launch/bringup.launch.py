"""Onero A1 bringup —— 启动 ros2_control 与话题桥适配层。

两种运行模式统一使用 hardware_mode：
    sim（默认）：启动话题桥，用模拟反馈联调，不连接 CAN。
    real：启动话题桥，通过 SDK 控制真机 CAN。

注意：本 launch 只负责【执行层】（ros2_control + 桥接）。
     MoveIt 的 move_group / RViz 由 onero_a1_moveit_config 包启动。

     ros2 launch onero_a1_moveit_config moveit.launch.py hardware_mode:=sim
"""

import json
import os
import subprocess
import sys
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, EnvironmentVariable, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from onero_a1_bringup.hardware_mode import arm_default, configure_arm, configure_hardware_mode, mode_default
from onero_a1_bringup.controller_tolerances import (
    prepare_controller_parameters, tolerance_launch_arguments,
)
from onero_a1_bringup.payload import payload_launch_arguments


def generate_launch_description():
    bringup_share = get_package_share_directory("onero_a1_bringup")
    desc_share = get_package_share_directory("onero_a1_description")

    xacro_file = os.path.join(desc_share, "urdf", "onero_a1.urdf.xacro")
    controllers_file = os.path.join(bringup_share, "config", "ros2_controllers.yaml")

    # ─── 参数 ────────────────────────────────────────────────────────
    simulate = LaunchConfiguration("onero_simulate")
    joint_commands_topic = LaunchConfiguration("joint_commands_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    args = [
        OpaqueFunction(function=configure_hardware_mode),
        DeclareLaunchArgument("hardware_mode", default_value="sim", choices=["sim", "real"],
                             description="sim: 话题桥模拟，不连接 CAN；real: SDK 真机 CAN"),
        DeclareLaunchArgument("arm", default_value="right", choices=["left", "right"]),
        DeclareLaunchArgument('enable_servo', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('sim_initial_positions', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument("joint_commands_topic", default_value="/robot_joint_commands"),
        DeclareLaunchArgument("joint_states_topic", default_value="/robot_joint_states"),
        DeclareLaunchArgument("can_channel", default_value=arm_default("can_channel")),
        DeclareLaunchArgument("model", default_value=arm_default("model")),
        DeclareLaunchArgument("kp", default_value=mode_default("kp"),
                             description="默认 50（模拟不使用增益）"),
        DeclareLaunchArgument("kd", default_value="1.0"),
        *tolerance_launch_arguments(),
        *payload_launch_arguments(),
        DeclareLaunchArgument("velocity_feedforward", default_value=mode_default("velocity_feedforward"),
                             description="全部七关节速度前馈；默认 real=true、sim=false；可显式覆盖"),
        DeclareLaunchArgument("velocity_feedforward_scale", default_value="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]",
                             description="兼容参数，仅接受全 1；使用 velocity_feedforward 统一开关"),
        DeclareLaunchArgument("velocity_feedforward_timeout", default_value="0.1"),
        DeclareLaunchArgument("mounting", default_value=arm_default("mounting")),
        DeclareLaunchArgument("with_gripper", default_value="true"),
        DeclareLaunchArgument("gripper_model", default_value="woan_picker", choices=["none", "woan_picker"]),
        DeclareLaunchArgument("allow_gripper_execution", default_value="true"),
        DeclareLaunchArgument("gripper_closed_sdk_opening", default_value="0.0"),
        DeclareLaunchArgument("gripper_open_sdk_opening", default_value="1.0"),
        DeclareLaunchArgument("gripper_calibration_file", default_value=""),
        DeclareLaunchArgument("gravity_compensation", default_value=mode_default("gravity_compensation"),
                             description="默认 real=true、sim=false；显式传参可覆盖"),
        DeclareLaunchArgument(
            "start_arm_controller", default_value="true",
            description="是否自动启动轨迹控制器；MoveIt 加载环境时延后启动",
        ),
        DeclareLaunchArgument(
            "runtime_python", default_value=EnvironmentVariable(
                "ONERO_RUNTIME_PYTHON", default_value=sys.executable
            ),
            description="本项目话题桥使用的 Python；setup_env.sh 优先选取 .venv",
        ),
        DeclareLaunchArgument("control_rate", default_value=mode_default("control_rate"),
                             description="话题桥默认 real=100 Hz，sim=200 Hz"),
        DeclareLaunchArgument(
            "trace_control", default_value="false",
            description="发布 SDK 命令、反馈和 CAN 周期诊断，供 record_tracking.py 记录",
        ),
        DeclareLaunchArgument(
            "publish_world_tf",
            default_value="true",
            description="是否发布 world -> base_link 的静态 TF",
        ),
        DeclareLaunchArgument(
            "watchdog_timeout",
            default_value="1.0",
            description="话题桥失联判定阈值（秒），超时则停用控制器",
        ),
        DeclareLaunchArgument(
            "watchdog_auto_deactivate",
            default_value="true",
            description="失联时是否自动停用 arm_controller",
        ),
    ]

    # ─── 机器人描述 ──────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(
            [
                "xacro ", xacro_file,
                " joint_commands_topic:=", joint_commands_topic,
                " joint_states_topic:=", joint_states_topic,
                " gripper_model:=", LaunchConfiguration("gripper_model"),
                " arm:=", LaunchConfiguration("arm"),
            ]
        ),
        value_type=str,
    )

    # ─── TF ──────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

    # MoveIt 的 SRDF virtual_joint 声明 base_link 挂在 world 下，
    # 因此 TF 树里也需要 world -> base_link
    def make_world_tf(context):
        mounting = LaunchConfiguration("mounting").perform(context)
        quaternion = [0.0, 0.0, 0.0, 1.0]
        if mounting != "upright":
            # Query the pip-installed SDK with the same interpreter as the
            # bridge; ROS launch itself does not need a second SDK installation.
            from scipy.spatial.transform import Rotation
            query = (
                "import json, sys; "
                "from onero_sdk.mounting import get_mounting_rotation, resolve_mounting; "
                "print(json.dumps(get_mounting_rotation(resolve_mounting(sys.argv[1])).tolist()))"
            )
            try:
                result = subprocess.run(
                    [LaunchConfiguration("runtime_python").perform(context), "-c", query, mounting],
                    check=True, capture_output=True, text=True, timeout=10,
                )
                rotation = json.loads(result.stdout)
            except (OSError, subprocess.SubprocessError, ValueError) as exc:
                raise RuntimeError(
                    "无法从项目运行环境读取 onero-sdk 安装方向；请重新 source setup_env.sh。"
                ) from exc
            quaternion = Rotation.from_matrix(
                rotation
            ).as_quat().tolist()
        return [Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="world_to_base_link", output="log",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--qx", str(quaternion[0]), "--qy", str(quaternion[1]),
                       "--qz", str(quaternion[2]), "--qw", str(quaternion[3]),
                       "--frame-id", "world", "--child-frame-id", "base_link"],
        )]

    world_tf = OpaqueFunction(
        function=make_world_tf,
        condition=IfCondition(LaunchConfiguration("publish_world_tf")),
    )

    # ─── ros2_control ────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # Topic publication must not hold the 100 Hz Servo control thread.
        additional_env={'RMW_FASTRTPS_PUBLICATION_MODE': IfElseSubstitution(
            LaunchConfiguration('enable_servo'), 'ASYNCHRONOUS',
            EnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE', default_value='SYNCHRONOUS'))},
        parameters=[ParameterFile(LaunchConfiguration("onero_controllers_file")),
                    {"robot_description": robot_description}],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    servo_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['servo_arm_controller', '-c', '/controller_manager', '--inactive'], output='screen')

    # Position states and commands start as NaN. Wait for fresh
    # bridge state before starting joint_state_broadcaster, so NaNs never reach
    # MoveIt/RSP. Then wait for the broadcaster's finite state before activating
    # JTC; activation must never use 0.0 as a fallback target for a real arm.
    bridge_state_ready = Node(
        package="onero_a1_bringup",
        executable="wait_for_joint_states",
        name="wait_for_bridge_joint_states",
        output="screen",
        parameters=[
            {
                "topic": joint_states_topic,
                "joints": LaunchConfiguration("onero_joint_names"),
            }
        ],
    )

    joint_state_ready = Node(
        package="onero_a1_bringup",
        executable="wait_for_joint_states",
        name="wait_for_valid_joint_states",
        output="screen",
        parameters=[
            {
                "joints": LaunchConfiguration("onero_joint_names"),
                "topic": "/joint_states",
                "gripper_model": LaunchConfiguration("gripper_model"),
            }
        ],
    )

    def spawn_broadcaster_after_bridge_state(event, _context):
        if event.returncode == 0:
            return [joint_state_broadcaster_spawner]
        return [LogInfo(msg="未收到话题桥关节状态；状态广播器和 arm_controller 保持未启动")]

    def activate_controller_after_state(event, context):
        servo = [servo_controller_spawner] if IfCondition(LaunchConfiguration('enable_servo')).evaluate(context) else []
        if not IfCondition(LaunchConfiguration("start_arm_controller")).evaluate(context):
            return servo+[LogInfo(msg="当前启动配置禁止自动启动 arm_controller（仅预览或等待环境检查）")] if event.returncode == 0 else []
        if event.returncode == 0:
            return servo+[arm_controller_spawner]
        return [LogInfo(msg="未收到有效关节状态；arm_controller 保持未启动")]

    # ─── 话题桥适配节点 ─────────────────────────────────────────────
    bridge_node = Node(
        package="onero_a1_bringup",
        executable="topic_bridge",
        prefix=[LaunchConfiguration("runtime_python")],
        name="onero_topic_bridge",
        output="screen",
        parameters=[
            {
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
                "joint_names": LaunchConfiguration("onero_joint_names"),
                "simulate": ParameterValue(simulate, value_type=bool),
                'sim_initial_positions': ParameterValue(LaunchConfiguration('sim_initial_positions'), value_type=List[float]),
                "trace_control": ParameterValue(
                    LaunchConfiguration("trace_control"), value_type=bool
                ),
                "control_rate": ParameterValue(
                    LaunchConfiguration("control_rate"), value_type=float
                ),
                "can_channel": LaunchConfiguration("can_channel"),
                "model": LaunchConfiguration("model"),
                "mounting": LaunchConfiguration("mounting"),
                "with_gripper": ParameterValue(LaunchConfiguration("with_gripper"), value_type=bool),
                'payload_mass': ParameterValue(LaunchConfiguration('payload_mass'), value_type=float),
                'payload_cog': ParameterValue(LaunchConfiguration('payload_cog'), value_type=List[float]),
                'payload_transition_time': ParameterValue(LaunchConfiguration('payload_transition_time'), value_type=float),
                "gripper_model": LaunchConfiguration("gripper_model"),
                "allow_gripper_execution": ParameterValue(LaunchConfiguration("allow_gripper_execution"), value_type=bool),
                "activate_gripper_on_start": ParameterValue(LaunchConfiguration("start_arm_controller"), value_type=bool),
                "gripper_closed_sdk_opening": ParameterValue(LaunchConfiguration("gripper_closed_sdk_opening"), value_type=float),
                "gripper_open_sdk_opening": ParameterValue(LaunchConfiguration("gripper_open_sdk_opening"), value_type=float),
                "gripper_calibration_file": ParameterValue(LaunchConfiguration("gripper_calibration_file"), value_type=str),
                "gravity_compensation": ParameterValue(
                    LaunchConfiguration("gravity_compensation"), value_type=bool
                ),
                "kp": ParameterValue(LaunchConfiguration("kp"), value_type=float),
                "kd": ParameterValue(LaunchConfiguration("kd"), value_type=float),
                "velocity_feedforward": ParameterValue(
                    LaunchConfiguration("velocity_feedforward"), value_type=bool),
                "velocity_feedforward_scale": LaunchConfiguration("velocity_feedforward_scale"),
                "velocity_feedforward_timeout": ParameterValue(
                    LaunchConfiguration("velocity_feedforward_timeout"), value_type=float),
            }
        ],
    )

    spawn_broadcaster_after_bridge = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge_state_ready,
            on_exit=spawn_broadcaster_after_bridge_state,
        )
    )
    spawn_arm_after_valid_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_ready,
            on_exit=activate_controller_after_state,
        )
    )

    # ─── 健康看门狗 ─────────────────────────────────────────────────
    # 硬件接口不校验状态时间戳，桥接节点挂掉时它会一直返回最后一帧，
    # MoveIt 会误判为正常。此节点负责失联时停用控制器。
    watchdog_node = Node(
        package="onero_a1_bringup",
        executable="health_watchdog",
        name="onero_bridge_watchdog",
        output="screen",
        parameters=[
            {
                "joint_states_topic": joint_states_topic,
                "controller_name": "arm_controller",
                'controller_names': LaunchConfiguration('onero_watchdog_controllers'),
                'auto_reactivate': ParameterValue(LaunchConfiguration('onero_watchdog_reactivate'), value_type=bool),
                "timeout": ParameterValue(
                    LaunchConfiguration("watchdog_timeout"), value_type=float
                ),
                "auto_deactivate": ParameterValue(
                    LaunchConfiguration("watchdog_auto_deactivate"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        args
        + [
            OpaqueFunction(function=configure_arm),
            OpaqueFunction(function=prepare_controller_parameters,
                           kwargs={"config_file": controllers_file}),
            robot_state_publisher,
            world_tf,
            ros2_control_node,
            bridge_node,
            watchdog_node,
            bridge_state_ready,
            joint_state_ready,
            spawn_broadcaster_after_bridge,
            spawn_arm_after_valid_state,
        ]
    )
