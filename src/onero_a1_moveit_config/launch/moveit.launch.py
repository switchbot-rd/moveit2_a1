"""Onero A1 的 MoveIt 2 启动入口：驱动、规划和可视化。

包含：ros2_control（含话题桥适配层） + move_group + RViz
      + 静态 TF world->base_link

两种模式：

    # 1) 模拟（默认）：经过话题桥，不连接 CAN
    ros2 launch onero_a1_moveit_config moveit.launch.py

    # 2) 真机
    ros2 launch onero_a1_moveit_config moveit.launch.py \
        hardware_mode:=real

无头环境（无 X）请加 rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from onero_a1_bringup.hardware_mode import arm_default, configure_arm, configure_hardware_mode, mode_default
from onero_a1_bringup.controller_tolerances import (
    TOLERANCE_ARGUMENTS, controller_config_from_launch, execution_goal_margin,
    tolerance_launch_arguments,
)
from onero_a1_bringup.payload import payload_launch_arguments


def generate_launch_description():
    moveit_pkg = get_package_share_directory("onero_a1_moveit_config")
    desc_pkg = get_package_share_directory("onero_a1_description")
    bringup_pkg = get_package_share_directory("onero_a1_bringup")

    use_rviz = LaunchConfiguration("rviz")

    args = [
        OpaqueFunction(function=configure_hardware_mode),
        DeclareLaunchArgument("hardware_mode", default_value="sim", choices=["sim", "real"],
                             description="sim: 话题桥模拟，不连接 CAN；real: SDK 真机 CAN"),
        DeclareLaunchArgument("arm", default_value="right", choices=["left", "right"]),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("can_channel", default_value=arm_default("can_channel")),
        DeclareLaunchArgument("model", default_value=arm_default("model")),
        DeclareLaunchArgument(
            "kp", default_value=mode_default("kp"),
            description="全关节位置比例增益；默认 50（模拟不使用增益）",
        ),
        DeclareLaunchArgument(
            "kd", default_value="1.0",
            description="SDK servoj 全关节速度阻尼增益；支持运行时平滑切换",
        ),
        *tolerance_launch_arguments(),
        DeclareLaunchArgument("mounting", default_value=arm_default("mounting")),
        DeclareLaunchArgument(
            "velocity_feedforward", default_value=mode_default("velocity_feedforward"),
            description="全部七关节速度前馈；默认 real=true、sim=false；可显式覆盖"),
        DeclareLaunchArgument(
            "velocity_feedforward_scale", default_value="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]",
            description="兼容参数，仅接受全 1；使用 velocity_feedforward 统一开关"),
        DeclareLaunchArgument("velocity_feedforward_timeout", default_value="0.1"),
        DeclareLaunchArgument("with_gripper", default_value="true"),
        *payload_launch_arguments(),
        DeclareLaunchArgument("gripper_model", default_value="woan_picker", choices=["none", "woan_picker"],
                             description="夹爪模型与开合接口；with_gripper 单独选择 SDK 重力档位"),
        DeclareLaunchArgument("gripper_closed_sdk_opening", default_value="0.0",
                             description="已确认物理闭合位置对应的 SDK 归一化开度，按实物标定"),
        DeclareLaunchArgument("gripper_open_sdk_opening", default_value="1.0"),
        DeclareLaunchArgument("gripper_calibration_file", default_value="",
                             description="默认按型号/CAN在ROS_HOME中保存并加载夹爪标定"),
        DeclareLaunchArgument("gravity_compensation", default_value=mode_default("gravity_compensation"),
                             description="重力补偿默认 real=true、sim=false；显式传参可覆盖"),
        DeclareLaunchArgument(
            "allow_execution", default_value="true",
            description="false: 仅规划预览，不启动轨迹控制器；真机 SDK 仍使能保持当前位置",
        ),
        DeclareLaunchArgument(
            "environment_file", default_value="",
            description="桌面/支架碰撞体 YAML 绝对路径；留空不加载环境",
        ),
        DeclareLaunchArgument(
            "trace_control", default_value="false",
            description="记录实际 SDK 命令及反馈、CAN 收发耗时和丢帧",
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value=mode_default("control_rate"),
            description="话题桥控制频率；默认 real=100 Hz，sim=200 Hz",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value="moveit.rviz",
            description="RViz 配置文件名（位于本包 rviz/ 下）",
        ),
    ]

    def prepare_environment(context):
        controller_config, _ = controller_config_from_launch(
            context, os.path.join(bringup_pkg, "config", "ros2_controllers.yaml"))
        minimum_margin = moveit_config.trajectory_execution.get(
            "trajectory_execution", {}).get("allowed_goal_duration_margin", 2.0)
        goal_margin = execution_goal_margin(controller_config, minimum_margin)
        path = LaunchConfiguration("environment_file").perform(context).strip()
        real_hardware = LaunchConfiguration("hardware_mode").perform(context) == "real"
        allow_execution = IfCondition(LaunchConfiguration("allow_execution")).evaluate(context)
        if path:
            from onero_a1_bringup.environment import load_environment
            path = os.path.abspath(os.path.expanduser(path))
            # Reject missing sizes, incompatible mounting and example-only
            # configurations before bringup can connect to CAN or enable motors.
            load_environment(
                path, mounting=LaunchConfiguration("mounting").perform(context),
                real_hardware=real_hardware,
                allow_execution=allow_execution,
                mirror_for_left=LaunchConfiguration("arm").perform(context) == "left",
            )
        return [
            SetLaunchConfiguration("onero_execution_goal_margin", str(goal_margin)),
            SetLaunchConfiguration("environment_file", path),
            SetLaunchConfiguration("onero_start_arm_controller", "true" if allow_execution and not path else "false"),
            SetLaunchConfiguration("onero_real_hardware", "true" if real_hardware else "false"),
            SetLaunchConfiguration("onero_moveit_controllers", "['arm_controller', 'gripper_controller']"
                                   if LaunchConfiguration("gripper_model").perform(context) == "woan_picker"
                                   else "['arm_controller']"),
            LogInfo(msg=(
                "环境启用：加载并检查场景后处理轨迹控制器启动；真机 SDK 仍按原流程使能并保持当前位置"
                if path else "未配置 environment_file；当前场景没有自动加入桌面/支架碰撞体"
            )),
            LogInfo(msg="轨迹执行已关闭；仅规划预览，真机 SDK 仍使能并保持当前位置",
                    condition=IfCondition("false" if allow_execution else "true")),
        ]

    # ─── MoveIt 参数 ─────────────────────────────────────────────────
    # 用 MoveItConfigsBuilder 拼装，它会自动把各文件放到正确的命名空间下：
    #   kinematics.yaml -> robot_description_kinematics.arm.*
    #   joint_limits.yaml -> robot_description_planning.joint_limits.*
    #   *_planning.yaml -> ompl.* / chomp.* / stomp.*
    # 手写这些前缀极易出错，且出错时往往【静默失效】。
    moveit_config = (
        MoveItConfigsBuilder("onero_a1", package_name="onero_a1_moveit_config")
        .robot_description(
            file_path=os.path.join(desc_pkg, "urdf", "onero_a1.urdf.xacro"),
            mappings={
                "joint_commands_topic": "/robot_joint_commands",
                "joint_states_topic": "/robot_joint_states",
                "gripper_model": LaunchConfiguration("gripper_model"),
                "arm": LaunchConfiguration("arm"),
            },
        )
        .robot_description_semantic(file_path="config/onero_a1.srdf.xacro",
                                    mappings={"gripper_model": LaunchConfiguration("gripper_model"),
                                              "arm": LaunchConfiguration("arm")})
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl", "chomp", "stomp"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # ─── 执行层（ros2_control + 话题桥）───────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in TOLERANCE_ARGUMENTS},
            "arm": LaunchConfiguration("arm"),
            "hardware_mode": LaunchConfiguration("hardware_mode"),
            "can_channel": LaunchConfiguration("can_channel"),
            "model": LaunchConfiguration("model"),
            "kp": LaunchConfiguration("kp"),
            "kd": LaunchConfiguration("kd"),
            "velocity_feedforward": LaunchConfiguration("velocity_feedforward"),
            "velocity_feedforward_scale": LaunchConfiguration("velocity_feedforward_scale"),
            "velocity_feedforward_timeout": LaunchConfiguration("velocity_feedforward_timeout"),
            "mounting": LaunchConfiguration("mounting"),
            "with_gripper": LaunchConfiguration("with_gripper"),
            **{name: LaunchConfiguration(name) for name in ('payload_mass', 'payload_cog', 'payload_transition_time')},
            "gripper_model": LaunchConfiguration("gripper_model"),
            "allow_gripper_execution": LaunchConfiguration("allow_execution"),
            "gripper_closed_sdk_opening": LaunchConfiguration("gripper_closed_sdk_opening"),
            "gripper_open_sdk_opening": LaunchConfiguration("gripper_open_sdk_opening"),
            "gripper_calibration_file": LaunchConfiguration("gripper_calibration_file"),
            "gravity_compensation": LaunchConfiguration("gravity_compensation"),
            "control_rate": LaunchConfiguration("control_rate"),
            "trace_control": LaunchConfiguration("trace_control"),
            "start_arm_controller": LaunchConfiguration("onero_start_arm_controller"),
        }.items(),
    )

    # ─── move_group ──────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {
            "allow_trajectory_execution": ParameterValue(LaunchConfiguration("allow_execution"), value_type=bool),
            "ompl.arm.projection_evaluator": LaunchConfiguration("onero_projection_evaluator"),
            "moveit_simple_controller_manager.arm_controller.joints": LaunchConfiguration("onero_joint_names"),
            "trajectory_execution.allowed_goal_duration_margin": ParameterValue(
                LaunchConfiguration("onero_execution_goal_margin"), value_type=float),
            "moveit_simple_controller_manager.controller_names": LaunchConfiguration("onero_moveit_controllers"),
        }],
    )

    # ─── RViz ────────────────────────────────────────────────────────
    # 注意：不能用 os.path.join 拼 LaunchConfiguration，要用 PathJoinSubstitution
    rviz_config = PathJoinSubstitution(
        [moveit_pkg, "rviz", LaunchConfiguration("rviz_config")]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    environment_loader = Node(
        package="onero_a1_bringup", executable="load_environment", output="screen",
        parameters=[{
            "environment_file": ParameterValue(LaunchConfiguration("environment_file"), value_type=str),
            "mounting": LaunchConfiguration("mounting"),
            "real_hardware": ParameterValue(LaunchConfiguration("onero_real_hardware"), value_type=bool),
            "allow_execution": ParameterValue(LaunchConfiguration("allow_execution"), value_type=bool),
            "gripper_model": LaunchConfiguration("gripper_model"),
            "joints": LaunchConfiguration("onero_joint_names"),
            "mirror_for_left": ParameterValue(LaunchConfiguration("onero_left_arm"), value_type=bool),
        }],
    )
    environment_arm_spawner = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    def after_environment(event, context):
        if context.is_shutdown:
            return []
        if event.returncode == 0:
            if not IfCondition(LaunchConfiguration("allow_execution")).evaluate(context):
                return [LogInfo(msg="环境已就绪，当前仅规划预览，arm_controller 保持未启动"), rviz_node]
            return [environment_arm_spawner]
        # Leave the world visible for diagnosing collisions, but do not start
        # the trajectory controller or remove the objects on initialization failure.
        return [LogInfo(msg="环境初始化/起始姿态检查未通过；arm_controller 未启动，RViz 可用于检查"), rviz_node]

    def after_environment_controller(event, context):
        if context.is_shutdown:
            return []
        message = ("环境和轨迹控制器已就绪" if event.returncode == 0
                   else "轨迹控制器启动失败；请查看 spawner 日志")
        actions = [LogInfo(msg=message), rviz_node]
        if event.returncode == 0 and LaunchConfiguration("gripper_model").perform(context) == "woan_picker":
            actions.append(ExecuteProcess(cmd=["ros2", "service", "call", "/gripper_controller/activate",
                                               "std_srvs/srv/Trigger", "{}"], output="screen"))
        return actions

    # Do not show or plan from MoveIt's default zero pose when the real bridge
    # has not supplied measured joints yet. The bringup launch already gates
    # ros2_control controller activation; this gates the MoveIt UI as well.
    moveit_state_ready = Node(
        package="onero_a1_bringup",
        executable="wait_for_joint_states",
        name="wait_for_moveit_joint_states",
        output="screen",
        parameters=[
            {
                "topic": "/joint_states",
                "gripper_model": LaunchConfiguration("gripper_model"),
                "joints": LaunchConfiguration("onero_joint_names"),
            }
        ],
    )

    def start_moveit_after_state(event, context):
        if context.is_shutdown:
            return []
        if event.returncode == 0:
            if LaunchConfiguration("environment_file").perform(context):
                return [move_group_node, environment_loader]
            return [move_group_node, rviz_node]
        return [LogInfo(msg="未收到有效关节状态；MoveIt 和 RViz 保持未启动")]

    start_moveit_after_state_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=moveit_state_ready,
            on_exit=start_moveit_after_state,
        )
    )

    return LaunchDescription(
        args + [
            OpaqueFunction(function=configure_arm),
            OpaqueFunction(function=prepare_environment),
            RegisterEventHandler(OnProcessExit(target_action=environment_loader, on_exit=after_environment)),
            RegisterEventHandler(OnProcessExit(target_action=environment_arm_spawner, on_exit=after_environment_controller)),
            start_moveit_after_state_ready, bringup, moveit_state_ready,
        ]
    )
