# Onero A1 × MoveIt 2

将 Onero A1 七自由度机械臂和 Woan Picker 夹爪接入 MoveIt 2 的 ROS 2 工作区。
支持单臂（左臂或右臂）及双臂规划、轨迹执行、碰撞场景和夹爪开合。
机械臂通过标准 FollowJointTrajectory / ros2_control 接口执行，适配层调用 onero-sdk 控制 CAN。

参数速查：[服务启动参数](#服务启动参数) · [示例参数](src/onero_a1_examples/README.md#示例参数)。

## 项目结构

| 位置 | 职责 |
|---|---|
| `src/onero_a1_description` | 机械臂、夹爪的 URDF、网格与 ros2_control 硬件描述 |
| `src/onero_a1_bringup` | 话题桥、onero-sdk 会话、控制器、状态看门狗与环境加载 |
| `src/onero_a1_moveit_config` | SRDF、运动学、规划限位、OMPL/CHOMP/STOMP、控制器映射和启动入口 |
| `src/onero_a1_common` | 可安装的公共 MoveIt Python 客户端 |
| `src/onero_a1_examples` | 功能示例，一项功能一个源码文件 |
| `tools/diagnostics` | 控制链路记录、精度验收、姿态存档、固定轨迹回放与 onero-sdk 对照 |

```text
onero_a1_examples ──▶ onero_a1_common
                 └─▶ onero_a1_moveit_config ──▶ onero_a1_bringup ──▶ onero_a1_description
tools/diagnostics ──▶ onero_a1_common
```

## 环境准备与构建

使用 Ubuntu 24.04 和已安装 ROS 2 Jazzy 的系统。在工作区根目录执行：

```bash
source setup_env.sh
```

`setup_env.sh` 统一检查并安装所需系统软件包、准备 `.venv`、通过 pip 安装 onero-sdk，
自动构建并加载工作区环境。缺失的系统软件包通过 apt 安装，需要系统安装权限。
首次运行或依赖配置变化时安装 Python 依赖；后续终端直接复用已有环境。
首次运行或 `src` 源码变化时自动执行增量构建；构建成功后立即加载结果。
源码没有变化时跳过构建，每个新终端执行同一条命令即可。
多个终端同时执行时，依赖安装和构建会排队进行；等待结束后重新检查状态，各终端分别加载环境。
ROS 日志位于工作区 `.ros/log/`。
`overlay/` 提供话题桥依赖的本地覆盖；安装对应系统软件包后可直接使用系统依赖。
无图形界面时，启动命令添加 `rviz:=false`。

脚本在工作区内的 `.venv` 中通过 pip 安装
[`onero-sdk[dynamics]==0.3.0`](https://pypi.org/project/onero-sdk/0.3.0/)。
重力补偿由 onero-sdk 计算，所需 Pinocchio 等依赖由其 `dynamics` 扩展声明。
虚拟环境使用 `--system-site-packages` 访问系统安装的 ROS 2 Python 接口。

| 文件 | 职责 |
|---|---|
| `tools/requirements-runtime.txt` | 声明 onero-sdk 版本及动力学扩展 |
| `tools/constraints-runtime.txt` | 约束 NumPy、Pinocchio 及相关库版本，保持 ROS 2 Jazzy 兼容 |

每个终端执行 `source setup_env.sh` 后，会自动激活 `.venv`，当前终端的 `python3` 和 pip 使用项目环境。
话题桥及安装方向查询同样使用 `.venv/bin/python`；MoveIt、RViz 等程序按各自的启动方式运行。
需要使用另一个已安装相同依赖的 Python 环境时，设置 `ONERO_RUNTIME_PYTHON` 为其解释器路径。

## 启动服务

| 控制对象 | 启动入口 |
|---|---|
| 单臂（右臂或左臂） | `ros2 launch onero_a1_moveit_config moveit.launch.py` |
| 双臂 | `ros2 launch onero_a1_moveit_config dual_moveit.launch.py` |

```bash
# 单臂模拟
ros2 launch onero_a1_moveit_config moveit.launch.py hardware_mode:=sim

# 单臂真机：加载桌面模型，预览规划路径
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real allow_execution:=false \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

左单臂添加 `arm:=left`。切换选臂时，显式填写的 `mounting` 需同步改为
`humanoid_left` / `humanoid_right`；省略时按选臂自动设置。
`allow_execution:=false` 时真机仍会使能并保持实测姿态。
切换运行模式前结束上一套服务，各客户端与服务端使用同一个 ROS 域。

`sim` 模拟关节跟随和夹爪开合，不连接 CAN，不计算重力、摩擦或接触动力学。
`real` 通过 onero-sdk 连接 CAN，启动时读取并保持实测姿态。

## 双臂控制

```bash
# 双臂模拟
ros2 launch onero_a1_moveit_config dual_moveit.launch.py hardware_mode:=sim

# 双臂真机
ros2 launch onero_a1_moveit_config dual_moveit.launch.py hardware_mode:=real
```

两臂共用一个规划场景，分别使用各自的 CAN 通道、控制器和夹爪接口。
任何一臂反馈失联会停用两个轨迹控制器，恢复后重新启动服务。
使用方法见[双臂示例](src/onero_a1_examples/README.md#双臂控制)。

## 服务启动参数

写法：`ros2 launch onero_a1_moveit_config <启动文件> 参数:=值`。
以下只列当前启动入口可配置的参数；省略时使用表内默认值，显式赋值优先。
“单臂”指 `moveit.launch.py`，“双臂”指 `dual_moveit.launch.py`。

分类：[基础启动](#基础启动) · [单臂安装](#单臂安装) · [双臂安装](#双臂安装) ·
[运动控制](#运动控制参数) · [执行容差](#机械臂执行容差) · [夹爪](#夹爪参数) ·
[监控与记录](#监控与记录) · [高级设置](#高级设置)。

### 基础启动

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `hardware_mode` | `sim` | `sim`、`real` | 选择模拟或真机 | 单臂、双臂 |
| `rviz` | `true` | `true`、`false` | 是否打开 RViz | 单臂、双臂 |
| `allow_execution` | `true` | `true`、`false` | 是否允许手臂轨迹与夹爪动作；`false` 仅预览，真机仍使能保持 | 单臂、双臂 |
| `environment_file` | 留空 | 碰撞环境 YAML 的绝对路径 | 加载桌面、支架等碰撞体；留空时不加载 | 单臂、双臂 |

### 单臂安装

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `arm` | `right` | `left`、`right` | 选择左臂或右臂 | 单臂 |
| `model` | 随 `arm`：左 `A1L`，右 `A1R` | `A1L`、`A1R`，必须与选臂匹配 | 选择机械臂型号 | 单臂 |
| `can_channel` | 随 `arm`：左 `can1`，右 `can0` | 实际 CAN 接口名，如 `can0`、`slcan0` | 指定真机通信接口 | 单臂 |
| `mounting` | 随 `arm`：左 `humanoid_left`，右 `humanoid_right` | `humanoid_left`、`humanoid_right`、`upright`、`a1_vertical`、`side_mount_left`、`side_mount_right`、`ceiling` | 安装方向，用于坐标映射和重力补偿 | 单臂 |

**将 `arm` 改为 `left` 时，显式填写的 `mounting` 需同步改为 `humanoid_left`；右臂对应 `humanoid_right`。**
使用单臂桌面配置时，左臂按镜像安装加载环境。

### 双臂安装

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `left_can_channel` | `can1` | 左臂实际 CAN 接口名 | 指定左臂真机通信接口 | 双臂 |
| `right_can_channel` | `can0` | 右臂实际 CAN 接口名，需与左臂不同 | 指定右臂真机通信接口 | 双臂 |
| `arm_spacing` | `0.07` | 米 | 两根部安装中心的左右间距，需与环境文件一致 | 双臂 |

### 运动控制参数

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `kp` | `50` | `[0, 500]` | 全关节位置增益，支持运行时平滑切换 | 单臂、双臂 |
| `kd` | `1.0` | `[0, 5]` | 全关节速度阻尼增益，支持运行时平滑切换 | 单臂、双臂 |
| `control_rate` | sim：`200`；real：`100` | 正数，Hz | 控制器与话题桥控制频率 | 单臂、双臂 |
| `gravity_compensation` | sim：`false`；real：`true` | `true`、`false` | 真机重力补偿 | 单臂、双臂 |
| `payload_mass` | `0.0` | 非负数，kg | 启动时空夹爪基线的额外负载质量 | 单臂 |
| `payload_cog` | `[0.0, 0.0, 0.0]` | 三个浮点数，m | 基线重心，相对 SDK 末端坐标系 | 单臂 |
| `left_payload_mass` / `right_payload_mass` | `0.0` | 非负数，kg | 各臂基线的额外负载质量 | 双臂 |
| `left_payload_cog` / `right_payload_cog` | `[0.0, 0.0, 0.0]` | 三个浮点数，m | 各臂基线负载重心 | 双臂 |
| `payload_transition_time` | `0.5` | 正数，s | 运行时负载平滑切换时长 | 单臂、双臂 |
| `velocity_feedforward` | sim：`false`；real：`true` | `true`、`false` | 同时开启或关闭七个关节的目标速度前馈 | 单臂、双臂 |
| `velocity_feedforward_timeout` | `0.1` | 秒 | 速度命令超过该时间未更新时，速度前馈归零 | 单臂 |

模拟模式不使用电机增益和重力补偿。双臂的公共启动参数同时作用于两只手臂，运行时通过各臂桥接节点分别调整。

### 机械臂执行容差

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `arm_trajectory_tolerance` | 沿用 YAML：`0.20` | 正数，rad | 运动过程中允许的关节位置跟踪误差 | 单臂、双臂 |
| `arm_goal_tolerance` | 沿用 YAML：`0.1` | 正数，rad | 轨迹结束时允许的关节位置误差 | 单臂、双臂 |
| `arm_goal_time_tolerance` | 沿用 YAML：`1.0` | 正数，秒 | 轨迹结束后，为满足到位条件允许额外等待的时间 | 单臂、双臂 |
| `arm_stopped_velocity_tolerance` | 沿用 YAML：`0.02` | 正数，rad/s | 轨迹结束时允许的关节速度 | 单臂、双臂 |

省略时读取 `src/onero_a1_bringup/config/ros2_controllers.yaml`；显式填写时统一覆盖关节配置。
参数供 RViz 与示例共同使用。放宽容差会改变成功判定，不会提高实物跟踪精度。

### 夹爪参数

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `gripper_model` | `woan_picker` | `woan_picker`、`none` | 选择夹爪模型和开合接口 | 单臂、双臂 |
| `with_gripper` | `true` | `true`、`false` | 启用 onero-sdk 带夹爪重力补偿档位 | 单臂、双臂 |
| `gripper_calibration_file` | 留空，按型号和 CAN 通道自动选择 | 标定文件路径；`none` 禁用文件 | 加载或保存夹爪开合标定 | 单臂 |
| `gripper_closed_sdk_opening` | `0.0` | `[0, 1]`，需小于全开端点 | 手工指定全闭时对应的 onero-sdk 开度 | 单臂 |
| `gripper_open_sdk_opening` | `1.0` | `[0, 1]`，需大于全闭端点 | 手工指定全开时对应的 onero-sdk 开度 | 单臂 |

自动标定文件优先于手工端点。双臂按各自型号和 CAN 通道自动加载标定文件。

### 监控与记录

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `watchdog_timeout` | `1.0` | 秒 | 关节反馈失联判定时间 | 单臂 |
| `watchdog_auto_deactivate` | `true` | `true`、`false` | 反馈失联时是否自动停用轨迹控制器 | 单臂 |
| `trace_control` | `false` | `true`、`false` | 发布命令、反馈、CAN 周期等诊断数据 | 单臂 |

### 高级设置

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用入口 |
|---|---|---|---|---|
| `runtime_python` | `ONERO_RUNTIME_PYTHON`，通常为 `.venv/bin/python` | Python 解释器路径 | 运行话题桥；单臂还用其查询安装方向 | 单臂、双臂 |
| `rviz_config` | `moveit.rviz` | 本包 `rviz/` 下的配置文件名 | 选择 RViz 布局 | 单臂 |
| `publish_world_tf` | `true` | `true`、`false` | 是否发布 `world → base_link`；关闭时需另行提供 TF | 单臂 |
| `joint_commands_topic` | `/robot_joint_commands` | ROS 话题名 | 控制器向话题桥发送关节命令的话题 | 单臂 |
| `joint_states_topic` | `/robot_joint_states` | ROS 话题名 | 话题桥向控制器发布关节反馈的话题 | 单臂 |

## 规划器选择

支持 OMPL、CHOMP、STOMP，默认使用 OMPL / RRTConnect。
各示例的支持情况见 [规划器选择](src/onero_a1_examples/README.md#规划器选择)，
命令行选项集中列在 [示例参数](src/onero_a1_examples/README.md#示例参数)。

## 功能示例与公共接口

使用方法及源码入口见 [Examples 指南](src/onero_a1_examples/README.md)。

## 控制链路

```text
MoveIt ExecuteTrajectory
  → arm_controller /follow_joint_trajectory
  → ros2_control position + velocity 命令接口
  → /robot_joint_commands
  → onero_topic_bridge → onero-sdk Arm.servoj() → CAN → 电机

电机反馈 → onero-sdk → /robot_joint_states → joint_state_broadcaster → /joint_states → MoveIt
```

JTC 与话题桥传递同一采样点的位置、速度，按关节名匹配。
`velocity_feedforward:=true` 启用七个关节的目标速度前馈，false 将七关节期望速度归零。
命令缺失、非有限、时间戳无效或超过 `velocity_feedforward_timeout` 时，
速度前馈归零。取消、保持、重新使能时清除缓存速度。

`kp` / `kd` 为 onero-sdk servoj 的全关节增益，支持运行时通过 ROS 参数单独或原子更新，在 0.5 秒内平滑切换，以桥接节点的 `gain_state` 确认生效。
`gravity_compensation:=true` 调用 onero-sdk `gravity_torque()`，按实测姿态、安装与负载计算自重补偿，
经 `tau_ff` 下发；力矩前馈受 onero-sdk 与适配层的限幅约束。
动力学依赖和模型在打开 CAN 前检查。模拟模式不计算动力学。

## 桌面与支架环境

`environment_file` 接收碰撞体 YAML 绝对路径，通过 MoveIt Planning Scene 加载环境。
未指定时不自动加入桌面或支架，关节反馈不包含环境识别信息。

单臂桌面与支架配置使用 `environment.desktop.yaml`，双臂使用 `environment.dual_desktop.yaml`。
使用前核对文件内的尺寸、位置和安装方式。
真机执行拒绝使用 `example_only: true` 的配置；预览可使用 `allow_execution:=false`。

双臂的 world 原点在两个根部安装中心的中点，轴方向与单臂一致；
左右根部的 Y 坐标分别为 +0.035 m、−0.035 m。环境文件中的安装间距必须与启动参数一致。

YAML 中 `frame_id` 为 world，`xyz` 表示长方体中心，`size` 为三个方向尺寸，单位为米；
`rpy` 为弧度，`color` 为 RGBA。`mounting` 必须与启动参数一致。
`environment.desktop.yaml` 描述木桌、深色底板、金属底座、立柱和连接板：

| 项目 | 几何约定 |
|---|---|
| 根部安装中心至深色底板顶面 | 0.330 m |
| 立柱 | 0.060 × 0.060 × 0.370 m |
| 金属底座（前后 × 横向 × 厚度） | 0.100 × 0.150 × 0.005 m |
| 连接板（前后 × 厚度 × 高度） | 0.160 × 0.005 × 0.080 m |
| 深色底板 | 1.000 × 1.000 × 0.010 m，顶面 z=-0.330 m |
| 木桌 | 1.000 × 1.000 × 0.030 m，顶面 z=-0.340 m |

环境配置不校准机械臂运动学、关节零位或负载模型，标记设为 false 也不能代替尺寸核对。
其他机械臂、线缆和桌面物品需另外建模；夹爪几何由 `gripper_model` 加入机器人模型。

启动流程为：检查配置 → 等待实测关节反馈 → 加载并回读场景 → 检查起始姿态 → 启动轨迹控制器。
应等待环境就绪后再发送规划请求。起点碰撞时核对模型与实际姿态，加载器不会自动放行接触。
环境对象保存在 MoveIt 场景中；加载器退出不移除对象，手动修改对象后以场景中的数据为准。

## Woan Picker

`gripper_model:=woan_picker` 启用夹爪外形、碰撞体、mimic 机构、gripper 规划组和开合接口。
`gripper_model:=none` 用于无夹爪模型场景。

| 接口 | 说明 |
|---|---|
| `/gripper_controller/gripper_cmd` | control_msgs/action/GripperCommand |
| `command.position` | 模型主关节角度：0 全闭，1.1 rad 全开 |
| `command.max_effort` | 仅接受 0，使用 onero-sdk 位置控制参数 |
| `/gripper_controller/state` | 开度、onero-sdk 反馈、活动标定及指令状态 |
| `/gripper_controller/calibrate` | 空载自动全开、全闭并保存端点 |
| `/gripper_controller/stop` | 停止当前动作/标定，保持反馈开度 |

夹爪与机械臂共用 onero-sdk CAN transport 和 I/O 锁。启动只查询开度，收到动作目标才使能并开合。
夹爪主关节发布到 `/joint_states`，其他机构由 URDF mimic 联动。

空载且完整开合范围无遮挡时标定：

```bash
ros2 service call /gripper_controller/calibrate std_srvs/srv/Trigger '{}'
```

标定分别保存原生指令端点与稳定反馈端点；全开/全闭使用驱动端点，中间开度按实测反馈行程换算。
普通目标在标定期间拒绝接收。故障、反馈异常、超时或停止会中断标定并保留有效配置。
取消服务调用客户端不撤销服务端标定；需要中止时调用：

```bash
ros2 service call /gripper_controller/stop std_srvs/srv/Trigger '{}'
```

标定文件按型号和 CAN 通道命名，例如 `.ros/gripper_calibration/A1R_can0_woan_picker.json`。
启动自动加载匹配的标定；`gripper_calibration_file` 可指定路径，`none` 禁用文件。
型号、通道、仿真模式必须与文件一致，更换设备或端点定义后重新标定。
开合标定相关启动选项见[夹爪参数](#夹爪参数)。

模型保留 onero-sdk 的安装变换和 CAD 惯性；选择视觉模型不会自动修改 onero-sdk payload 或 TCP。
onero-sdk 原装夹爪补偿档位由 `with_gripper` 单独控制，单臂和双臂默认均为 `true`，
按带夹爪的经验系数修正重力补偿力矩。实物未装夹爪时设置 `with_gripper:=false`。
夹爪开合不包含带物夹取成功判定。

## 模型与参数配置

机械臂和夹爪的 URDF、SRDF 与网格资源随本项目提供，常规安装和启动直接使用这些资源。
模型来自 onero-sdk 的 A1L/A1R 和 Woan Picker 描述，保留连杆/关节几何与限位。
机械臂描述位于 `src/onero_a1_description`，规划组与命名姿态位于
`src/onero_a1_moveit_config/config`。

本体碰撞配置排除直接相邻连杆，夹爪配置包含机构连接和闭合接触对。
模型来源和网格校验值见 `src/onero_a1_description/urdf/woan_picker_source.json`。

MoveItConfigsBuilder 组织参数命名空间：

| 配置 | 参数路径 |
|---|---|
| 逆运动学 | `robot_description_kinematics.arm.*` |
| 规划限位 | `robot_description_planning.joint_limits.<关节>.*` |
| OMPL 管线 | `planning_pipelines`、`ompl.*` |
| 控制器映射 | `moveit_simple_controller_manager.*` |

```bash
ros2 param get /move_group robot_description_kinematics.arm.kinematics_solver
ros2 param get /move_group robot_description_planning.joint_limits.joint1-a1_r.max_velocity
ros2 param get /move_group ompl.planning_plugins
```

arm 组使用 KDL，OMPL 默认 RRTConnect，并配置 RRTstar、PRM 等规划器。
规划速度、加速度以 `joint_limits.yaml` 为准。

## 状态监控与服务

状态看门狗监控关节反馈，失联时停用轨迹控制器。单臂可调整的选项见[监控与记录](#监控与记录)。
双臂任一侧失联时停用两侧控制器。话题桥统计 CAN 丢帧和电机故障。

真机提供 `/onero_topic_bridge/enable` 和 `/onero_topic_bridge/disable` 服务。
使能对齐实测位置；失能服务默认拒绝执行。
当前负载见 `/onero_topic_bridge/payload_state`；确认已放下物体后可调用 `/onero_topic_bridge/reset_payload` 恢复启动基线。
断使能会卸除支撑力矩，现场支撑和硬件急停按设备操作流程处理。
软件状态监控不替代驱动器保护和硬件急停。

## 机械臂失能

[tools/arm_disable.py](tools/arm_disable.py) 通过 CAN 显式发送失能指令。
先支撑好机械臂并结束使用相同 CAN 接口的控制服务，再运行此工具；失能会卸除支撑力矩。

在工作区根目录加载环境：

```bash
source setup_env.sh
```

### 单臂失能

按实际设备选择一条：

```bash
# 右臂
python3 tools/arm_disable.py --channel can0 --model A1R

# 左臂
python3 tools/arm_disable.py --channel can1 --model A1L
```

### 双臂失能

一条命令依次向左臂 `can1/A1L`、右臂 `can0/A1R` 发送失能指令：

```bash
python3 tools/arm_disable.py --both
```

CAN 接口不同时显式指定：

```bash
python3 tools/arm_disable.py --both --left-channel slcan1 --right-channel slcan0
```

一侧失败时仍会尝试另一侧，并分别输出结果；任一侧失败时返回非零退出码。

### 失能工具参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `--channel` | `slcan0` | 单臂 CAN 接口 |
| `--model` | `A1R` | 单臂型号，左臂指定 `A1L` |
| `--both` | 不传 | 启用双臂失能 |
| `--left-channel` | `can1` | 双臂模式的左臂 CAN 接口，型号为 A1L |
| `--right-channel` | `can0` | 双臂模式的右臂 CAN 接口，型号为 A1R |
| `--yes` | 不传 | 跳过交互确认 |

`--both` 使用 `--left-channel` / `--right-channel`，不能与 `--channel` / `--model` 混用。

## 功能范围与使用限制

- 支持单臂（左臂或右臂）和双臂；Servo、MTC 和相机感知管线未配置。
- 手臂支持 position + velocity 轨迹接口，夹爪使用 GripperCommand。
- 控制频率见启动参数；双臂控制器与话题桥使用相同频率。
- `kp` / `kd` 支持运行时平滑切换。
- 夹爪未提供经过标定的夹持力接口，也未定义权威指尖 TCP。
- 精度诊断使用关节反馈和 URDF 计算 Link7 位姿；实物末端精度需要独立测量。
- 规划场景的避碰效果取决于模型、安装变换与实际环境的一致性。

## 使用文档

- [功能示例与源码](src/onero_a1_examples/README.md)
- [公共 MoveIt 客户端](src/onero_a1_common/README.md)
- [诊断工具](tools/diagnostics/README.md)
