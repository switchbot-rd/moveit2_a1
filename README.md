# Onero A1 × MoveIt 2

Onero A1 七自由度机械臂与 Woan Picker 的 ROS 2 工作区，支持左/右单臂、双臂规划、执行、避障和夹爪控制。
控制链路：MoveIt → ros2_control → onero-sdk → CAN。

[运行示例](src/onero_a1_examples/SHOWCASE.md) · [示例参数](src/onero_a1_examples/README.md#示例参数) · [服务参数](#服务启动参数) · [诊断工具](tools/diagnostics/README.md)

## 项目结构

| 位置 | 内容 |
|---|---|
| `src/onero_a1_description` | URDF、网格、硬件描述 |
| `src/onero_a1_bringup` | SDK 话题桥、控制器、看门狗、环境加载 |
| `src/onero_a1_moveit_config` | 运动学、限位、规划器、控制器映射及 launch |
| `src/onero_a1_common` | 公共 MoveIt Python 客户端 |
| `src/onero_a1_examples` | 功能示例 |
| `tools/diagnostics` | 采集、姿态存档、轨迹回放、精度及 SDK 对照 |

## 环境准备与构建

Ubuntu 24.04 + ROS 2 Jazzy，每个新终端在工作区根目录执行：

```bash
source setup_env.sh
```

脚本安装缺失依赖、准备 `.venv`、增量构建并加载环境；源码未变时跳过构建，多终端并发时自动排队。
apt 安装需要系统权限，ROS 日志位于 `.ros/log/`，`overlay/` 提供本地依赖覆盖。
已加载环境时，修改现有 Python/YAML 后重新启动对应程序即可；新增入口或依赖需重新构建。

运行环境使用 [`onero-sdk[dynamics]==0.3.0`](https://pypi.org/project/onero-sdk/0.3.0/)，`.venv` 可访问系统 ROS Python 包。
话题桥使用 `.venv/bin/python`；其他解释器通过 `ONERO_RUNTIME_PYTHON` 指定。
依赖声明见 [requirements-runtime.txt](tools/requirements-runtime.txt)，版本约束见 [constraints-runtime.txt](tools/constraints-runtime.txt)。

## 启动服务

```bash
# 单臂模拟，默认右臂
ros2 launch onero_a1_moveit_config moveit.launch.py hardware_mode:=sim

# 单臂真机：加载环境，仅预览轨迹
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real allow_execution:=false \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

左臂添加 `arm:=left`，无界面添加 `rviz:=false`，允许执行时设置 `allow_execution:=true`。
**`allow_execution:=false` 仍会使能真机并保持实测姿态。** 切换模式前结束旧服务，各终端使用同一 ROS 域。
`sim` 只模拟关节跟随和夹爪开合，不连接 CAN，也不模拟重力、摩擦或接触。

## 双臂控制

```bash
ros2 launch onero_a1_moveit_config dual_moveit.launch.py hardware_mode:=sim

ros2 launch onero_a1_moveit_config dual_moveit.launch.py \
  hardware_mode:=real \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.dual_desktop.yaml"
```

双臂共用规划场景，各自使用 CAN、控制器和夹爪接口。任一侧反馈失联时停用两侧轨迹控制器，恢复后重启服务。
选臂和运动命令见[双臂示例](src/onero_a1_examples/SHOWCASE.md#双臂运行指令)。

## 服务启动参数

格式：`ros2 launch onero_a1_moveit_config <启动文件> 参数:=值`。布尔值使用 `true` / `false`。
下表“单臂”指 `moveit.launch.py`，“双臂”指 `dual_moveit.launch.py`。

### 基础启动

| 参数 | 默认值 | 说明 | 入口 |
|---|---|---|---|
| `hardware_mode` | `sim` | `sim` / `real` | 两者 |
| `rviz` | `true` | 打开 RViz | 两者 |
| `allow_execution` | `true` | 允许手臂、夹爪执行；关闭后真机仍使能保持 | 两者 |
| `environment_file` | 空 | 碰撞环境 YAML 绝对路径；空则不加载 | 两者 |

### 单臂安装

| 参数 | 默认值 | 说明 |
|---|---|---|
| `arm` | `right` | `left` / `right` |
| `model` | 左 `A1L`，右 `A1R` | 必须与选臂匹配 |
| `can_channel` | 左 `can1`，右 `can0` | 实际 CAN 接口，如 `slcan0` |
| `mounting` | 左 `humanoid_left`，右 `humanoid_right` | 坐标映射和重力补偿的安装方向 |

显式填写 `mounting` 时须随选臂修改；省略时自动选择，单臂桌面环境随左臂镜像加载。
其他安装方向：`upright`、`a1_vertical`、`side_mount_left`、`side_mount_right`、`ceiling`。

### 双臂安装

| 参数 | 默认值 | 说明 |
|---|---|---|
| `left_can_channel` | `can1` | 左臂 CAN 接口 |
| `right_can_channel` | `can0` | 右臂 CAN 接口，须与左臂不同 |
| `arm_spacing` | `0.07` m | 两根部中心间距，须与环境文件一致 |

### 运动控制参数

| 参数 | 默认值 | 说明 | 入口 |
|---|---|---|---|
| `kp` | `50` | 位置增益 `[0, 500]` | 两者 |
| `kd` | `1.0` | 速度阻尼 `[0, 5]` | 两者 |
| `control_rate` | sim `200`，real `100` Hz | 控制器、话题桥同频 | 两者 |
| `gravity_compensation` | sim `false`，real `true` | 真机重力补偿 | 两者 |
| `payload_mass` | `0.0` kg | 空夹爪启动基线的额外负载，非负 | 单臂 |
| `payload_cog` | `[0.0, 0.0, 0.0]` m | 基线重心，SDK 末端坐标系 | 单臂 |
| `left_payload_mass` / `right_payload_mass` | `0.0` kg | 各臂基线额外负载，非负 | 双臂 |
| `left_payload_cog` / `right_payload_cog` | `[0.0, 0.0, 0.0]` m | 各臂基线重心 | 双臂 |
| `payload_transition_time` | `0.5` s | 运行时负载切换时长，正数 | 两者 |
| `velocity_feedforward` | sim `false`，real `true` | 七关节目标速度前馈 | 两者 |
| `velocity_feedforward_timeout` | `0.1` s | 命令过期后速度前馈归零 | 单臂 |

`kp` / `kd` 支持运行时平滑切换；sim 不使用电机增益及重力补偿。
双臂公共启动参数作用于两侧，运行时通过各臂桥接节点分别调整。

### 机械臂执行容差

以下均适用单/双臂。省略时沿用 [ros2_controllers.yaml](src/onero_a1_bringup/config/ros2_controllers.yaml)，显式设置时统一覆盖各关节。

| 参数 | YAML 默认值 | 含义 |
|---|---|---|
| `arm_trajectory_tolerance` | `0.20` rad | 运动中关节位置跟踪误差 |
| `arm_goal_tolerance` | `0.1` rad | 终点关节位置误差 |
| `arm_goal_time_tolerance` | `1.0` s | 终点额外等待时间 |
| `arm_stopped_velocity_tolerance` | `0.02` rad/s | 终点速度 |

容差须为正数，影响 RViz 和示例的成功判定；放宽容差不会提高实物精度。

### 夹爪参数

| 参数 | 默认值 | 说明 | 入口 |
|---|---|---|---|
| `gripper_model` | `woan_picker` | `woan_picker` / `none` | 两者 |
| `with_gripper` | `true` | SDK 原装夹爪重力补偿档位 | 两者 |
| `gripper_calibration_file` | 自动按型号、CAN 选择 | 文件路径；`none` 禁用 | 单臂 |
| `gripper_closed_sdk_opening` | `0.0` | 全闭原生开度，`[0, 1]` | 单臂 |
| `gripper_open_sdk_opening` | `1.0` | 全开原生开度，`[0, 1]`，须大于全闭值 | 单臂 |

标定文件优先于手工端点；双臂分别自动加载各自标定。

### 监控与记录

以下仅适用单臂。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `watchdog_timeout` | `1.0` s | 反馈失联时间 |
| `watchdog_auto_deactivate` | `true` | 失联时停用轨迹控制器 |
| `trace_control` | `false` | 发布控制链路诊断数据 |

### 高级设置

| 参数 | 默认值 | 说明 | 入口 |
|---|---|---|---|
| `runtime_python` | `ONERO_RUNTIME_PYTHON`，通常为 `.venv/bin/python` | 话题桥解释器；单臂还用其查询安装方向 | 两者 |
| `rviz_config` | `moveit.rviz` | 本包 `rviz/` 下的布局文件 | 单臂 |
| `publish_world_tf` | `true` | 发布 `world → base_link`；关闭后须自行提供 TF | 单臂 |
| `joint_commands_topic` | `/robot_joint_commands` | 控制器发往话题桥的命令 | 单臂 |
| `joint_states_topic` | `/robot_joint_states` | 话题桥发往控制器的反馈 | 单臂 |

## 规划器选择

默认 OMPL / RRTConnect，也支持 CHOMP、STOMP。适用示例和选项见[规划器选择](src/onero_a1_examples/README.md#规划器选择)。

## 功能示例与公共接口

[示例与参数](src/onero_a1_examples/README.md) · [运行命令](src/onero_a1_examples/SHOWCASE.md) · [公共客户端](src/onero_a1_common/README.md)

## 控制链路

```text
MoveIt ExecuteTrajectory → arm_controller/FollowJointTrajectory → ros2_control
  → /robot_joint_commands → onero_topic_bridge → onero-sdk Arm.servoj() → CAN
反馈 → /robot_joint_states → joint_state_broadcaster → /joint_states → MoveIt
```

位置与速度按关节名同步传递。速度前馈关闭、命令缺失/非法/过期时，期望速度归零；取消、保持及重新使能会清除速度缓存。
`kp` / `kd` 为 SDK 全关节增益，运行时在 0.5 s 内平滑切换，以 `gain_state` 确认生效。
重力补偿按实测姿态、安装和负载计算 `tau_ff`，受 SDK、适配层限幅；动力学依赖及模型在打开 CAN 前检查。

## 桌面与支架环境

`environment_file` 将 YAML 碰撞体加载到 Planning Scene；默认不加载环境，也不自动识别实物。
单臂使用 [environment.desktop.yaml](src/onero_a1_moveit_config/config/environment.desktop.yaml)，
双臂使用 [environment.dual_desktop.yaml](src/onero_a1_moveit_config/config/environment.dual_desktop.yaml)。

- 核对尺寸、位置、`mounting`；双臂还须匹配 `arm_spacing`。
- `frame_id` 为 world，`xyz` 为物体中心，`size` 为尺寸（m），`rpy` 为 rad，`color` 为 RGBA。
- 当前根部距深色底板顶面 `0.330` m；底板顶面 Z=`-0.330`，木桌顶面 Z=`-0.340` m，其余尺寸见配置。
- world 的 X 向前、Y 向左、Z 向上；双臂原点在根部中心的中点，默认左/右根部 Y=`+0.035` / `-0.035` m。

加载顺序：配置检查 → 等待反馈 → 加载并回读场景 → 起点碰撞检查 → 启动轨迹控制器。
起点碰撞不会自动放行；真机执行拒绝 `example_only: true`，仅预览可用 `allow_execution:=false`。
环境对象在加载器退出后保留，手动修改后以场景数据为准。
环境文件不校准运动学、零位或负载；其他机械臂、线缆及桌面物品需另行建模。

## Woan Picker

`gripper_model:=woan_picker` 加载外形、碰撞体、mimic 机构、规划组及开合接口；`none` 表示无夹爪模型。

| 接口 / 字段 | 说明 |
|---|---|
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| `command.position` | 主关节角度：0 全闭，1.1 rad 全开 |
| `command.max_effort` | 仅接受 0，使用 SDK 位置控制 |
| `/gripper_controller/state` | 开度、反馈、标定和指令状态 |
| `/gripper_controller/calibrate` | 自动全开/全闭并保存端点 |
| `/gripper_controller/stop` | 停止动作或标定，保持反馈开度 |

夹爪与手臂共用 CAN transport 和 I/O 锁；启动只查询开度，收到动作目标才使能。
主关节发布到 `/joint_states`，其余关节通过 mimic 联动。

空载且开合范围无遮挡时标定：

```bash
ros2 service call /gripper_controller/calibrate std_srvs/srv/Trigger '{}'
```

需要中止时，在另一终端调用 stop：

```bash
ros2 service call /gripper_controller/stop std_srvs/srv/Trigger '{}'
```

标定保存原生指令和反馈端点：全开/全闭使用驱动端点，中间开度按反馈行程换算。
标定期间拒绝普通目标；故障、异常反馈、超时或停止会中断并保留有效配置。
**取消调用客户端不会停止服务端标定，需调用 stop。**
文件如 `.ros/gripper_calibration/A1R_can0_woan_picker.json`，按型号、CAN 和 sim/real 校验；更换设备或端点后重新标定。

视觉模型不会自动修改 SDK payload 或 TCP。实物未装原装夹爪时设置 `with_gripper:=false`。
基础开合接口不判断物体是否夹稳，抓取判定由任务层完成。

## 模型与参数配置

URDF、SRDF 和网格随项目提供，来源为 SDK A1L/A1R、Woan Picker，保留几何、惯性、安装变换和限位。
本体只排除相邻连杆碰撞，夹爪另含机构连接/闭合接触对；来源和校验值见
[woan_picker_source.json](src/onero_a1_description/urdf/woan_picker_source.json)。

| 配置 | MoveIt 参数路径 |
|---|---|
| KDL 逆运动学 | `robot_description_kinematics.arm.*` |
| `joint_limits.yaml` | `robot_description_planning.joint_limits.<关节>.*` |
| 规划管线 | `planning_pipelines`、`ompl.*` |
| 控制器映射 | `moveit_simple_controller_manager.*` |

```bash
ros2 param get /move_group robot_description_kinematics.arm.kinematics_solver
ros2 param get /move_group robot_description_planning.joint_limits.joint1-a1_r.max_velocity
ros2 param get /move_group ompl.planning_plugins
```

## 状态监控与服务

看门狗在反馈失联时停用轨迹控制器，话题桥统计 CAN 丢帧和电机故障。
真机服务 `/onero_topic_bridge/enable` 对齐实测位置后使能；`/onero_topic_bridge/disable` 默认拒绝执行。
负载见 `/onero_topic_bridge/payload_state`；确认已放下物体后，用 `/onero_topic_bridge/reset_payload` 恢复启动基线。
失能会卸除支撑力矩，软件监控不替代驱动器保护及硬件急停。

## 机械臂失能

使用 [arm_disable.py](tools/arm_disable.py) 前，先支撑机械臂并结束占用同一 CAN 的服务。
以下命令在已加载环境的工作区根目录运行。

### 单臂失能

按实际设备选择一条：

```bash
python3 tools/arm_disable.py --channel can0 --model A1R  # 右臂
python3 tools/arm_disable.py --channel can1 --model A1L  # 左臂
```

### 双臂失能

```bash
python3 tools/arm_disable.py --both
# 自定义 CAN
python3 tools/arm_disable.py --both --left-channel slcan1 --right-channel slcan0
```

左右依次执行，一侧失败仍尝试另一侧；任一侧失败则返回非零。

### 失能工具参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--channel` | `slcan0` | 单臂 CAN |
| `--model` | `A1R` | 单臂型号，左臂用 `A1L` |
| `--both` | 关闭 | 双臂模式，与 `--channel` / `--model` 互斥 |
| `--left-channel` | `can1` | 双臂左侧 A1L |
| `--right-channel` | `can0` | 双臂右侧 A1R |
| `--yes` | 关闭 | 跳过交互确认 |

## 功能范围与使用限制

- 未配置 Servo、MTC 和相机感知；场景避碰依赖模型、安装及实物一致。
- 手臂使用 position + velocity 轨迹接口，夹爪使用 GripperCommand。
- 夹持力和指尖 TCP 未标定；FK 精度来自关节反馈与 URDF，实物精度须独立测量。

## 使用文档

[功能示例](src/onero_a1_examples/README.md) · [公共客户端](src/onero_a1_common/README.md) · [诊断工具](tools/diagnostics/README.md)
