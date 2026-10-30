# Onero A1 × MoveIt 2

Onero A1 七自由度机械臂与 Woan Picker 的 ROS 2 工作区，支持左/右单臂、双臂运动、避障、抓取放置和键盘遥操作。

[运行示例](src/onero_a1_examples/SHOWCASE.md) · [示例参数](src/onero_a1_examples/README.md#示例参数) · [服务参数](#服务启动参数) · [诊断工具](tools/diagnostics/README.md) · [Python 接口](src/onero_a1_common/README.md)

## 环境准备与构建

Ubuntu 24.04 + ROS 2 Jazzy，每个新终端在工作区根目录执行：

```bash
source setup_env.sh
```

脚本自动安装依赖、构建并加载环境，首次安装可能需要 sudo。ROS 日志位于 `.ros/log/`。
修改配置后重启对应服务；更新代码后重新执行 `source setup_env.sh`。
Python 环境默认使用 `.venv`，自定义解释器通过 `ONERO_RUNTIME_PYTHON` 指定。
依赖及版本见 [requirements-runtime.txt](tools/requirements-runtime.txt) 和 [constraints-runtime.txt](tools/constraints-runtime.txt)。

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

双臂可单独或同步操作。任一侧反馈失联时两臂停止，连接恢复后重启服务。
选臂和运动命令见[双臂示例](src/onero_a1_examples/SHOWCASE.md#双臂运行指令)。

## 功能示例

启动服务后，在另一终端运行基础运动、夹爪开合、抓取放置或键盘遥操作示例。

- [示例功能与参数](src/onero_a1_examples/README.md)：选择示例、查看参数和使用条件。

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
| `enable_servo` | `false` | 开启键盘遥操作支持 | 单/双臂 |
| `servo_debug` | `false` | 记录 Servo 指令、输出轨迹和反馈 | 单/双臂 |
| `servo_debug_file` | 自动生成 | 运动记录路径，默认在 `.ros/log/servo_debug/` | 单/双臂 |

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
| `kp` | `50` | 位置增益（保持力度）`[0, 500]` | 两者 |
| `kd` | `1.0` | 速度阻尼 `[0, 5]` | 两者 |
| `control_rate` | sim `200`，real `100` Hz | 控制更新频率 | 两者 |
| `gravity_compensation` | sim `false`，real `true` | 真机重力补偿 | 两者 |
| `payload_mass` | `0.0` kg | 额外负载质量，非负 | 单臂 |
| `payload_cog` | `[0.0, 0.0, 0.0]` m | 基线重心，SDK 末端坐标系 | 单臂 |
| `left_payload_mass` / `right_payload_mass` | `0.0` kg | 各臂额外负载质量，非负 | 双臂 |
| `left_payload_cog` / `right_payload_cog` | `[0.0, 0.0, 0.0]` m | 各臂基线重心 | 双臂 |
| `payload_transition_time` | `0.5` s | 运行时负载切换时长，正数 | 两者 |
| `velocity_feedforward` | sim `false`，real `true` | 七关节目标速度前馈 | 两者 |
| `velocity_feedforward_timeout` | `0.1` s | 命令过期后速度前馈归零 | 单臂 |

`kp` / `kd` 支持运行时调整，0.5 s 内平滑生效；sim 不使用电机增益及重力补偿。
双臂公共启动参数作用于两侧，运行时可分别调整。

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
| `with_gripper` | `true` | 是否按原装夹爪进行重力补偿 | 两者 |
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
| `trace_control` | `false` | 记录详细运动数据，供故障排查 |

### 高级设置

| 参数 | 默认值 | 说明 | 入口 |
|---|---|---|---|
| `runtime_python` | `ONERO_RUNTIME_PYTHON`，通常为 `.venv/bin/python` | 设备控制使用的 Python 解释器 | 两者 |
| `rviz_config` | `moveit.rviz` | 本包 `rviz/` 下的布局文件 | 单臂 |
| `publish_world_tf` | `true` | 发布 `world → base_link`；关闭后须自行提供 TF | 单臂 |
| `joint_commands_topic` | `/robot_joint_commands` | 运动指令话题 | 单臂 |
| `joint_states_topic` | `/robot_joint_states` | 关节反馈话题 | 单臂 |

## 规划器选择

默认 OMPL / RRTConnect，也支持 CHOMP、STOMP。适用示例和选项见[规划器选择](src/onero_a1_examples/README.md#规划器选择)。

## 桌面与支架环境

`environment_file` 指定桌面、支架等障碍物的位置和尺寸。程序不会自动识别现场物体，使用前须核对配置。
单臂使用 [environment.desktop.yaml](src/onero_a1_moveit_config/config/environment.desktop.yaml)，
双臂使用 [environment.dual_desktop.yaml](src/onero_a1_moveit_config/config/environment.dual_desktop.yaml)。

- 核对尺寸、位置、`mounting`；双臂还须匹配 `arm_spacing`。
- `frame_id` 为 world，`xyz` 为物体中心，`size` 为尺寸（m），`rpy` 为 rad，`color` 为 RGBA。
- 默认环境中根部距深色底板顶面 `0.330` m；底板顶面 Z=`-0.330`，木桌顶面 Z=`-0.340` m，其余尺寸见配置。
- world 的 X 向前、Y 向左、Z 向上；双臂原点在根部中心的中点，默认左/右根部 Y=`+0.035` / `-0.035` m。

环境配置和起始姿态检查通过后才允许执行，起点碰撞时须先调整现场或配置。
标记 `example_only: true` 的环境只能配合 `allow_execution:=false` 预览。
加载后可在 RViz 中查看和调整碰撞体。
配置未包含的物品不会参与避障；现场新增障碍物时需补充配置。

## Woan Picker

`gripper_model:=woan_picker` 启用夹爪显示、碰撞检查和开合控制；`none` 表示无夹爪模型。

通过[夹爪示例](src/onero_a1_examples/README.md#夹爪开合)设置开度：0 全闭、1 全开。启动服务时夹爪不会自动开合。

空载且开合范围无遮挡时标定：

```bash
ros2 service call /gripper_controller/calibrate std_srvs/srv/Trigger '{}'
```

需要中止时，在另一终端调用 stop：

```bash
ros2 service call /gripper_controller/stop std_srvs/srv/Trigger '{}'
```

标定结果自动保存到 `.ros/gripper_calibration/`。标定期间不能开合；失败时保留原设置，更换设备或开合端点后重新标定。
**关闭调用终端不会停止标定，需要使用上面的 stop 命令。**

实物未装原装夹爪时设置 `with_gripper:=false`。更换工具后须核对负载和 TCP，加载外形模型不会自动校准。
开合命令仅设置目标开度；抓取确认请使用抓放任务示例。

## 模型与参数配置

模型须与实物的型号、安装方向、夹爪和关节零位一致。常用配置如下，修改后重启服务。

| 配置 | 用途 |
|---|---|
| [kinematics.yaml](src/onero_a1_moveit_config/config/kinematics.yaml) | 逆运动学求解设置 |
| [posture.yaml](src/onero_a1_kinematics/config/posture.yaml) | 肘部和手腕的自然姿态偏好 |
| [joint_limits.yaml](src/onero_a1_moveit_config/config/joint_limits.yaml) | 关节速度和加速度上限 |
| [ros2_controllers.yaml](src/onero_a1_bringup/config/ros2_controllers.yaml) | 运动跟踪和终点容差 |
| [servo.yaml](src/onero_a1_moveit_config/config/servo.yaml) | 遥操作速度与运动保护设置 |

## 状态监控与服务

反馈失联时自动停止运动，日志显示通信或电机故障；连接恢复后重启服务。需要排查时使用[诊断工具](tools/diagnostics/README.md)。
`/onero_topic_bridge/payload_state` 可查看负载设置；放下物体后用 `/onero_topic_bridge/reset_payload` 恢复启动值。
失能会卸除支撑力矩，请按下方步骤操作。

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

左右依次失能，终端分别显示结果；请确认两侧均成功。

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

- Servo 键盘支持单/双臂，Tab 切换控制手臂，见[运行命令](src/onero_a1_examples/SHOWCASE.md#4-servo-键盘遥操作)。
- 场景避碰依赖模型、安装及实物一致。
- 夹持力和指尖位置需要实物标定；软件显示的位置不能代替实物精度测量。
