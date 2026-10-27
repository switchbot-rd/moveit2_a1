# Onero A1 MoveIt 2 Examples

支持左/右单臂、双臂和 Woan Picker。运动默认只规划，添加 `--execute` 执行。

[运行命令](SHOWCASE.md) · [示例参数](#示例参数) · [启动与构建](../../README.md#环境准备与构建) · [服务参数](../../README.md#服务启动参数)

## 代码结构

| 示例（源码） | 功能 |
|---|---|
| [joint_goal](onero_a1_examples/joint_goal.py) | 相对关节目标 |
| [named_pose](onero_a1_examples/named_pose.py) | SRDF 命名姿态 `ready` / `home` |
| [pose_goal](onero_a1_examples/pose_goal.py) | 法兰位置和朝向 |
| [cartesian_path](onero_a1_examples/cartesian_path.py) | 笛卡尔直线、平面正方形 |
| [orientation_constraint](onero_a1_examples/orientation_constraint.py) | 运动中保持朝向 |
| [planning_scene](onero_a1_examples/planning_scene.py) | 临时障碍物与避让 |
| [collision_check](onero_a1_examples/collision_check.py) | 状态有效性与碰撞对象 |
| [gripper](onero_a1_examples/gripper.py) | 夹爪开合 |
| [arm_gripper_task](onero_a1_examples/arm_gripper_task.py) | 取瓶、回起点停顿、放回 |
| [fixed_pick_place](onero_a1_examples/fixed_pick_place.py) | 固定 A/B 点往返抓放 |
| [record_startup](onero_a1_examples/record_startup.py) | 记录起始位置，不运动 |
| [return_startup](onero_a1_examples/return_startup.py) | 返回已记录的起始位置 |
| [freedrive](onero_a1_examples/freedrive.py) | 重力补偿手动拖动 |
| [dual_joint_goal](onero_a1_examples/dual_joint_goal.py) | 十四关节联合规划 |
| [dual_cartesian_square](onero_a1_examples/dual_cartesian_square.py) | 双臂同时画正方形 |
| [dual_gripper](onero_a1_examples/dual_gripper.py) | 双夹爪协调开合 |

[公共客户端](../onero_a1_common/README.md) 封装 ROS 通信；[support.py](onero_a1_examples/support.py) 管理命令行和生命周期，
[task_support.py](onero_a1_examples/task_support.py) 共享抓取反馈及轨迹检查。

## 示例运行指令

完整命令见 [SHOWCASE.md](SHOWCASE.md)：[单臂](SHOWCASE.md#单臂运行指令) / [双臂](SHOWCASE.md#双臂运行指令)。

## 起始位置与返回

静止时运行 `record_startup` 保存当前位置，`return_startup` 规划返回；启动服务和普通运动不会覆盖记录。
记录按手臂、sim/real、型号、CAN、安装及零位配置区分，相同配置下可跨服务重启沿用；再次记录只覆盖所选臂。
文件：`$ROS_HOME/startup_poses/recorded_startup.json`，工作区默认在 `.ros/` 下；无匹配记录时须先记录。

## 双臂控制

双臂共用场景和 world 坐标系，原点在两根部中心的中点。

| 示例 | `--arm` |
|---|---|
| `named_pose`、`record_startup`、`return_startup`、`collision_check`、`freedrive` | `left` / `right` / `both` |
| `dual_joint_goal`、`dual_cartesian_square`、`dual_gripper` | 自动双臂，不传此参数 |
| 其余示例 | `left` / `right`；双臂服务默认 `right`，单臂自动识别 |

## 规划器选择

默认 OMPL / RRTConnect；仅下列示例接受 `--pipeline` / `--planner`。

| 示例 | 可用管线 |
|---|---|
| `joint_goal`、`dual_joint_goal`、`named_pose`、`return_startup`、`planning_scene` | OMPL、CHOMP、STOMP |
| `pose_goal` | OMPL、STOMP |
| `orientation_constraint`、`gripper`、`dual_gripper` | OMPL |

夹爪固定使用 OMPL / RRTConnect。`cartesian_path`、`dual_cartesian_square` 和联合任务的手臂使用笛卡尔接口。
`fixed_pick_place` 的预抓取、待机回位使用 OMPL，其余手臂运动使用笛卡尔接口。
查询、记录和手动拖动不选择规划器。

## 示例参数

格式：`--参数 值`；`--execute`、`--help` 为开关。服务启动参数使用 `参数:=值`。
下表位移为 m、角度为 rad，数值须有限；完整命令见 [SHOWCASE.md](SHOWCASE.md)。

### 通用设置

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--arm` | 单臂自动识别，双臂 `right` | 允许值见[双臂控制](#双臂控制) |
| `--frame` | `world` | `world` / `base_link`；双臂时后者为所选臂根部，`both` 为公共根部 |
| `--execute` | 关闭 | 执行运动；`freedrive` 进入拖动。查询、记录不接受 |
| `--help` | — | 参数帮助 |

两种抓放任务固定使用 `--frame world`。

### 速度与规划

记录、碰撞查询和拖动不使用本节参数。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--velocity` / `--acceleration` | `0.05` | 缩放 `(0, 1]` |
| `--planning-time` | `10` s | 规划预算，正数；笛卡尔接口不使用该搜索预算 |
| `--pipeline` | `ompl` | `ompl` / `chomp` / `stomp`，适用范围见上表 |
| `--planner` | `RRTConnect` | 仅用于 OMPL；CHOMP/STOMP 省略 |

OMPL 算法：`RRTConnect`、`RRTstar`、`PRM`、`EST`、`LBKPIECE`、`BiTRRT`、`AnytimePathShortening`。
夹爪缩放作用于规划轨迹，实物开合由适配层开度斜坡执行。

### 关节与命名姿态

| 参数 | 默认值 | 说明 / 示例 |
|---|---|---|
| `--joint` | 所选臂第 1 关节 | `joint1-a1_l`…`joint7-a1_l` 或 `joint1-a1_r`…`joint7-a1_r`；`joint_goal` |
| `--delta` | `0.1` | 相对关节角；`joint_goal` |
| `--target` | `named_pose`：`ready`；`collision_check`：当前 | SRDF 姿态名，如 `home` / `ready` |

`return_startup` 使用已记录的起始位置，不另设目标。

### 末端位姿

目标为 Link7 法兰，平移沿参考系轴、旋转绕参考系轴。

| 参数 | 默认值 | 示例 |
|---|---|---|
| `--dx` / `--dy` / `--dz` | `0` / `0` / `0.01` | `pose_goal`、`cartesian_path`、`orientation_constraint` |
| `--roll` / `--pitch` / `--yaw` | 均 `0` | `pose_goal`；仅旋转时须将三个平移量设为 `0` |

### 笛卡尔路径与朝向约束

| 参数 | 默认值 | 说明 / 示例 |
|---|---|---|
| `--shape` | `line` | `line` / `square`；`cartesian_path` |
| `--size` | 单臂 `0.02`，双臂 `0.05` | 正方形边长，正数；`cartesian_path`、`dual_cartesian_square` |
| `--plane` | `yz` | `xy` / `xz` / `yz`，基于 `--frame`；单臂正方形 |
| `--path-angle-tolerance` | `0.1` | 各轴朝向偏差，正数；`orientation_constraint` |

### 场景避障

`planning_scene`：`--scene-delta` 默认 `0.4` rad，为 joint4 相对变化；`--obstacle-radius` 默认 `0.01` m，须为正数。

### 夹爪开合

开度 `[0, 1]`：0 全闭、0.5 半开、1 全开。
`gripper --opening` 默认 `1`；`dual_gripper --left-opening` / `--right-opening` 均默认 `1`。

### 手臂与夹爪联合任务

`arm_gripper_task`：桌面取瓶 → 保持朝向回到本次起点 → 停顿 → 放回松爪。
需先预览确定瓶子坐标，再按坐标摆放实物；真机确认受阻开度后才搬运，RViz 模型不产生接触反馈。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--diameter` / `--bottle-height` | `0.05` / `0.18` m | 瓶身尺寸，另加 2 cm 瓶颈 |
| `--reach` | `0.12` m | 自动选点的前伸距离 |
| `--lift` | `0.03` m | 夹取后抬升 |
| `--pause` | `2` s | 回起点后的停顿 |
| `--open-opening` | `1` | 张开开度 `[0, 1]` |
| `--grasp-opening` | 按瓶径计算 | 预期接触开度，可用实测值 |
| `--tcp-offset X Y Z` | 按模型计算 | 夹持中心相对 Link7 的偏移，m |
| `--bottle-position X Y Z` | 自动选点 | 瓶身中心，m，瓶底须落在桌面 |
| `--support-height` | 读取桌面 | 核对支撑面高度，m |
| `--orientation-tolerance` | `0.1` rad | 每段起点跟踪偏差，目标朝向保持不变 |

持瓶去程、回程分别连续规划，转角平滑，只在起点停顿；结束输出模型 FK 回位高度差。
预览记录：`$ROS_HOME/tasks/arm_gripper_task_<left或right>.json`。

### 固定位置抓取放置

`fixed_pick_place` 默认 A→B 放下退开 → 重新夹取 B→A 放回退开；可独立运行。
预设水平距离约 26.5 cm、搬运时瓶底离桌面约 20 cm。法兰可绕 world Z 转动，接近方向保持配置值，物体保持基本直立。

| 参数 | 说明 |
|---|---|
| `--config PATH` | 可选，[配置预设及注释](config/fixed_pick_place.yaml) |
| `--record-orientation` | 记录侧夹参考朝向，搭配 `--config` |
| `--one-way` | 仅 A→B |

执行重新规划，速度和加速度可与预览不同；返程使用 B 点放置反馈重新规划并再次确认夹取。
默认放置位置误差门限为 15 mm，可在配置中调整。

### 双臂动作

`dual_joint_goal`：`--joint-index` 默认 `1`（范围 1…7），`--left-delta` / `--right-delta` 默认 `-0.2` / `0.2` rad。
`dual_cartesian_square`：两侧同时在 YZ 平面向上、向外、向下、返回，保持朝向；对应角点停顿，全路径通过碰撞检查后才执行。

### 零重力拖动

`freedrive --damping` 默认 `0.3`，范围 `[0, 5]`，越大越难拖动；`--duration` 为正数秒，默认不限时。
仅 real 模式，须开启重力补偿、静止且已使能；`--execute` 以零位置刚度进入拖动并暂停轨迹控制器。
Ctrl+C 后保持当前位置、恢复控制器；拖动不更新起始位置，也不经过 MoveIt 避障规划。
重力参数有偏差时可能漂移；心跳中断超过 2 s 自动保持，不自动失能，强制退出后可重启服务恢复控制器。

## 运行结果

成功输出轨迹点数、时长和执行结果；路径不完整或检查失败时返回非零。
所有示例在成功、失败或中断后清理临时模型及碰撞允许项，包括附着模型。
抓放任务夹取后使用 `kp=120`（固定任务可配置），松爪后恢复原值；持物中止时保持夹持和当前增益，不自动松爪。
固定任务不设置负载补偿。精度报告、回放和对照见[诊断工具](../../tools/diagnostics/README.md)。

## world 目标如何变成运动

MoveIt 结合 `world → base_link` TF、URDF 和逆运动学，将法兰目标转换为关节轨迹，经控制器和 SDK 执行。
当前 `humanoid_right` 中 world +Z 对应 base_link +X；更换参考系不会校准安装、模型或关节零位。
