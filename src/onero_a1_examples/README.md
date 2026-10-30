# Onero A1 MoveIt 2 Examples

支持左/右单臂、双臂和 Woan Picker。默认仅规划或检查条件，添加 `--execute` 执行。

[运行命令](SHOWCASE.md) · [示例参数](#示例参数) · [启动与构建](../../README.md#环境准备与构建) · [服务参数](../../README.md#服务启动参数)

## 功能一览

| 示例 | 功能 |
|---|---|
| [joint_goal](onero_a1_examples/joint_goal.py) | 相对关节目标 |
| [named_pose](onero_a1_examples/named_pose.py) | 预设姿态 `ready` / `home` |
| [pose_goal](onero_a1_examples/pose_goal.py) | 法兰位置和朝向 |
| [cartesian_path](onero_a1_examples/cartesian_path.py) | 笛卡尔直线、平面正方形 |
| [orientation_constraint](onero_a1_examples/orientation_constraint.py) | 运动中保持朝向 |
| [planning_scene](onero_a1_examples/planning_scene.py) | 临时障碍物与避让 |
| [collision_check](onero_a1_examples/collision_check.py) | 检查姿态是否可用、是否碰撞 |
| [gripper](onero_a1_examples/gripper.py) | 夹爪开合 |
| [arm_gripper_task](onero_a1_examples/arm_gripper_task.py) | 双臂镜像取瓶、同步搬运并放回 |
| [fixed_pick_place](onero_a1_examples/fixed_pick_place.py) | 单瓶 A→B→C→B→A 双臂接力 |
| [record_startup](onero_a1_examples/record_startup.py) | 记录起始位置，不运动 |
| [return_startup](onero_a1_examples/return_startup.py) | 返回已记录的起始位置 |
| [freedrive](onero_a1_examples/freedrive.py) | 重力补偿手动拖动 |
| [servo_keyboard](onero_a1_examples/servo_keyboard.py) | 键盘平移、旋转与夹爪抓放 |
| [dual_joint_goal](onero_a1_examples/dual_joint_goal.py) | 双臂同步关节运动 |
| [dual_cartesian_square](onero_a1_examples/dual_cartesian_square.py) | 双臂同时画正方形 |
| [dual_gripper](onero_a1_examples/dual_gripper.py) | 双夹爪协调开合 |

## 示例运行指令

完整命令见 [SHOWCASE.md](SHOWCASE.md)：[单臂](SHOWCASE.md#单臂运行指令) / [双臂](SHOWCASE.md#双臂运行指令)。

## 起始位置与返回

静止时运行 `record_startup` 保存当前位置，用 `return_startup` 返回。连续试用多个基础运动示例时，每项完成后先返回起始位置。相同设备配置下，重启服务后仍可使用；更换设备或安装配置后重新记录。
文件：`$ROS_HOME/startup_poses/recorded_startup.json`，工作区默认在 `.ros/` 下；无匹配记录时须先记录。

## 双臂控制

双臂共用场景和 world 坐标系，原点在两根部中心的中点。

| 示例 | `--arm` |
|---|---|
| `named_pose`、`record_startup`、`return_startup`、`collision_check`、`freedrive` | `left` / `right` / `both` |
| `dual_joint_goal`、`dual_cartesian_square`、`dual_gripper`、`arm_gripper_task`、`fixed_pick_place` | 自动双臂，不传此参数 |
| `servo_keyboard` | `left` / `right` / `both`；双臂窗口用 Tab 切换 |
| 其余示例 | `left` / `right`；双臂服务默认 `right`，单臂自动识别 |

## 规划器选择

默认 OMPL / RRTConnect；仅下列示例接受 `--pipeline` / `--planner`。

| 示例 | 可用管线 |
|---|---|
| `joint_goal`、`dual_joint_goal`、`named_pose`、`return_startup`、`planning_scene` | OMPL、CHOMP、STOMP |
| `pose_goal` | OMPL、STOMP |
| `orientation_constraint`、`gripper`、`dual_gripper` | OMPL |

其余示例无需选择规划器。

### 自然构型优化

运动时尽量保持肘部向外、手腕不过度反折，优先保证末端到位和避障；直接指定关节角时按指定值到位。
默认已启用，如需调整可修改[姿态配置](../onero_a1_kinematics/config/posture.yaml)，然后重启服务。

## 示例参数

格式：`--参数 值`；`--execute`、`--help` 为开关。服务启动参数使用 `参数:=值`。
下表位移单位为 m、角度单位为 rad；完整命令见 [SHOWCASE.md](SHOWCASE.md)。

### 通用设置

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--arm` | 单臂自动识别，双臂 `right` | 允许值见[双臂控制](#双臂控制) |
| `--frame` | `world` | `world` / `base_link`；双臂时后者为所选臂根部，`both` 为公共根部 |
| `--execute` | 关闭 | 执行运动；`freedrive` 进入拖动。查询、记录不接受 |
| `--help` | — | 参数帮助 |

两种抓放任务使用双臂服务和 `--frame world`，启动服务时需加载 `environment.dual_desktop.yaml`；模拟模式也需加载。

### 速度与规划

记录、碰撞查询和拖动不使用本节参数。Servo 使用独立的线速度、角速度参数，见下方说明。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--velocity` / `--acceleration` | `0.05` | 缩放 `(0, 1]` |
| `--planning-time` | `10` s | 规划允许用时；直线路径示例不使用此参数 |
| `--pipeline` | `ompl` | `ompl` / `chomp` / `stomp`，适用范围见上表 |
| `--planner` | `RRTConnect` | 仅用于 OMPL；CHOMP/STOMP 省略 |

OMPL 算法：`RRTConnect`、`RRTstar`、`PRM`、`EST`、`LBKPIECE`、`BiTRRT`、`AnytimePathShortening`。
夹爪开合速度不受 `--velocity` / `--acceleration` 控制。

### 关节与命名姿态

| 参数 | 默认值 | 说明 / 示例 |
|---|---|---|
| `--joint` | 所选臂第 1 关节 | `joint1-a1_l`…`joint7-a1_l` 或 `joint1-a1_r`…`joint7-a1_r`；`joint_goal` |
| `--delta` | `0.1` | 相对关节角；`joint_goal` |
| `--target` | `named_pose`：`ready`；`collision_check`：当前 | 姿态名称，如 `home` / `ready` |

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

`arm_gripper_task` 使用双臂服务：以右臂当前姿态确定左臂镜像目标，左臂先镜像就位，再同步取瓶 → 回起点 → 停顿 → 放回松爪。
先预览，再按输出坐标各摆一个水瓶。真机确认夹到瓶子后才开始搬运。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--diameter` / `--bottle-height` | `0.05` / `0.18` m | 瓶身尺寸，另加 2 cm 瓶颈 |
| `--reach` | `0.12` m | 自动选点的前伸距离 |
| `--lift` | `0.03` m | 夹取后抬升 |
| `--pause` | `2` s | 回起点后的停顿 |
| `--open-opening` | `1` | 张开开度 `[0, 1]` |
| `--grasp-opening` | 按瓶径计算 | 预期接触开度，可用实测值 |
| `--tcp-offset X Y Z` | 按模型计算 | 夹持中心相对 Link7 的偏移，m |
| `--bottle-position X Y Z` | 自动选点 | 右瓶中心，左瓶自动镜像；m，瓶底须落在桌面 |
| `--support-height` | 读取桌面 | 核对支撑面高度，m |
| `--orientation-tolerance` | `0.1` rad | 每段起点跟踪偏差，目标朝向保持不变 |

预览记录：`$ROS_HOME/tasks/arm_gripper_task_both.json`。

### 固定位置抓取放置

`fixed_pick_place` 使用双臂服务和一个水瓶：默认 **右 A→B → 左 B→C**，加 `--round-trip` 才继续 **左 C→B → 右 B→A**。每段松爪并撤离后再继续，C 自动取 A 关于双臂中线的镜像。
接力前左臂先向外让位；B 点放瓶后，放瓶手臂从当前撤离位置继续向所属手臂的外侧让位，让位目标随当前状态生成，并计入夹持中心偏移、瓶径和撤离余量。公共规划层选择朝向、关节构型和避障路径，另一臂随后接近取瓶。C 点松爪短暂撤离后重新夹取。
B 点位于立柱中线，A→B、B→C 水平距离均约 19.7 cm。默认瓶身直径为 4 cm，夹取中心位于瓶身中心上方 4 cm。A/C 在桌面上（桌面顶面 -0.340 m，18 cm 瓶身中心 -0.250 m），B 在深色板/托盘上（顶面 -0.330 m，中心 -0.240 m）；B 点保持在原位，保留左臂 B 点夹取偏置的现场标定。A/C 点从原 0.636 m 收到 0.59 m，肩到 Link7 的伸展率从约 96% 降到约 88%。从抓起到放下只硬性要求瓶身全程直立；抓取后抬升 6 cm，放到 B 或从 B 取出时先在 10 cm 上方中转越过托盘边框，再用最终瓶位 5D 直立目标下降到目标，路径允许弧线。
`left_b_grasp_offset` 是左臂在 B 点夹取的 world 坐标修正；按实物对位偏差，预设为 `[0, -0.015, 0]` m，设为 `null` 可关闭。修正同时应用于预抓取和最终夹取位置，不随候选朝向旋转。闭爪前输出模型夹持中心 XYZ 偏差及最大关节跟踪误差；这项局部补偿不能代替双臂安装位置和 TCP 标定。
示例只声明任务目标与抓放事件；公共规划层统一选择肘部向外、手腕少反折且保留关节余量的构型。每次取放只下发三条手臂轨迹：连续接近、连续直立搬运、连续撤离并让位。抓紧和松爪处确认状态，运动内部不再按逻辑目标逐段停下。
公共规划层保留 OMPL 避障，用 MoveIt 原生路径组合和整体时间参数化生成连续轨迹；示例不插值或重采样，预览与执行使用同一生成方式。

| 参数 | 说明 |
|---|---|
| `--config PATH` | 可选，[配置预设及注释](config/fixed_pick_place.yaml) |
| `--record-orientation` | 记录侧夹参考朝向，搭配 `--config` |
| `--round-trip` | 在默认的右 A→B、左 B→C 后，继续左 C→B、右 B→A |

预览检查完整接力，预览速度和加速度缩放至少为 0.5，缩短动画等待；执行仍使用指定的 `--velocity`、`--acceleration`，按最新位置逐段规划并运行。
公共规划层优先选择评分较好的自然构型，并按末端条件数、关节余量和腕部偏差比较候选；固定取放默认使用普通 RRTConnect，以便比较两条构型候选，默认单次运动规划预算为 10 秒，`--planning-time` 可调整；本地 FK、碰撞检查和整体时间参数化不再逐点调用 ROS 服务。
执行时只检查抓取接近可行性，随后按真实状态逐次规划并执行，不再先虚拟规划整段取放再重复规划一遍。更新零空间 IK 插件或规划配置后，先重启 MoveIt 服务，再重新预览。
默认放置位置误差门限为 15 mm，另有独立的瓶底高度误差门限 3 mm（`placement_height_tolerance`）。
搬运前检查规划终点；松爪前使用动作结束后的新鲜关节反馈，最多等 2 秒确认模型位置、瓶底高度和倾斜连续稳定 0.15 秒。
控制器的成功状态不代替这项检查。每次放置会输出 XYZ、瓶底间隙和最大关节跟踪误差，并保存到 `$ROS_HOME/tasks/fixed_pick_place_<arm>_placement.json`。模型高度检查不能替代实物几何与桌面标定。

### 双臂动作

`dual_joint_goal`：`--joint-index` 默认 `1`（范围 1…7），`--left-delta` / `--right-delta` 默认 `-0.2` / `0.2` rad。
`dual_cartesian_square`：两侧同时在 YZ 平面向上、向外、向下、返回，保持朝向；对应角点停顿，全路径通过碰撞检查后才执行。

### 零重力拖动

`freedrive --damping` 默认 `0.3`，范围 `[0, 5]`，越大越难拖动；`--duration` 为正数秒，默认不限时。
仅用于已开启重力补偿的真机。手臂静止时用手托住，再添加 `--execute` 进入拖动；Ctrl+C 退出并保持当前位置。
拖动时不会自动避障，手臂可能缓慢漂移。连接中断后自动保持；强制退出后可重启服务恢复。

### Servo 键盘遥操作

启动双臂服务时添加 `enable_servo:=true`，再运行 `servo_keyboard --execute` 打开控制窗口。
可控制左臂、右臂或双臂。双臂模式下沿相同方向移动，夹爪键作用于所选手臂。
默认使用公共 world 轴：从立柱后向前看，W/S 沿前后 X，A/D 沿左右 Y，R/F 沿上下 Z；旋转键也绕公共坐标轴操作。
按住 Shift 操作，松键停止、切换到其他窗口时暂停。退出后可继续运行普通运动示例。遥操作期间不能同时执行其他运动或手动拖动。
遥操作使用 `kp=120`；`kd` 保持原值。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--linear-speed` | `0.2` m/s | 法兰线速度，正数 |
| `--angular-speed` | `0.5` rad/s | 法兰角速度，正数 |
| `--arm` | `right` | 初始选臂 `left` / `right` / `both` |
| `--frame` | `world` | `world` / 公共 `base_link`，控制点为各臂 Link7 |

窗口高亮当前按键。

| 按键 | 功能 |
|---|---|
| W / S、A / D、R / F | 前后、左右、上下移动 |
| U / O、I / K、J / L | 分别绕 X、Y、Z 轴转动 |
| Z / X | 张开 / 夹取；松键或退出后继续夹持 |
| Tab | 切换左臂、右臂、双臂；停稳后重新按 Shift |
| 空格 / Enter | 暂停 / 恢复 |
| Esc / 关闭窗口 | 退出遥操作 |

平移时手臂会调整姿态，夹爪朝向可能小幅变化。接近活动边界或障碍时会减速或停止，可沿有空间的方向退离，但不会自动绕过障碍。
Servo 对腕部 `Link6` 与 Woan 夹爪底座的安装间隙单独处理：该配合间隙不触发近距减速，真实碰撞仍检查；其余近距减速会在日志中给出几何对和距离。
出现异常时先退出遥操作；无法恢复则重启服务。[运动记录与排查](../../tools/diagnostics/README.md#servo-运动记录)。
模拟中若持续提示等待状态更新，先完成一次[双臂联合运动](SHOWCASE.md#双臂联合运动)，再打开键盘窗口。
[遥操作运行命令](SHOWCASE.md#4-servo-键盘遥操作)。

## 运行结果

示例显示规划和执行结果，结束或中断后自动清理临时模型。
抓放任务持瓶时使用 `kp=120`（固定任务可配置），松爪后恢复原值；持瓶时发生异常会停止后续动作，并保持夹持。
精度报告、回放和对照见[诊断工具](../../tools/diagnostics/README.md)。
