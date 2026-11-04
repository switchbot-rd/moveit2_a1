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
| [collision_contacts](onero_a1_examples/collision_contacts.py) | 在 RViz 持续显示碰撞接触点、对象名称和穿透深度 |
| [planning_compare](onero_a1_examples/planning_compare.py) | 比较规划方案及平滑前后结果，预览或执行选中的方案 |
| [gripper](onero_a1_examples/gripper.py) | 夹爪开合 |
| [arm_gripper_task](onero_a1_examples/arm_gripper_task.py) | 双臂镜像取瓶、同步搬运并放回 |
| [fixed_pick_place](onero_a1_examples/fixed_pick_place.py) | 单瓶 A→B→C 双臂接力，可选往返 |
| [mtc_pick_place](onero_a1_examples/mtc_pick_place.py) | MTC 单瓶 A→B→C 双臂接力，可选回程 |
| [record_startup](onero_a1_examples/record_startup.py) | 记录起始位置，不运动 |
| [return_startup](onero_a1_examples/return_startup.py) | 返回已记录的起始位置 |
| [freedrive](onero_a1_examples/freedrive.py) | 重力补偿手动拖动 |
| [servo_keyboard](onero_a1_examples/servo_keyboard.py) | 键盘平移、旋转与夹爪抓放 |
| [dual_joint_goal](onero_a1_examples/dual_joint_goal.py) | 双臂同步关节运动 |
| [dual_cartesian_square](onero_a1_examples/dual_cartesian_square.py) | 双臂同时画正方形 |
| [dual_gripper](onero_a1_examples/dual_gripper.py) | 双夹爪协调开合 |

## 示例运行指令

完整命令见 [SHOWCASE.md](SHOWCASE.md)：[单臂](SHOWCASE.md#单臂运行指令) / [双臂](SHOWCASE.md#双臂运行指令)。

## 运行日志

每次运行会在 `.ros/log/` 下多一个 `onero_a1_<示例名>_<时间>_<进程号>.log`，开头会打印本次路径。它包含终端上的全部内容：示例自己的打印、ROS 的 `[INFO]`/`[WARN]`/`[ERROR]`。排查问题或反馈时直接把这个文件发出来即可。

`.ros/log/` 位于工作区（`setup_env.sh` 把 `ROS_HOME` 指到这里），不是用户主目录的 `~/.ros/log`。同目录下另有 ROS 自己按节点写的日志，那里只有 `[INFO]`，没有示例打印。

终端太吵时，可只保留警告和错误：在命令末尾加 `--ros-args --log-level warn`。

## 起始位置与返回

静止时运行 `record_startup` 保存当前位置，用 `return_startup` 返回。连续试用多个基础运动示例时，每项完成后先返回起始位置。相同设备配置下，重启服务后仍可使用；更换设备或安装配置后重新记录。
文件：`$ROS_HOME/startup_poses/recorded_startup.json`，工作区默认在 `.ros/` 下；无匹配记录时须先记录。

## 双臂控制

双臂共用场景和 world 坐标系，原点在两根部中心的中点。

| 示例 | `--arm` |
|---|---|
| `named_pose`、`record_startup`、`return_startup`、`collision_check`、`collision_contacts`、`freedrive` | `left` / `right` / `both` |
| `dual_joint_goal`、`dual_cartesian_square`、`dual_gripper`、`arm_gripper_task`、`fixed_pick_place`、`mtc_pick_place` | 自动双臂，不传此参数 |
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
`planning_compare` 用 `--pipelines` 选择多个方案，见[轨迹平滑与规划比较](#轨迹平滑与规划比较)。

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
| `--target` | `named_pose`：`ready`；`collision_check` / `collision_contacts`：当前 | 姿态名称，如 `home` / `ready` |

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

### 碰撞接触点可视化

`collision_contacts` 在 RViz 中用紫色球标出碰撞位置，白色文字显示碰撞对象和穿透深度（mm），不移动机器人。
运行后查看 `Collision Contacts`；旧窗口没有此项时，添加 `MarkerArray`，话题选 `/collision_contacts`。无碰撞时不显示标记，按 Ctrl+C 结束后自动清除接触点和演示球。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--target` | 当前实测姿态 | 可选 `home` / `ready`；只查询该姿态，RViz 场景机器人仍显示实测姿态 |
| `--rate` | `5` Hz | 查询频率，正数 |
| `--marker-size` | `0.02` m | 接触点直径和文字高度，正数 |
| `--demo-obstacle` | 关闭 | 在所查姿态的法兰处放置临时碰撞球，退出时删除；`--arm both` 时放在右臂法兰 |
| `--obstacle-radius` | `0.04` m | 演示球半径，正数；仅配合 `--demo-obstacle` |

仿真中可加 `--demo-obstacle` 快速查看效果。演示球固定在启动时的位置，运行期间会影响其他任务的避障。显示的是模型计算的接触点，不是真机接触检测结果。

### 轨迹平滑与规划比较

`planning_compare` 从同一起点到同一目标，比较各方案的规划耗时、关节路程和预计执行时间，列出 Ruckig 平滑前后的结果。默认选择平滑后预计执行时间最短的可用方案，在 RViz 预览；添加 `--execute` 只执行选中的一条。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--target` | 不指定 | 以 `home` / `ready` 等命名姿态为运动目标；不与 `--joint`、`--delta` 同用 |
| `--joint` / `--delta` | 所选臂第 1 关节 / `0.1` rad | 相对关节目标，用法与 `joint_goal` 相同 |
| `--pipelines` | `ompl chomp stomp` | 空格分隔要比较的规划器；也可只选一个观察平滑效果 |
| `--ompl-planner` | `RRTConnect` | OMPL 算法，可选值见上方规划器说明 |
| `--smoothing` | `ruckig` | 平滑方式；`none` 使用规划器原始结果 |
| `--jerk` | `10` rad/s³ | 加速度变化率上限，正数；越小通常越慢、更柔和 |
| `--select` | `duration` | 按预计执行时间选方案；`length` 按关节路程选 |

规划耗时是电脑计算用时，预计执行时间是手臂运动时长。关节路程用于比较各关节转动量，单位为 rad，并非末端移动的米数。
平滑后路程通常不变，运动时间可能增加。某个方案失败会显示原因并继续比较其余方案；开启平滑时，平滑失败的方案不参与选择。实际平顺性请在真机上对比。
若提示轨迹控制器未激活，先退出拖动或遥操作；若服务曾反馈失联，恢复通信后重启机械臂服务，再运行执行命令。

### 夹爪开合

开度 `[0, 1]`：0 全闭、0.5 半开、1 全开。
`gripper --opening` 默认 `1`；`dual_gripper --left-opening` / `--right-opening` 均默认 `1`。

### 手臂与夹爪联合任务

`arm_gripper_task` 让两臂各搬一个水瓶：左臂先摆到与右臂对称的位置，再同步取瓶 → 回起点 → 停顿 → 放回松爪。
先不带 `--execute` 预览，再按输出坐标左右各摆一个水瓶，最后添加 `--execute` 执行。真机上按提示确认夹到瓶子后才开始搬运。

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

`fixed_pick_place` 使用双臂搬运一个水瓶，默认 **右臂 A→B，左臂 B→C**。添加 `--round-trip` 后，再由 **左臂 C→B，右臂 B→A** 返回。交接时先放下、松爪并让位，再由另一臂取瓶；搬运中保持瓶身直立。

1. 启动双臂服务，加载 `environment.dual_desktop.yaml`。
2. 不带 `--execute` 运行示例，检查 RViz 中的路径和放置位置。
3. 在 A 点摆一个水瓶，清空 B/C 点，再使用相同配置添加 `--execute` 执行。

点位、瓶子尺寸和抓取朝向见[任务配置](config/fixed_pick_place.yaml)，位置单位为米，使用 world 坐标系：

| 位置 | 配置项 | 用途 |
|---|---|---|
| A | `pick_position` | 右臂取瓶的起点 |
| B | `place_position` | 两臂交接的放瓶位置 |
| C | `c_position` | 左臂放瓶的终点；设为 `null` 时使用 A 的左右镜像 |

| 参数 | 说明 |
|---|---|
| `--config PATH` | 使用自定义任务配置；省略时使用项目预设 |
| `--record-orientation` | 记录侧夹参考朝向，搭配 `--config` |
| `--round-trip` | 在默认的右 A→B、左 B→C 后，继续左 C→B、右 B→A |

预览动画会加速显示，实际运动速度以 `--velocity`、`--acceleration` 为准。修改设备或任务配置后重新预览；往返预览和执行都需添加 `--round-trip`。

若左臂在 B 点对位有偏差，可调整 `left_b_grasp_offset`；设为 `null` 关闭。该修正不能代替双臂安装位置和工具工作点标定。
松爪前会检查瓶子位置、瓶底高度和倾斜，超出配置中的容差时中止。放置结果保存在 `$ROS_HOME/tasks/fixed_pick_place_<arm>_placement.json`；仍需核对实物与桌面位置。

### MTC 抓取放置

`mtc_pick_place` 让右臂将水瓶从 A 搬到 B，再由左臂从 B 搬到 C；加 `--round-trip` 接力返回 A 点。

1. 执行 `source setup_env.sh`，启动双臂服务，加载 `environment.dual_desktop.yaml` 并启用 Woan Picker。
2. 按[运行命令](SHOWCASE.md#mtc-抓取放置)先预览。在 RViz `Task Tree` 中单击有成功方案的任务，再从右侧解列表选择方案；保持终端运行，Ctrl+C 结束。
3. 在 A 点摆瓶、清空 B/C 点，按实物质量设置 `--mass`，添加 `--execute` 执行。本示例不传 `--arm`。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--config PATH` | [双臂预设](config/mtc_pick_place.yaml) | A/B/C 点位、瓶子尺寸及抓取设置 |
| `--round-trip` | 关闭 | 在 A→B→C 后继续 C→B→A |
| `--max-solutions` | `3` | 最多搜索的完整方案数，正整数 |
| `--grasp-angle-step` | `1.5708` rad | 抓取朝向间隔，默认 90° |
| `--task-timeout` | `60` s | 每段搜索时限 |
| `--planning-time` | `10` s | 单次连接规划时限 |
| `--planner` | `RRTConnect` | 搬运路径的 OMPL 规划器 |
| `--mass` | `0.1` kg | 用于负载补偿，执行前按实物填写 |
| `--smoothing` | `none` | `ruckig` 开启平滑，可能增加运动时间 |
| `--reset-task` | 关闭 | 清理任务模型并恢复两臂启动负载；搭配 `--execute` |

点位、瓶子尺寸和夹取/放置补偿见[任务配置](config/mtc_pick_place.yaml)。A/B/C 为 world 坐标系中的瓶身中心，单位 m；修改瓶高时同步调整三个点的中心高度，再重新预览。

夹取或放置失败后会停止后续动作，持瓶时保留夹持、模型和负载。若提示已有附着水瓶，先托住水瓶，再依次操作：

```bash
# 张开两侧夹爪后，取下实物水瓶
ros2 run onero_a1_examples dual_gripper --left-opening 1 --right-opening 1 --execute

# 确认实物已取下，再恢复两臂负载并清理模型
ros2 run onero_a1_examples mtc_pick_place --reset-task --execute
```

### 双臂动作

`dual_joint_goal`：`--joint-index` 默认 `1`（范围 1…7），`--left-delta` / `--right-delta` 默认 `-0.2` / `0.2` rad。
`dual_cartesian_square`：两侧同时在 YZ 平面向上、向外、向下、返回，保持朝向；对应角点停顿，全路径通过碰撞检查后才执行。

### 零重力拖动

`freedrive --damping` 默认 `0.3`，范围 `[0, 5]`，越大越难拖动；`--duration` 为正数秒，默认不限时。
默认进入拖动前恢复启动负载基线；确实带物拖动时加 `--keep-payload` 保留当前负载补偿。
仅用于已开启重力补偿的真机。手臂静止时用手托住，再添加 `--execute` 进入拖动；Ctrl+C 退出并保持当前位置。
起始姿态碰撞导致轨迹控制器未启动时，也可用拖动调整姿态；调整后重启服务，重新进行起始姿态检查。
拖动时不会自动避障，手臂可能缓慢漂移。连接中断后自动保持；强制退出后可重启服务恢复。

### Servo 键盘遥操作

启动单臂或双臂服务时添加 `enable_servo:=true`，再运行 `servo_keyboard --execute` 打开控制窗口。
可控制左臂、右臂或双臂。双臂模式下沿相同方向移动，夹爪键作用于所选手臂。
默认使用公共 world 轴：从立柱后向前看，W/S 沿前后 X，A/D 沿左右 Y，R/F 沿上下 Z；旋转键也绕公共坐标轴操作。
按住 Shift 操作，松键停止、切换到其他窗口时暂停。退出后可继续运行普通运动示例。遥操作期间不能同时执行其他运动或手动拖动。
遥操作使用 `kp=120`；`kd` 保持原值。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--linear-speed` | `0.2` m/s | 法兰线速度，正数 |
| `--angular-speed` | `0.5` rad/s | 法兰角速度，正数 |
| `--arm` | 单臂自动识别，双臂 `right` | 初始选臂 `left` / `right` / `both` |
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
因接近障碍物减速时，日志会显示相关物体和距离，便于判断退离方向。
出现异常时先退出遥操作；无法恢复则重启服务。[运动记录与排查](../../tools/diagnostics/README.md#servo-运动记录)。
模拟中若持续提示等待状态更新，先完成一次[双臂联合运动](SHOWCASE.md#双臂联合运动)，再打开键盘窗口。
[遥操作运行命令](SHOWCASE.md#4-servo-键盘遥操作)。

## 运行结果

示例显示规划和执行结果，结束或中断后自动清理临时模型。
抓放任务持瓶时使用 `kp=120`（固定任务可配置），松爪后恢复原值；持瓶时发生异常会停止后续动作，并保持夹持。
精度报告、回放和对照见[诊断工具](../../tools/diagnostics/README.md)。
