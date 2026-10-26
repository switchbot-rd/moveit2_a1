# Onero A1 MoveIt 2 Examples

本包提供机械臂功能示例：每个功能一个 Python 文件，展示目标构造和标准 MoveIt 调用。
运动示例默认只规划，添加 `--execute` 才执行。支持单臂（左臂或右臂）、双臂和 Woan Picker。

参数速查：[示例参数](#示例参数) · [服务启动参数](../../README.md#服务启动参数)。

## 代码结构

| 入口 | 源码 | 展示功能 |
|---|---|---|
| `joint_goal` | [joint_goal.py](onero_a1_examples/joint_goal.py) | 相对关节目标与关节空间规划 |
| `named_pose` | [named_pose.py](onero_a1_examples/named_pose.py) | 从 SRDF 读取 `ready` / `home` |
| `pose_goal` | [pose_goal.py](onero_a1_examples/pose_goal.py) | 法兰位置及朝向目标 |
| `cartesian_path` | [cartesian_path.py](onero_a1_examples/cartesian_path.py) | 笛卡尔直线、XY/XZ/YZ 平面正方形路径 |
| `orientation_constraint` | [orientation_constraint.py](onero_a1_examples/orientation_constraint.py) | 运动过程保持朝向 |
| `planning_scene` | [planning_scene.py](onero_a1_examples/planning_scene.py) | 临时碰撞体、避让及清理 |
| `collision_check` | [collision_check.py](onero_a1_examples/collision_check.py) | 查询当前或命名姿态的有效性与接触对 |
| `gripper` | [gripper.py](onero_a1_examples/gripper.py) | 通过 MoveIt 规划并执行夹爪开合 |
| `arm_gripper_task` | [arm_gripper_task.py](onero_a1_examples/arm_gripper_task.py) | 手臂与夹爪联合任务，以桌面水瓶取放演示动作配合 |
| `record_startup` | [record_startup.py](onero_a1_examples/record_startup.py) | 将当前关节位置保存为起始位置，不产生运动 |
| `return_startup` | [return_startup.py](onero_a1_examples/return_startup.py) | 规划返回已记录的起始位置 |
| `freedrive` | [freedrive.py](onero_a1_examples/freedrive.py) | 重力补偿手动拖动，退出后保持当前位置 |
| `dual_joint_goal` | [dual_joint_goal.py](onero_a1_examples/dual_joint_goal.py) | 双臂十四关节联合规划与执行 |
| `dual_cartesian_square` | [dual_cartesian_square.py](onero_a1_examples/dual_cartesian_square.py) | 双臂同时沿 YZ 平面画正方形，保持各自朝向 |
| `dual_gripper` | [dual_gripper.py](onero_a1_examples/dual_gripper.py) | 分别设置双夹爪开度并协调执行 |

公共接口位于 `onero_a1_common/moveit_client.py`，封装状态读取、FK、规划、显示、执行及取消。
[support.py](onero_a1_examples/support.py) 仅处理示例命令行和节点生命周期。
示例通过公共接口调用 MoveIt，再由控制器、适配层和 onero-sdk 控制机械臂。

## 示例运行指令

示例运行指令统一见 [SHOWCASE.md](SHOWCASE.md)：

- [单臂运行指令](SHOWCASE.md#单臂运行指令)：右臂或左臂独立运行。
- [双臂运行指令](SHOWCASE.md#双臂运行指令)：单侧控制、双臂联合运动及双夹爪控制。

环境与构建流程见[环境准备与构建](../../README.md#环境准备与构建)。
公共客户端说明见[Common 指南](../onero_a1_common/README.md)。

## 起始位置与返回

起始位置由 `record_startup` 显式记录。手臂保持静止时，该示例读取所选手臂的当前关节反馈，
保存为回位目标，不产生运动。服务启动时读取当前位置用于保持姿态，不自动记录或覆盖回位目标。

`return_startup` 从当前姿态规划返回已记录的位置。手臂移动不会更新记录；
再次运行 `record_startup` 只覆盖所选手臂的记录，双臂可分别记录，也可同时记录。

记录持久化保存在 `$ROS_HOME/startup_poses/recorded_startup.json`；
使用工作区环境时为 `.ros/startup_poses/recorded_startup.json`。
相同机械臂配置下，服务重启后可直接沿用，无需重新记录。
左右臂、sim / real 及不同型号、CAN 接口、安装方向和零位配置的记录分别保存；
当前配置没有记录时，`return_startup` 会提示先运行 `record_startup`。

## 双臂控制

双臂共用一个规划场景，可选择 `left_arm`、`right_arm`、`both_arms` 规划组。
双臂服务下，选臂方式取决于示例名称：

| 示例 | 选臂方式 |
|---|---|
| `joint_goal`、`pose_goal`、`cartesian_path`、`orientation_constraint`、`planning_scene`、`gripper`、`arm_gripper_task` | `--arm left` 或 `--arm right`，控制单侧 |
| `named_pose`、`record_startup`、`return_startup`、`collision_check`、`freedrive` | `--arm left`、`--arm right` 或 `--arm both` |
| `dual_joint_goal`、`dual_cartesian_square`、`dual_gripper` | 自动控制双臂，不接受 `--arm` |

world 原点位于两根部安装中心的中点，坐标轴方向与单臂一致。
具体操作见[双臂运行指令](SHOWCASE.md#双臂运行指令)。

## 规划器选择

支持 OMPL、CHOMP、STOMP，默认使用 OMPL / RRTConnect。

| 示例 | OMPL | CHOMP | STOMP |
|---|---|---|---|
| `joint_goal`、`named_pose`、`return_startup`、`planning_scene` | 支持 | 支持 | 支持 |
| `dual_joint_goal` | 支持 | 支持 | 支持 |
| `pose_goal` | 支持 | 不支持 | 支持 |
| `orientation_constraint` | 支持 | 不支持 | 不支持 |
| `gripper`、`dual_gripper` | 支持 | 不支持 | 不支持 |

夹爪固定使用 OMPL / RRTConnect。`cartesian_path`、`dual_cartesian_square` 使用独立的笛卡尔路径接口，
`collision_check` 只查询碰撞，`record_startup` 只记录当前位置，`freedrive` 切换手动拖动模式，均不选择规划器。
`arm_gripper_task` 的手臂运动使用笛卡尔路径，夹爪规划使用 OMPL，不接受 `--pipeline` / `--planner`。

## 示例参数

服务启动参数使用 `参数:=值`，见[服务启动参数](../../README.md#服务启动参数)。
示例参数使用 `--参数 值` 形式，运行指令见 [SHOWCASE.md](SHOWCASE.md)。
省略参数时使用表内默认值；`--execute` 和 `--help` 直接添加，不跟值。

分类：[通用设置](#通用设置) · [速度与规划](#速度与规划) · [关节与命名姿态](#关节与命名姿态) ·
[末端位姿](#末端位姿) · [笛卡尔路径与朝向约束](#笛卡尔路径与朝向约束) ·
[场景避障](#场景避障) · [夹爪开合](#夹爪开合) · [手臂与夹爪联合任务](#手臂与夹爪联合任务) · [双臂动作](#双臂动作) · [零重力拖动](#零重力拖动)。

### 通用设置

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--arm` | 单臂服务自动识别；双臂服务中的基础示例默认 `right` | `left`、`right`；`named_pose`、`record_startup`、`return_startup`、`collision_check`、`freedrive` 还支持 `both` | 选择要控制、记录或查询的手臂 | 基础示例 |
| `--frame` | `world` | `world`、`base_link` | 位移、旋转及场景对象的参考系 | 全部接受；空间目标使用 |
| `--execute` | 关闭 | 开关，无需赋值 | 执行规划结果；`freedrive` 进入拖动模式，省略时只检查条件 | 除 `record_startup`、`collision_check` 外 |
| `--help` | — | 开关，无需赋值 | 显示当前示例的参数帮助 | 全部 |

双臂服务下，`base_link` 指所选手臂的根部；选择 `both` 时指公共根部。
`arm_gripper_task` 固定使用 `--frame world`。

### 速度与规划

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--velocity` | `0.05` | `(0, 1]` | 相对关节规划限位的速度缩放 | 除 `record_startup`、`collision_check`、`freedrive` 外 |
| `--acceleration` | `0.05` | `(0, 1]` | 相对关节规划限位的加速度缩放 | 除 `record_startup`、`collision_check`、`freedrive` 外 |
| `--planning-time` | `10` | 正数，秒 | 规划请求时间预算，不是动作时长 | 除 `record_startup`、`collision_check`、`freedrive` 外接受；笛卡尔路径不使用该搜索预算 |
| `--pipeline` | `ompl` | `ompl`、`chomp`、`stomp`，支持情况见[规划器选择](#规划器选择) | 选择规划管线 | 手臂规划示例，`cartesian_path`、`dual_cartesian_square`、`arm_gripper_task` 除外 |
| `--planner` | `RRTConnect` | `RRTConnect`、`RRTstar`、`PRM`、`EST`、`LBKPIECE`、`BiTRRT`、`AnytimePathShortening` | 选择 OMPL 算法 | 接受 `--pipeline` 且选择 `ompl` 的示例 |

使用 CHOMP/STOMP 时省略 `--planner`。夹爪的速度、加速度缩放作用于规划轨迹，
实物开合由适配层的开度斜坡执行。

### 关节与命名姿态

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--joint` | 所选手臂第 1 关节 | 左：`joint1-a1_l` 至 `joint7-a1_l`；右：`joint1-a1_r` 至 `joint7-a1_r` | 选择要改变目标角度的关节 | `joint_goal` |
| `--delta` | `0.1` | 有限数值，rad | 关节目标相对当前角度的变化量 | `joint_goal` |
| `--target` | `named_pose`：`ready`；`collision_check`：当前姿态 | `home`、`ready` 或 SRDF 中定义的姿态名 | 选择命名目标或要查询的姿态 | `named_pose`、`collision_check` |

`return_startup` 的目标见[起始位置与返回](#起始位置与返回)，无需额外指定目标参数。

### 末端位姿

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--dx` | `0` | 有限数值，米 | 沿参考系 X 轴的相对位移 | `pose_goal`、`cartesian_path`、`orientation_constraint` |
| `--dy` | `0` | 有限数值，米 | 沿参考系 Y 轴的相对位移 | `pose_goal`、`cartesian_path`、`orientation_constraint` |
| `--dz` | `0.01` | 有限数值，米 | 沿参考系 Z 轴的相对位移 | `pose_goal`、`cartesian_path`、`orientation_constraint` |
| `--roll` | `0` | 有限数值，rad | 绕参考系 X 轴的增量旋转 | `pose_goal` |
| `--pitch` | `0` | 有限数值，rad | 绕参考系 Y 轴的增量旋转 | `pose_goal` |
| `--yaw` | `0` | 有限数值，rad | 绕参考系 Z 轴的增量旋转 | `pose_goal` |

目标对应 Link7 法兰。只旋转时，将 `--dx`、`--dy`、`--dz` 都设为 `0`。

### 笛卡尔路径与朝向约束

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--shape` | `line` | `line`、`square` | 选择直线或正方形路径 | `cartesian_path` |
| `--size` | 单臂 `0.02`；双臂 `0.05` | 正数，米 | 正方形边长 | `cartesian_path`（`--shape square`）、`dual_cartesian_square` |
| `--plane` | `yz` | `xy`、`xz`、`yz` | 正方形所在平面，基于 `--frame` | `cartesian_path`，`--shape square` |
| `--path-angle-tolerance` | `0.1` | 正数，rad | 运动过程中各轴允许的朝向偏差 | `orientation_constraint` |

### 场景避障

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--scene-delta` | `0.4` | 有限数值，rad | 所选手臂 joint4 目标角度的相对变化 | `planning_scene` |
| `--obstacle-radius` | `0.01` | 正数，米 | 临时球形障碍物半径 | `planning_scene` |

### 夹爪开合

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--opening` | `1` | `[0, 1]` | 所选夹爪的开度，0 全闭、0.5 半开、1 全开 | `gripper` |
| `--left-opening` | `1` | `[0, 1]` | 左夹爪开度 | `dual_gripper` |
| `--right-opening` | `1` | `[0, 1]` | 右夹爪开度 | `dual_gripper` |

### 手臂与夹爪联合任务

以下参数仅用于 `arm_gripper_task`。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `--diameter` | `0.05` m | 瓶身直径 |
| `--bottle-height` | `0.18` m | 瓶身高度，另含 2 cm 瓶颈 |
| `--reach` | `0.12` m | 自动选择瓶子位置时的前伸距离 |
| `--lift` | `0.03` m | 夹取后抬升距离 |
| `--pause` | `2` s | 回到本次执行起点后的停顿 |
| `--open-opening` | `1` | 张开开度，范围 `[0, 1]` |
| `--grasp-opening` | 按瓶径计算 | 预期接触开度，实物可指定标定值 |
| `--tcp-offset X Y Z` | 按夹爪模型计算 | 夹持中心相对 Link7 的偏移，米 |
| `--bottle-position X Y Z` | 自动选择桌面位置 | 指定瓶身中心坐标，米；瓶底必须落在桌面 |
| `--support-height` | 读取桌面模型 | 核对桌面高度，米 |
| `--orientation-tolerance` | `0.1` rad | 每段开始时允许的实际跟踪偏差，目标朝向保持不变 |

预览记录位于 `$ROS_HOME/tasks/arm_gripper_task_<left或right>.json`。
真机检查夹爪受阻开度后才搬运；RViz 模型不会产生实物接触反馈。失败时停止后续动作，不自动松爪。
持瓶去程、回程分别整体规划，转弯平滑过渡，只在起点停顿；执行后输出回位实测高度差。
夹取确认后自动将 `kp` 平滑切换为 `120`，放回松爪后恢复执行前的值；持瓶中止时保留当前增益。

### 双臂动作

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--joint-index` | `1` | 整数 `1` 至 `7` | 两臂要改变目标角度的关节序号 | `dual_joint_goal` |
| `--left-delta` | `-0.2` | 有限数值，rad | 左臂关节相对转角 | `dual_joint_goal` |
| `--right-delta` | `0.2` | 有限数值，rad | 右臂关节相对转角 | `dual_joint_goal` |

`dual_cartesian_square` 从两侧当前法兰位置出发，在参考系的 YZ 平面画正方形：
先向上，左臂向 +Y、右臂向 −Y 外移，再向下、返回各自起点，保持各自朝向。
两臂同时到达每个角点并短暂停顿，整圈路径通过碰撞检查后才允许执行。

### 零重力拖动

| 参数 | 默认值 | 可选值 / 单位 | 作用 | 适用示例 |
|---|---|---|---|---|
| `--damping` | `0.3` | `[0, 5]` | 拖动时的关节阻尼 kd，越大越难拖动，0 为无阻尼 | `freedrive` |
| `--duration` | 不限时 | 正数，秒 | 到时退出拖动；省略时按 Ctrl+C 退出 | `freedrive` |

仅支持 real 模式，需要重力补偿开启、手臂静止且已使能。进入后暂停所选手臂的轨迹控制，
以零位置刚度、重力补偿和所设阻尼手动引导；退出时保持拖动后的当前位置，恢复原先激活的控制器。
拖动不会更新已记录的起始位置，需要时另用 `record_startup` 记录。

重力补偿近似抵消重量，安装或负载参数有偏差时，松手仍可能漂移；手动拖动不经过 MoveIt 避障规划。
客户端心跳中断超过 2 秒时，话题桥退出拖动并请求保持当前位置，不自动失能；
客户端被强制结束后，轨迹控制器可能仍停用，可重启服务恢复。

## 运行结果

规划成功时输出轨迹点数和时长，执行成功时输出 MoveIt 执行结果；失败时返回非零状态码。
笛卡尔路径不完整时拒绝执行。碰撞查询输出状态有效性和接触对象，起始位置记录成功时输出确认信息。
Ctrl+C 请求取消活动 Action，并清理当前示例创建的临时场景对象。`arm_gripper_task` 成功、失败或中断结束后均自动移除水瓶模型。

精度报告、姿态存档、轨迹回放和 onero-sdk 对照见 [诊断工具](../../tools/diagnostics/README.md)。

## world 目标如何变成运动

启动文件按 `mounting` 发布 `world → base_link` 的 TF。
示例请求中标明参考系，MoveIt 结合 TF 和 URDF 求解关节目标并规划轨迹，
通过 `ExecuteTrajectory → 控制器 → 适配层 → onero-sdk → CAN` 执行。
按当前 humanoid_right 配置，world 的向上 1 cm 对应 base_link 的 +X 方向 1 cm；
各关节如何配合由逆运动学求解。参考系选项不会自动校准安装方向、模型或关节零位。
