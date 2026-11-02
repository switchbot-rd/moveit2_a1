# Onero A1 Python 接口

查询机械臂状态、规划和执行运动，以及调整增益、负载和 Servo 速度。
使用前先[加载环境并启动服务](../../README.md#启动服务)。

| 模块 / 接口 | 功能 |
|---|---|
| `constants.JOINTS` / `constants.LEFT_JOINTS` | A1R / A1L 七关节名称 |
| `moveit_client.MoveItClient` | 查询关节与末端位置，检查碰撞，规划、预览和执行运动 |
| `moveit_client.MoveItError` | 报告查询、规划或执行失败 |
| `gains.GainClient` | 调整 kp/kd、确认生效并恢复调整前的增益 |
| `payload.PayloadClient` | 设置负载、查看生效状态和恢复启动值 |
| `servo.ServoClient` | 开启/停止 Servo 会话，发送末端速度与夹爪开合方向 |
| `servo.KeyboardState` | 按键方向、Shift 使能和暂停状态 |
| `onero_a1_kinematics.Profile` | 设置自然姿态偏好，评价手臂姿态与运动路径 |

导入：`from onero_a1_common.moveit_client import MoveItClient`。
编写自己的脚本时，可参照[单关节运动示例](../onero_a1_examples/onero_a1_examples/joint_goal.py)；直接操作机械臂请使用[运行示例](../onero_a1_examples/SHOWCASE.md)。

## 查询与选臂

| 接口 | 用法 |
|---|---|
| `select_arm()` | 选择 `left` / `right` / `both`；省略时单臂自动识别，双臂默认右臂 |
| `state()` | 读取手臂和夹爪的当前关节位置 |
| `fk(q)` | 根据关节位置查询法兰位置和朝向 |
| `named_target(name)` | 读取 `home` / `ready` 等预设关节位置，不移动手臂 |
| `validity(q)` | 检查指定姿态是否有效，返回碰撞接触信息 |

## 规划与执行

先读取当前位置，再规划、预览，确认后执行。规划本身不会驱动手臂。

| 接口 | 适用任务 |
|---|---|
| `plan(start, goal)` | 规划一段运动；用 `pipeline`、`planner` 选择[规划器](../onero_a1_examples/README.md#规划器选择) |
| `plan_position(start, point, offset=...)` | 让工具上的指定点到达目标位置，自动选择朝向 |
| `plan_pose_candidates(start, poses)` | 提供多个目标姿态，选择可行路径 |
| `ik_candidates(poses, seed, extra_seeds=...)` | 为目标姿态寻找可用的关节位置 |
| `plan_upright(start, position, tip_to_object)` | 搬运时保持物体直立 |
| `plan_upright_position(start, position, tip_to_object)` | 把物体直立放到指定位置，允许绕竖直轴转向 |
| `plan_continuous(start, goals)` | 将同一手臂的多个目标连成一段连续运动 |
| `dual_cartesian_path()` | 生成双臂同步路径，两臂同时到达对应路径点并停止 |
| `smooth_trajectory(trajectory, jerk=10.)` | 平滑已有轨迹，限制加速度变化率，并检查平滑后的路径 |
| `show_plan(trajectory)` | 在 RViz 预览轨迹 |
| `execute(trajectory)` | 执行轨迹 |

直立搬运只要求物体保持直立，路径可以绕行；需要竖直升降时应另外指定路径。
各示例的[速度、位移和规划参数](../onero_a1_examples/README.md#示例参数)可作为脚本配置参考。

## 记录起点与手动拖动

`record_startup()` 保存所选臂当前位置，`startup_snapshot()` 读取记录。服务重启后仍可使用；更换设备或安装配置后重新记录。
记录文件位于 `$ROS_HOME/startup_poses/recorded_startup.json`，工作区默认在 `.ros/` 下；再次记录只覆盖所选臂。

`freedrive()` 默认只检查条件；指定 `execute=True` 进入拖动，退出时保持当前位置。使用方法见[零重力拖动](../onero_a1_examples/README.md#零重力拖动)。

## 使用注意

反馈过期、起点超出限位或存在碰撞时，规划会报错。先检查反馈、起始姿态和现场障碍物。
默认尽量保持肘部向外、减少手腕反折，姿态偏好可在 [posture.yaml](../onero_a1_kinematics/config/posture.yaml) 中调整。
腕部相对参考姿态的转动默认限制为 90°；[固定抓放配置](../onero_a1_examples/config/fixed_pick_place.yaml) 使用 75°。起点已超限时允许向范围内退出；直接指定关节角的普通运动示例仍按指定目标运行。
