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
[完整示例](../onero_a1_examples/README.md) · [规划器支持](../onero_a1_examples/README.md#规划器选择) · [参数](../onero_a1_examples/README.md#示例参数)

`select_arm()` 接受 `left` / `right` / `both`；单臂自动识别，双臂默认右臂。
`state()` 返回手臂和夹爪的关节反馈；`plan()` 使用 `pipeline`、`planner` 参数选择规划器。

`ik_candidates(poses, seed, extra_seeds=...)` 对无碰撞任务位姿的逆解统一择优：从当前状态和任务参考姿态分别求逆解，按肘部/小臂外展、腕部参考姿态、关节余量、数值雅可比条件数和移动连续性合并排序。`plan_pose_candidates()` 的笛卡尔优先模式会评估全部候选路径；如果最佳候选的末端条件数、关节余量或腕部偏差偏弱，即使 Cartesian 路径完整也会尝试多 seed 关节空间分支。`plan_position()` 为工具上指定点选择自然的末端朝向；`plan_upright()` 约束附着物体直立，并按绝对 world yaw 采样绕竖直轴的朝向。固定取放的放置阶段直接调用 `plan_upright_position()`，以最终瓶位的 5D 直立目标（位置 + 竖直轴，yaw 自由）一次规划；只约束瓶身直立，不要求垂直下降，路径可以弧线。
`plan()` 比较 MoveIt 生成的路径候选；IK 的零空间优化是自然构型策略的一部分。
零空间 IK 参考 armsdk 的 `solve_ik_with_nullspace`：每次迭代叠加 DLS 主任务和投影后的关节/空间姿态次目标（肘部及小臂中点使用解析雅可比梯度），沿用上一状态热启动，保留限位、末端误差及碰撞检查。相邻笛卡尔采样点到位后结束 IK，零空间步幅随主任务步幅限制，减少沿途累积扭转。完整位姿使用 6×7 雅可比；自由位置目标使用 3×7 雅可比；直立持物目标使用 5×7 雅可比，保留竖直轴转向和冗余构型调整。
固定取放默认使用普通 `RRTConnect`，让共享层能够比较 `planning_candidates` 条构型候选；`RRTConnectOnline` 只返回一条未简化路径，仍可用于显式选择，但不是固定取放的默认值。整段圆滑仍由 MoveIt TOTG 完成，碰撞检查分辨率保持原配置。上述单目标接口保留 MoveIt 返回的轨迹。

`plan_continuous(start, goals)` 把同一手臂的一次完整运动交给公共层处理：逻辑目标共享规划搜索预算，前面目标未用完的份额留给后续目标，同时给每个搜索保留最低预算，避免前面的长搜索把后续目标饿死。MoveIt 原生 `RobotTrajectory` 和 TOTG 对整条几何路径统一生成时间参数，并在本地 MoveIt 场景中重新检查圆滑后的碰撞，FK 和逐点碰撞不再往返调用 ROS 服务；调用方只执行一次。直立持物先尝试带碰撞检查的原生笛卡尔路径，受阻时再用约束 OMPL 搜索；姿态约束同时用于两者和圆滑后的检查；碰撞/约束失败时最多收紧一次原生圆滑容差，J5/J7 姿态超限不再靠收紧 TOTG 重试。此接口读取 move_group 的实际关节限速，避免使用 URDF 导出的占位限速。

`dual_cartesian_path()` 生成双臂同步轨迹，两臂同时到达对应路径点并停止。

`freedrive()` 默认只检查条件，`execute=True` 进入单/双臂拖动；退出时保持当前位置，可继续执行普通运动。

`record_startup()` 保存所选臂实测关节，不产生运动；`startup_snapshot()` 按机械臂配置读取，支持跨服务重启沿用。
文件：`$ROS_HOME/startup_poses/recorded_startup.json`，工作区默认在 `.ros/`，再次记录只覆盖所选臂。

反馈过期、起点超出限位或存在碰撞时，规划报错。

自然构型规划会检查实际返回轨迹的 J5/J7 滚转。共享 `max_wrist_roll_deviation` 默认限制其相对参考姿态偏离不超过 90°；固定取放通过 `fixed_pick_place.yaml` 单独收紧到 75°。已超限的起点允许向范围内退出，终点须回到范围内。候选选择还会按条件数、关节余量和腕部偏差做软评分，优先选择腕部偏差接近 0.35 rad 以内的分支。该限制不改变硬件机械限位，显式普通关节角示例仍按其指定目标工作。
