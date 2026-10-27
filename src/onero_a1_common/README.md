# Onero A1 Common

供示例、应用和诊断工具复用的公共 Python 模块，通过标准 ROS 服务/Action 访问 MoveIt。

| 模块 / 接口 | 功能 |
|---|---|
| `constants.JOINTS` / `constants.LEFT_JOINTS` | A1R / A1L 七关节名称 |
| `moveit_client.MoveItClient` | 状态、FK、碰撞、参数、规划、显示、执行、取消及场景清理 |
| `moveit_client.MoveItError` | 服务和 Action 错误 |
| `gains.GainClient` | 单独或原子更新 kp/kd，确认生效，恢复本次修改的增益 |
| `payload.PayloadClient` | 原子更新负载，确认控制周期生效，恢复启动基线 |

导入：`from onero_a1_common.moveit_client import MoveItClient`。
[完整示例](../onero_a1_examples/README.md) · [规划器支持](../onero_a1_examples/README.md#规划器选择) · [参数](../onero_a1_examples/README.md#示例参数)

`select_arm()` 按 SRDF 配置关节、规划组、法兰和控制器，接受 `left` / `right` / `both`；单臂自动识别，双臂默认右臂。
`state()` 合并手臂、夹爪反馈并逐关节检查时间戳；`plan()` 从参数对象的 `pipeline`、`planner` 选管线和算法。
诊断工具通过继承客户端添加严格校验和报告，见 [DiagnosticClient](../../tools/diagnostics/client.py)。

`dual_cartesian_path()` 合成十四关节轨迹：同步左右路径点、按规划限位和缩放统一时长、检查双臂碰撞，对应路径点停止。
双臂轨迹由同一次 `ExecuteTrajectory` 请求交给两个控制器。

`freedrive()` 默认只检查条件，`execute=True` 进入单/双臂拖动；退出时保持当前位置并恢复控制器。
SDK 负责重力计算和单周期拖动，话题桥保持唯一命令发送循环。

`record_startup()` 保存所选臂实测关节，不产生运动；`startup_snapshot()` 按机械臂配置读取，支持跨服务重启沿用。
文件：`$ROS_HOME/startup_poses/recorded_startup.json`，工作区默认在 `.ros/`，再次记录只覆盖所选臂。

规划起点允许 ≤`0.0004 rad` 的反馈边界偏差，归界并记录提示；更大越界报错。反馈、FK、目标及模型限位保持原值。
