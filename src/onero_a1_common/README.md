# Onero A1 Common

可由 colcon 安装的公共 Python 模块，供 examples、应用程序及诊断工具复用。

- `constants.JOINTS`：A1R 七关节名称。
- `constants.LEFT_JOINTS`：A1L 七关节名称。
- `moveit_client.MoveItClient`：状态读取、FK、碰撞查询、参数读取、规划、显示、执行及取消/场景清理。
- `moveit_client.MoveItError`：服务和 Action 的基本错误。
- `payload.PayloadClient`：原子更新负载，等待控制周期确认生效，并恢复启动基线。
- `gains.GainClient`：单独或原子切换 kp/kd，等待控制周期确认生效，并恢复本次修改的增益。

客户端通过标准 ROS 接口访问 MoveIt。
`tools/diagnostics/client.py` 通过继承客户端实现报告记录和严格校验。

`MoveItClient.plan()` 从参数对象的 `pipeline`、`planner` 字段选择规划器；
支持情况见 [规划器选择](../onero_a1_examples/README.md#规划器选择)，
可选值和默认值见 [示例参数](../onero_a1_examples/README.md#示例参数)。

```python
from onero_a1_common.moveit_client import MoveItClient
```

可运行的完整用法见 `onero_a1_examples`。

`select_arm()` 根据当前 SRDF 选择关节、规划组、法兰和控制器，可选 `left`、`right`、`both`。
省略时自动识别当前单臂，双臂服务默认选择右臂。
`state()` 合并分开发布的手臂、夹爪状态，并按各关节时间戳检查新鲜度。
双臂轨迹通过同一次 ExecuteTrajectory 请求交给左右两个控制器。

`dual_cartesian_path()` 接收左右法兰对应的路径点，逐边调用 MoveIt 笛卡尔路径接口，
统一每条边的时长并合成十四关节轨迹；检查两臂同时运动时的碰撞，整条路径通过后返回轨迹。
合并轨迹按当前规划限位和请求的速度、加速度缩放校验并延长时间；两臂在对应路径点停止。

`freedrive()` 协调所选手臂的重力补偿拖动和轨迹控制器切换，默认只检查条件，
设置 `execute=True` 进入拖动。正常退出后保持当前位置并恢复控制器，支持单臂或双臂。
重力计算与单周期拖动由 onero-sdk 完成，桥接节点保持唯一的手臂命令发送循环。

`record_startup()` 将所选手臂的当前关节反馈持久化保存为起始位置，不产生运动；
`startup_snapshot()` 读取与当前机械臂配置匹配的记录，供回位规划使用。
记录按手臂分别保存，相同配置下可跨服务重启沿用；再次记录只覆盖所选手臂。
文件位于 `$ROS_HOME/startup_poses/recorded_startup.json`，工作区默认在 `.ros` 下。

手臂规划请求允许不超过 `0.0004 rad` 的起点反馈边界偏差，将该起点归到模型边界并记录提示；
更大的越界会直接报错。该处理只作用于规划请求的起点，关节反馈、FK、目标位置及模型限位保持原值。
