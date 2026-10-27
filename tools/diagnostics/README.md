# 诊断工具

控制链路采集、姿态存档、轨迹回放、精度验收及 SDK 对照。公共通信使用 `onero_a1_common`，严格校验和报告由 `DiagnosticClient` 提供。
功能演示见 [Examples](../../src/onero_a1_examples/README.md)。

## 运行环境

在工作区根目录先执行 `source setup_env.sh`，ROS 工具需对应服务已启动。
环境包含 numpy、scipy、rosidl_runtime_py；SDK 直接执行使用 `ONERO_RUNTIME_PYTHON`，默认指向项目 `.venv`。
安装见[环境准备与构建](../../README.md#环境准备与构建)。

| 模块 | 功能 |
|---|---|
| `record_tracking` | 采集控制器、桥接、SDK、Action 及日志 |
| `saved_pose` | 绑定模型及硬件身份的实测姿态 |
| `trial` | 规划存档、返回起点、原时间回放、精度验收 |
| `compare_trials` | 同轨迹、配置和起点下的报告对比 |
| `sdk_compare` | 导出、执行及比较 SDK 控制序列 |
| `check_collision` | 自碰撞排除配置回归 |

## 控制链路采集

服务启用 `trace_control:=true`，发送运动前启动记录，动作结束后继续采集保持段，再 Ctrl+C 停止：

```bash
python3 -m tools.diagnostics.record_tracking
python3 -m tools.diagnostics.record_tracking --output .logs/tracking/control.jsonl --duration 20
python3 -m tools.diagnostics.record_tracking --observe-only --duration 10
```

记录器不发目标、不使能、不占用 CAN；`--observe-only` 不会代替操作者停止运动。
默认输出 `.logs/tracking/*.jsonl`，对应 `*.summary.json` 比较原始规划终点与反馈。

| 字段 | 含义 |
|---|---|
| `sdk_input_position` / `sdk_input_velocity` | SDK 输入目标 |
| `command_position` / `command_velocity` | SDK 限幅后命令 |
| `raw_command_position` | 含 SDK 零位换算的底层目标 |
| `requested_torque_ff` | 请求重力补偿力矩 |
| `sdk_torque_ff` / `command_torque_ff` | SDK 限幅后 / 最终力矩前馈 |
| `sdk_ff_limited` / `bridge_ff_limited` | 限幅状态 |
| `payload.mass` / `payload.cog` | 当前质量 / 末端坐标系重心 |
| `payload.target_mass` / `payload.applied_revision` | 请求质量 / 控制周期已确认版本 |
| `period_s` / `io_s` / `command_age_s` | 周期 / I/O 耗时 / 命令年龄 |
| `missed_joint_indices` / `exception` | 丢帧关节 / 异常 |

发出 SDK 命令不代表运动完成，需结合反馈、Action 和丢帧判断；反馈力矩也不等同于末端接触力。

## 实测姿态存档

在有净空的静止姿态保存；加载默认只规划，添加 `--execute` 执行：

```bash
python3 -m tools.diagnostics.saved_pose save --file .logs/trials/air_baseline.json
python3 -m tools.diagnostics.saved_pose load --file .logs/trials/air_baseline.json
```

`onero_a1_joint_pose` 文件保存七关节、模型指纹和桥接身份，不含夹爪开度或环境。
模型、限位、安装、CAN、sim/real、夹爪配置不匹配时拒绝加载；加载检查限位、碰撞和路径，已有文件不覆盖。
它与 `record_startup` / `return_startup` 的起始位置记录分别管理。

## 固定轨迹与回放

在正确场景和起点规划存档：

```bash
python3 -m tools.diagnostics.trial plan \
  --frame world --dx 0 --dy 0 --dz 0.01 --output .logs/trials/line.json
```

确认路径后回放；需要返回起点时使用 prepare：

```bash
python3 -m tools.diagnostics.trial replay \
  --from-report .logs/trials/line.json --output .logs/trials/replay_01.json --execute

# 返回该轨迹的起点，默认只规划；添加 --execute 执行
python3 -m tools.diagnostics.trial prepare --from-report .logs/trials/line.json
```

报告含起点、完整轨迹、时间、参考系、模型指纹、轨迹哈希和桥接配置。
`replay` 保留采样点、速度、加速度、`time_from_start`，仅重置绝对启动时间，并按当前场景重新检查。
起点关节差须 ≤0.01 rad，法兰起点须满足位姿容差；改位移或速度需重新生成轨迹。
`prepare` 从实测姿态重新规划返回，不保证沿原路倒退；返回后仍检查起点条件。
`trial plan --execute` 可直接执行新规划并输出报告。

## 精度与配置对比

默认验收 Link7：位置 `0.005` m、朝向 `0.05` rad、执行后关节误差 `0.05` rad。
可用 `--position-tolerance`、`--orientation-tolerance`、`--goal-tolerance` 修改，仅影响报告判定。
Action 成功（`execution_action_result` 的 status=4、error_code=1）与精度验收独立，精度超限仍返回失败。
FK 来自关节反馈及 URDF；实物指尖精度需 TCP 标定和外部测量。

```bash
python3 -m tools.diagnostics.compare_trials \
  .logs/trials/replay_01.json .logs/trials/replay_02.json
```

对比核对模型、轨迹、时间、安装和实测起点，输出关节/FK 误差及三轴位移。
比较增益时只改变待评估参数，保持场景、起点、轨迹、速度、加速度、频率、负载一致。
kp/kd 支持启动设置和运行时平滑切换；`gain_state` 确认生效，`command_kp` / `command_kd` 为实际下发值。

## onero-sdk 与 MoveIt 对照

仅支持 `gripper_model:=none` 的单臂。A 阶段通过 MoveIt 执行并采集，B 阶段直接用 SDK `Arm.servoj()` 执行同一序列；
位置、速度、增益、时间、零位、安装、负载一致，B 阶段重力补偿按其自身反馈计算。

1. 准备完整执行报告和控制记录。默认保持段为 10 s，A 阶段动作结束后须再采集至少 11 s。
2. 导出对照包：`hold` 比较固定目标保持，`motion` 比较运动及保持。导出只读，绑定 SDK、模型和环境，变化后须重新导出。

```bash
python3 -m tools.diagnostics.sdk_compare export \
  --from-report .logs/trials/replay_01.json --trace .logs/tracking/control.jsonl \
  --phase hold --output .logs/trials/sdk_hold.json

# 预览 B 阶段命令
"$ONERO_RUNTIME_PYTHON" -m tools.diagnostics.sdk_compare run --bundle .logs/trials/sdk_hold.json

# 同配置 MoveIt 服务下返回 A 阶段起点；默认只规划，添加 --execute 执行
python3 -m tools.diagnostics.sdk_compare prepare --bundle .logs/trials/sdk_hold.json
```

`prepare` 使用 0.05 速度/加速度缩放。起点检查通过后，结束占用 CAN 的桥接服务，保持场景与姿态，再运行 B 阶段。
七关节须已使能且无故障，起点差 ≤0.01 rad / 5 mm；工具不会自动追赶起点。

```bash
"$ONERO_RUNTIME_PYTHON" -m tools.diagnostics.sdk_compare run \
  --bundle .logs/trials/sdk_hold.json --output .logs/trials/sdk_result.json --execute
```

工具使用 CAN 进程锁并检查桥接占用；其他 CAN 程序须自行停止。采集要求新鲜反馈，结束后继续刷新保持目标和重力补偿。
保持段允许短暂缺帧，故障和连续丢帧由 SDK 保护。运动中第一次 Ctrl+C 转入保持，再次 Ctrl+C 退出，退出前支撑机械臂。
事件写入 `<结果名>.hold_events.json`；对比输出关节误差、最后 3 s 平均法兰误差和命令完整性，保持段对照不评价动态跟踪。

退出后离线对比：

```bash
python3 -m tools.diagnostics.sdk_compare compare \
  --bundle .logs/trials/sdk_hold.json --result .logs/trials/sdk_result.json
```

## 自碰撞回归

在未加载桌面/支架的 sim 服务中运行：

```bash
python3 -m tools.diagnostics.check_collision
```

检查正常姿态和折叠反例。反例超出硬件限位，只作几何查询，禁止执行；应用查询用 `ros2 run onero_a1_examples collision_check`。

## 报告与故障定位

默认报告位于 `.logs/diagnostics/`，`--output` 指定路径，已有文件不覆盖。参数见各工具 `--help`；检查失败返回非零。

| 现象 | 检查 |
|---|---|
| 无逆解 / 路径不完整 | 起点、工作空间、朝向、限位、碰撞 |
| 控制器中止 | Action 错误码、跟踪容差、反馈时效 |
| 周期 / 命令年龄增大 | 线程调度、DDS、CAN 吞吐及排队 |
| 低速稳定偏差 | 负载、重力补偿、增益、驱动限制 |
| FK 误差小而实物偏差大 | 零位、安装变换、TCP |

控制器中止后可能以实测位置作为保持目标，判断原目标是否到位须对照原始规划终点，不能只看保持阶段的 reference/error。
