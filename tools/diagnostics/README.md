# 诊断工具

用于记录控制链路、保存实测姿态、验收轨迹精度，以及比较 MoveIt 与 onero-sdk 的执行结果。
公共 ROS 通信使用 `onero_a1_common`，报告和严格检查由 `DiagnosticClient` 提供。
功能调用示例见 [onero_a1_examples](../../src/onero_a1_examples/README.md)。

## 运行环境

从工作区根目录运行命令：

```bash
source setup_env.sh
```

ROS 工具需要已构建的公共接口、模型与 MoveIt 启动包，以及 numpy、scipy、rosidl_runtime_py。
onero-sdk 直接执行使用 `ONERO_RUNTIME_PYTHON` 指定的项目运行环境，默认位于工作区 `.venv`。
安装步骤见[环境准备与构建](../../README.md#环境准备与构建)。

| 模块 | 功能 |
|---|---|
| `record_tracking` | 记录控制器、桥接输入/输出、Action 结果及日志 |
| `saved_pose` | 保存和加载绑定模型、硬件身份的实测关节姿态 |
| `trial` | 保存直线轨迹、返回轨迹起点、按原时间回放及精度验收 |
| `compare_trials` | 比较相同轨迹、配置和起点下的执行报告 |
| `sdk_compare` | 导出控制序列、运行 onero-sdk 对照并比较结果 |
| `check_collision` | 使用已知折叠几何验证自碰撞排除配置 |

## 控制链路采集

服务端启用 `trace_control:=true`，采集实际 onero-sdk 输入、限幅结果、CAN 时序和反馈。
在客户端发送运动之前启动记录器：

```bash
python3 -m tools.diagnostics.record_tracking

# 指定输出及采集时长
python3 -m tools.diagnostics.record_tracking \
  --output .logs/tracking/control.jsonl --duration 20

# 仅观察静止保持；此选项不会代替操作者停止运动
python3 -m tools.diagnostics.record_tracking --observe-only --duration 10
```

记录器只订阅话题和读取服务结果，不发送目标、不使能电机、不占用 CAN。
每份记录对应一次目标执行；动作结束后继续采集保持段，再按 Ctrl+C 结束。
文件默认位于 `.logs/tracking/`：

- `*.jsonl`：控制器参考/反馈、桥接数据、onero-sdk 命令、Action 状态及日志。
- `*.summary.json`：规划原始终点与反馈的逐关节差值。

| 字段 | 含义 |
|---|---|
| `sdk_input_position` / `sdk_input_velocity` | 传给 onero-sdk 的目标位置、速度 |
| `command_position` / `command_velocity` | onero-sdk 限幅后的命令 |
| `raw_command_position` | 包含 onero-sdk 零位换算的底层目标 |
| `requested_torque_ff` | 请求的重力补偿力矩 |
| `sdk_torque_ff` / `command_torque_ff` | onero-sdk 限幅后及最终下发的力矩前馈 |
| `sdk_ff_limited` / `bridge_ff_limited` | 力矩前馈限幅状态 |
| `payload.mass` / `payload.cog` | 当前 SDK 负载质量与末端坐标系下重心 |
| `payload.target_mass` / `payload.applied_revision` | 请求负载及已完成控制周期确认的版本 |
| `period_s` / `io_s` / `command_age_s` | 控制周期、onero-sdk I/O 耗时和命令年龄 |
| `missed_joint_indices` / `exception` | 丢帧关节和调用异常 |

onero-sdk 命令记录与驱动器实际完成运动是不同证据，需同时检查反馈、Action 结果和丢帧。
反馈力矩的物理含义取决于 onero-sdk/驱动器标定，不直接等同于实测末端接触力。

## 实测姿态存档

在已确认具有测试净空的姿态保存关节位置。保存只读，加载默认只规划：

```bash
python3 -m tools.diagnostics.saved_pose save \
  --file .logs/trials/air_baseline.json

python3 -m tools.diagnostics.saved_pose load \
  --file .logs/trials/air_baseline.json

python3 -m tools.diagnostics.saved_pose load \
  --file .logs/trials/air_baseline.json --execute
```

文件类型为 `onero_a1_joint_pose`，包含七关节角度、模型指纹和桥接身份。
模型、限位、安装、CAN、运行模式或夹爪配置不一致时拒绝加载，已有姿态文件不覆盖。
加载时检查限位、碰撞与返回路径，执行结果写入诊断报告。
姿态文件仅描述关节位置，不保存夹爪开度或环境对象。

`record_startup` 记录的起始位置由 `onero_a1_examples return_startup` 使用；它与单独保存的悬空测试位分别管理。

## 固定轨迹与回放

在正确的场景及测试起点下，规划并保存一次直线：

```bash
python3 -m tools.diagnostics.trial plan \
  --frame world --dx 0 --dy 0 --dz 0.01 \
  --output .logs/trials/line.json
```

报告包含起点、完整关节轨迹、时间参数、参考系、模型指纹、轨迹哈希和桥接配置。
确认路径后回放该轨迹：

```bash
python3 -m tools.diagnostics.trial replay \
  --from-report .logs/trials/line.json \
  --output .logs/trials/replay_01.json --execute
```

`replay` 保留采样点、速度、加速度和 `time_from_start`，重置绝对启动时间戳。
每次按当前场景检查轨迹。起点关节差必须不超过 0.01 rad，法兰起点满足配置的位姿容差。
目标/时间参数从报告读取，修改位移或速度需重新生成轨迹。

返回保存的轨迹起点：

```bash
python3 -m tools.diagnostics.trial prepare \
  --from-report .logs/trials/line.json

python3 -m tools.diagnostics.trial prepare \
  --from-report .logs/trials/line.json --execute
```

`prepare` 从实测姿态重新规划返回，不保证沿执行路径倒退。
返回后检查 0.01 rad 的起点条件；未满足时拒绝将其当作可重复回放的起点。
`trial plan --execute` 则直接执行其生成的直线并输出同类诊断报告。

## 精度与配置对比

默认验收点为 Link7 法兰，位置容差 0.005 m、朝向容差 0.05 rad、执行后关节容差 0.05 rad。
使用 `--position-tolerance`、`--orientation-tolerance`、`--goal-tolerance` 指定验收条件。
这些参数只影响报告判定，不修改控制器、增益或 onero-sdk 限幅。

执行 Action 成功与末端验收分别记录。`execution_action_result` 中 status=4、error_code=1
表示 MoveIt 执行成功；法兰精度超限仍使诊断报告返回失败。
误差由关节反馈和 URDF 计算，实物指尖精度需要 TCP 标定和外部测量。

使用同一轨迹进行多次回放，以不同输出名保存结果：

```bash
python3 -m tools.diagnostics.compare_trials \
  .logs/trials/replay_01.json .logs/trials/replay_02.json
```

对比工具核对模型、轨迹、时间、安装配置和实测起点，再显示关节/FK 误差及三轴位移。
比较增益时，以相同场景、起点、轨迹、速度、加速度、频率和负载运行，只改变待评估参数。
kp/kd 可通过启动参数设置，均支持运行时平滑切换；`gain_state` 记录其生效状态，控制记录的 `command_kp` / `command_kd` 为实际下发值。

## onero-sdk 与 MoveIt 对照

用于判断同一命令序列在 MoveIt 执行链和 onero-sdk 直接执行下的差异。
支持 `gripper_model:=none` 的单臂数据；采集 A 阶段数据时就应使用相同模型配置。
A 阶段由 MoveIt 执行并采集控制记录，B 阶段直接调用 onero-sdk Arm.servoj()。
控制序列的位置、速度、增益、时间、零位、安装及负载配置保持一致，重力补偿按 B 阶段反馈计算。

导出需要完整的执行报告和控制记录。默认保持段长度为 10 秒，
A 阶段动作结束后需继续采集至少 11 秒，供工具截取稳定保持段。

```bash
python3 -m tools.diagnostics.sdk_compare export \
  --from-report .logs/trials/replay_01.json \
  --trace .logs/tracking/control.jsonl \
  --phase hold \
  --output .logs/trials/sdk_hold.json
```

`--phase hold` 评估固定目标保持；`--phase motion` 评估完整运动及保持段。
导出只查询模型、FK、碰撞状态和输入文件，不发送运动。
对照包绑定 onero-sdk/模型/环境文件，相关内容变化时需要重新导出。

预览 B 阶段命令：

```bash
"$ONERO_RUNTIME_PYTHON" -m tools.diagnostics.sdk_compare run \
  --bundle .logs/trials/sdk_hold.json
```

需要恢复 A 阶段实测起点时，在同配置的 MoveIt 服务下运行：

```bash
python3 -m tools.diagnostics.sdk_compare prepare \
  --bundle .logs/trials/sdk_hold.json

python3 -m tools.diagnostics.sdk_compare prepare \
  --bundle .logs/trials/sdk_hold.json --execute
```

返回动作使用 0.05 速度/加速度缩放。显示起点检查通过后，结束占用该 CAN 的桥接服务，
保持实物场景与姿态，再由 B 阶段接管。B 阶段要求七关节已使能、无故障，
起点差不超过 0.01 rad / 5 mm，不会自动追赶不匹配的起点。

```bash
"$ONERO_RUNTIME_PYTHON" -m tools.diagnostics.sdk_compare run \
  --bundle .logs/trials/sdk_hold.json \
  --output .logs/trials/sdk_result.json --execute
```

工具使用 CAN 进程锁并检查话题桥占用。其他未使用进程锁的 CAN 程序应由操作者停止。
采集期间严格要求新鲜反馈。采集完成后保存结果，并继续刷新保持目标和重力补偿；
保持段允许短暂缺帧，故障和连续丢帧由 onero-sdk 保护。
运动中 Ctrl+C 转入保持，再次 Ctrl+C 退出；退出前按现场流程支撑机械臂。
保持事件写入 `<结果名>.hold_events.json`。

离线对比：

```bash
python3 -m tools.diagnostics.sdk_compare compare \
  --bundle .logs/trials/sdk_hold.json \
  --result .logs/trials/sdk_result.json
```

输出同一控制序列下的关节误差、最后 3 秒平均法兰误差及命令完整性结果。
保持段对照不用于评价动态跟踪。

## 自碰撞回归

在不加载桌面/支架的 `sim` 场景中运行：

```bash
python3 -m tools.diagnostics.check_collision
```

工具检查正常姿态和已知折叠反例。反例角度超出硬件限位，只用于几何查询，禁止执行。
应用中查询当前或命名姿态时，使用 `ros2 run onero_a1_examples collision_check`。

## 报告与故障定位

诊断报告默认写入 `.logs/diagnostics/`，`--output` 可指定路径，已有文件不覆盖。
`--help` 显示各工具参数。规划、执行、模型身份、起点或精度检查失败时返回非零。

| 现象 | 重点检查 |
|---|---|
| 目标无逆解或路径不完整 | 起点、可达范围、朝向约束、关节限位及碰撞 |
| 控制器中止 | Action 错误码、跟踪容差、反馈新鲜度 |
| 周期或命令年龄增大 | 控制线程调度、DDS、CAN 吞吐与排队 |
| 低速仍有稳定关节偏差 | 负载、重力补偿、增益与驱动器限制 |
| FK 误差小而实物末端偏差大 | 模型零位、安装变换和指尖 TCP |

控制器中止后可能把实测位置作为保持目标，定位时应对照原始规划终点，
不要仅使用保持阶段 controller_state 的 reference/error 判断是否到达原目标。
