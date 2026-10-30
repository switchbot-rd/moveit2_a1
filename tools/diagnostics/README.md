# 诊断工具

采集运动数据、保存实测姿态、回放轨迹、检查精度并对比 SDK 执行结果。
[功能示例](../../src/onero_a1_examples/SHOWCASE.md) · [环境准备](../../README.md#环境准备与构建)

## 运行环境

在工作区根目录先执行 `source setup_env.sh`，ROS 工具需对应服务已启动。
SDK 直接执行使用 `ONERO_RUNTIME_PYTHON`，默认指向项目 `.venv`。

| 工具 | 功能 |
|---|---|
| `record_tracking` | 记录目标位置、实际运动和故障信息 |
| `saved_pose` | 保存当前位置，供以后返回 |
| `trial` | 保存、回放轨迹并检查到位精度 |
| `compare_trials` | 比较两次运动的误差 |
| `sdk_compare` | 比较 MoveIt 与 SDK 的执行效果 |
| `check_collision` | 检查手臂自身是否碰撞 |
| `servo_debug` | 记录遥操作输入、实际运动和停止原因 |

## Servo 运动记录

启动服务时添加 `servo_debug:=true` 自动记录遥操作过程。单臂服务还可添加 `trace_control:=true` 记录详细控制数据。
也可在服务运行后单独启动采集：

```bash
python3 -m tools.diagnostics.servo_debug
```

默认文件位于 `.ros/log/servo_debug/*.jsonl`，可用 `--output PATH` 指定；自动采集时用启动参数 `servo_debug_file:=PATH`。
关闭服务或按 Ctrl+C 结束采集，生成同名 `.summary.json` 摘要，可查看运动误差及减速、停止情况。
采集不会驱动机械臂。出现不顺畅或无法移动时，保留原始记录、摘要和对应 ROS 日志。

## 运动数据采集

服务启用 `trace_control:=true`，发送运动前启动记录，动作结束后继续采集保持段，再 Ctrl+C 停止：

```bash
python3 -m tools.diagnostics.record_tracking
python3 -m tools.diagnostics.record_tracking --output .logs/tracking/control.jsonl --duration 20
python3 -m tools.diagnostics.record_tracking --observe-only --duration 10
```

默认输出 `.logs/tracking/*.jsonl` 及同名 `.summary.json`，用于比较目标位置和实际位置。
`--observe-only` 仅观察，不会停止机械臂。

报告包含目标与实际位置、运行耗时、负载设置及通信故障。

判断是否到位应查看实际位置；反馈力矩不能直接当作夹持力。

## 实测姿态存档

在有净空的静止姿态保存；加载默认只规划，添加 `--execute` 执行：

```bash
python3 -m tools.diagnostics.saved_pose save --file .logs/trials/air_baseline.json
python3 -m tools.diagnostics.saved_pose load --file .logs/trials/air_baseline.json
```

姿态文件保存七关节实测角度和设备配置，不含夹爪开度或环境。
设备或模型配置不匹配、路径不可行时不能加载；已有文件不会被覆盖。
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

`replay` 按保存的路径和速度执行，并检查当前环境是否允许通过；修改位移或速度需重新生成轨迹。
回放前须回到原起点：关节差不超过 0.01 rad，法兰位置和朝向也须满足容差。`prepare` 可返回起点，路线可能与原路不同。
`trial plan --execute` 可直接执行新规划并输出报告。

## 精度与配置对比

默认验收 Link7：位置 `0.005` m、朝向 `0.05` rad、执行后关节误差 `0.05` rad。
可用 `--position-tolerance`、`--orientation-tolerance`、`--goal-tolerance` 修改，仅影响报告判定。
运动完成后仍会检查精度，误差超限则报告失败。软件估算位置不能代替实物指尖测量。

```bash
python3 -m tools.diagnostics.compare_trials \
  .logs/trials/replay_01.json .logs/trials/replay_02.json
```

报告显示关节误差、末端位置误差和三轴位移。比较参数效果时，每次只改变一个参数，保持设备、场景、起点和运动设置一致。

## onero-sdk 与 MoveIt 对照

仅适用于 `gripper_model:=none` 的单臂。在相同设备、起点和运动设置下，分别用 MoveIt 和 SDK 执行，比较保持或运动效果。

1. 准备完整执行报告和控制记录。默认保持段为 10 s，A 阶段动作结束后须再采集至少 11 s。
2. 导出对照包：`hold` 比较位置保持，`motion` 比较运动及保持；设备、模型或环境变化后重新导出。

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

执行前停止其他占用 CAN 的程序。运动结束后继续保持位置；第一次 Ctrl+C 停止运动并保持，再次 Ctrl+C 退出，退出前支撑机械臂。
报告显示关节和末端位置误差；`hold` 结果仅评价位置保持。

退出后离线对比：

```bash
python3 -m tools.diagnostics.sdk_compare compare \
  --bundle .logs/trials/sdk_hold.json --result .logs/trials/sdk_result.json
```

## 自碰撞检查

在未加载桌面/支架的 sim 服务中运行：

```bash
python3 -m tools.diagnostics.check_collision
```

检查正常姿态和折叠姿态的碰撞结果。折叠姿态超出硬件限位，只作几何查询，禁止执行。
查询实测姿态或命名姿态用 `ros2 run onero_a1_examples collision_check`。

## 报告与故障定位

默认报告位于 `.logs/diagnostics/`，`--output` 指定路径，已有文件不覆盖。参数见各工具 `--help`；检查失败返回非零。

| 现象 | 检查 |
|---|---|
| 目标不可达 / 路径不完整 | 调整起点、目标位置或朝向，检查障碍物 |
| 运动中止 | 查看日志中的停止原因、运动误差和通信状态 |
| 运动卡顿 / 延迟 | 检查电脑负载和 CAN 通信，保存运动记录 |
| 低速稳定偏差 | 负载、重力补偿、增益、驱动限制 |
| 软件位置准确但实物有偏差 | 核对关节零位、安装位置和指尖标定 |

判断原目标是否到位，应比较原目标与实际位置；停下后能稳定保持，不代表已到达原目标。

## 固定抓放接力快照验证

只启动独立规划服务，不加载控制器或连接真机。快照目录包含 `robot.urdf`、`robot.srdf`、`scene.bin`、`state.json`；当前支持 world/base_link 重合的双臂快照。

```bash
ROS_DOMAIN_ID=87 ros2 launch tools/diagnostics/relay_snapshot.launch.py snapshot:=/tmp/a1-relay-collision-query
# 另一终端，同一 ROS_DOMAIN_ID；输出目录必须尚不存在
ROS_DOMAIN_ID=87 python3 -m tools.diagnostics.validate_relay_snapshot \
  --snapshot /tmp/a1-relay-collision-query --output .logs/relay_validation
```

检查完整右 A→B、左 B→C、左 C→B、右 B→A；记录各阶段规划耗时、关节余量、腕部偏离参考量、肘部外展距离，并使用原场景的碰撞规则检查双臂每个 MoveIt 轨迹点。安装底座与安装板的固定装配接触不属于 `both_arms` 检查组。
`report.json` 保存通过/失败结果与四次释放位姿；逐段 `.bin` 保留原始轨迹和场景，`postures.json` 保存姿态骨架供复核。检查不改写或插值轨迹，连续边段由 MoveIt 的规划验证负责。阶段耗时包含 IK 候选比较和规划重试；`--planning-time` 是单次 MoveIt 请求预算。
