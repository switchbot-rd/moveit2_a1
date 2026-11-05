# 功能示例运行指令

[功能、参数与使用条件](README.md) · [服务启动参数](../../README.md#服务启动参数)

[单臂命令](#单臂运行指令) · [双臂命令](#双臂运行指令)

先启动对应服务，再在另一终端加载环境，选择需要的示例运行。先不带 `--execute` 查看预览或检查结果，确认后再添加此参数执行；查询和记录命令不需要此参数。

## 单臂运行指令

### 右单臂运行指令

#### 1. 启动服务

```bash
# 在工作区根目录运行
source setup_env.sh
```

```bash
# 模拟服务（与真机服务二选一）
ros2 launch onero_a1_moveit_config moveit.launch.py hardware_mode:=sim arm:=right
```

```bash
# 真机服务
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real arm:=right \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

```bash
# 另一终端：运行示例前加载环境
source setup_env.sh
```

#### 2. 记录与返回起始位置

```bash
ros2 run onero_a1_examples record_startup
```

```bash
ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 3. 单关节抬臂

```bash
ros2 run onero_a1_examples joint_goal --delta 0.6 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 4. 法兰位置目标

```bash
ros2 run onero_a1_examples pose_goal \
  --dx -0.05 --dy -0.10 --dz 0.10 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 5. 法兰朝向目标

```bash
ros2 run onero_a1_examples pose_goal \
  --dx 0 --dy 0 --dz 0 --yaw 0.5 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 6. 笛卡尔直线

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape line --dx 0 --dy 0 --dz 0.12 \
  --velocity 0.5 --acceleration 0.5 --execute
```

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape line --dx 0 --dy -0.12 --dz 0 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 7. 笛卡尔正方形

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape square --plane yz --size 0.10 \
  --dx -0.05 --dy 0 --dz 0.02 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 8. 保持朝向搬运

```bash
ros2 run onero_a1_examples orientation_constraint \
  --dx -0.04 --dy -0.10 --dz 0.10 \
  --path-angle-tolerance 0.05 --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 9. 障碍物避让

```bash
ros2 run onero_a1_examples planning_scene \
  --scene-delta 0.4 --obstacle-radius 0.03 --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 10. 命名姿态

```bash
ros2 run onero_a1_examples named_pose --target ready --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 11. 夹爪标定与开合

```bash
ros2 service call /gripper_controller/calibrate std_srvs/srv/Trigger '{}'

# 需要停止时执行
ros2 service call /gripper_controller/stop std_srvs/srv/Trigger '{}'
```

```bash
ros2 run onero_a1_examples gripper --opening 1 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples gripper --opening 0.5 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples gripper --opening 0 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 12. 碰撞查询

```bash
ros2 run onero_a1_examples collision_check
ros2 run onero_a1_examples collision_check --target ready
ros2 run onero_a1_examples collision_check --target home
```

碰撞接触点可视化（以下命令任选一条，持续运行，Ctrl+C 结束）：

```bash
# 观察当前姿态与现有场景的碰撞
ros2 run onero_a1_examples collision_contacts

# 仿真快速演示：在当前法兰处放置临时球体，立即查看接触点
ros2 run onero_a1_examples collision_contacts --demo-obstacle

# 检查命名姿态，不移动机器人；球体也按该姿态放置
ros2 run onero_a1_examples collision_contacts --target ready --demo-obstacle
```

在 RViz 的 `Collision Contacts` 查看紫色接触点与对象标签，退出后自动清理。显示设置和参数见[碰撞接触点可视化](README.md#碰撞接触点可视化)。

#### 13. 规划器选择

```bash
# OMPL / RRTstar
ros2 run onero_a1_examples joint_goal --delta 0.1 --pipeline ompl --planner RRTstar \
  --velocity 0.5 --acceleration 0.5

# CHOMP
ros2 run onero_a1_examples joint_goal --delta 0.1 --pipeline chomp \
  --velocity 0.5 --acceleration 0.5

# STOMP
ros2 run onero_a1_examples joint_goal --delta 0.1 --pipeline stomp \
  --velocity 0.5 --acceleration 0.5
```

#### 14. 零重力拖动（仅 real）

```bash
# 只检查条件，不改变控制模式
ros2 run onero_a1_examples freedrive

# 进入拖动，Ctrl+C 退出
ros2 run onero_a1_examples freedrive --damping 0.3 --execute

# 也可指定 30 秒后自动退出
ros2 run onero_a1_examples freedrive --damping 0.3 --duration 30 --execute
```

#### 15. 轨迹平滑与规划比较

```bash
# 比较三个规划器及平滑前后结果，只预览选中的方案
ros2 run onero_a1_examples planning_compare --delta 0.2 \
  --velocity 0.5 --acceleration 0.5

# 只看 OMPL 的平滑效果，目标为 ready 姿态
ros2 run onero_a1_examples planning_compare --pipelines ompl --target ready --jerk 10 \
  --velocity 0.5 --acceleration 0.5

# 按关节路程最短选择并执行一条平滑后的方案
ros2 run onero_a1_examples planning_compare --delta 0.2 --select length \
  --velocity 0.5 --acceleration 0.5 --execute
```

关闭平滑可加 `--smoothing none`。双臂服务下使用 `--arm left` 或 `--arm right` 选择一侧。
[参数与结果说明](README.md#轨迹平滑与规划比较)。

### 左单臂运行指令

| 项目 | 右臂参数 | 左臂修改 |
|---|---|---|
| 启动服务 | `arm:=right` | `arm:=left`，sim / real 均适用 |
| CAN 通道 | 默认 `can0` | 默认 `can1`，实际接线不同时用 `can_channel` 指定 |
| 安装方向 | 默认 `humanoid_right` | 默认 `humanoid_left`；显式指定 `mounting` 时也需同步修改 |
| 单关节抬臂 | `joint_goal --delta 0.6` | `--delta -0.6` |
| 法兰位置目标 | `pose_goal --dy -0.10` | `--dy 0.10` |
| 法兰朝向目标 | `pose_goal --yaw 0.5` | `--yaw -0.5` |
| 水平向外侧直线 | `cartesian_path --dy -0.12` | `--dy 0.12` |
| 保持朝向搬运 | `orientation_constraint --dy -0.10` | `--dy 0.10` |
| 障碍物避让 | `planning_scene --scene-delta 0.4` | `--scene-delta -0.4` |
| 规划器选择示例 | `joint_goal --delta 0.1` | `--delta -0.1`，规划器参数沿用 |

## 双臂运行指令

### 1. 启动服务

```bash
# 模拟服务（与真机服务二选一）
source setup_env.sh
ros2 launch onero_a1_moveit_config dual_moveit.launch.py hardware_mode:=sim \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.dual_desktop.yaml"
```

```bash
# 真机服务
source setup_env.sh
ros2 launch onero_a1_moveit_config dual_moveit.launch.py \
  hardware_mode:=real \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.dual_desktop.yaml"
```

```bash
# 另一终端：运行示例前加载环境
source setup_env.sh
```

### 2. 基础示例：使用 --arm 选臂

#### 控制单侧

```bash
# 第 1 关节抬臂约 11°
ros2 run onero_a1_examples joint_goal --arm left --delta -0.2 \
  --velocity 0.5 --acceleration 0.5 --execute

# 法兰竖直向上走 5 cm
ros2 run onero_a1_examples cartesian_path --arm left \
  --shape line --dx 0 --dy 0 --dz 0.05 \
  --velocity 0.5 --acceleration 0.5 --execute

# 打开单侧夹爪
ros2 run onero_a1_examples gripper --arm left --opening 1 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 起始位置记录与返回、命名姿态和碰撞查询

```bash
# 同时记录两臂当前位置，不产生运动
ros2 run onero_a1_examples record_startup --arm both
```

```bash
# 两臂回到已记录的起始位置
ros2 run onero_a1_examples return_startup --arm both \
  --velocity 0.5 --acceleration 0.5 --execute

# 两臂切换到各自的 ready 姿态
ros2 run onero_a1_examples named_pose --arm both --target ready --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute

# 查询统一场景中的当前状态或命名姿态，不产生运动
ros2 run onero_a1_examples collision_check --arm both
ros2 run onero_a1_examples collision_check --arm both --target ready
ros2 run onero_a1_examples collision_check --arm both --target home
```

```bash
# 持续显示两臂与统一场景的接触点；Ctrl+C 结束
ros2 run onero_a1_examples collision_contacts --arm both

# 仿真演示：在右臂法兰处放临时球体，也可改为 --arm left / right
ros2 run onero_a1_examples collision_contacts --arm both --demo-obstacle
```

#### 零重力拖动（仅 real）

```bash
# 左臂拖动，右臂继续保持
ros2 run onero_a1_examples freedrive --arm left --damping 0.3 --execute

# 右臂拖动，左臂继续保持
ros2 run onero_a1_examples freedrive --arm right --damping 0.3 --execute

# 同时进入双臂拖动
ros2 run onero_a1_examples freedrive --arm both --damping 0.3 --execute
```

### 3. 双臂运动与抓放

#### 双臂联合运动

```bash
ros2 run onero_a1_examples dual_joint_goal \
  --joint-index 1 --left-delta -0.2 --right-delta 0.2 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 双臂同时沿 YZ 平面画正方形

```bash
ros2 run onero_a1_examples dual_cartesian_square --size 0.10 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 双臂关节角度误差测量

先预览一对关节的往返轨迹，确认后执行；执行时自动保存运动曲线、误差曲线和 CSV。

```bash
ros2 run onero_a1_examples dual_joint_tracking --joint-index 1

ros2 run onero_a1_examples dual_joint_tracking --joint-index 1 --execute

# 全部七对关节，每组重复三次
ros2 run onero_a1_examples dual_joint_tracking --repeats 3 --execute
```

默认幅度 `0.1` rad、每段移动至少 `3` s、停点保持 `1` s。参数与结果说明见[误差测量](README.md#关节角度与轨迹误差测量)。

#### 三个抓放示例的轨迹误差采集

完成各示例要求的预览和实物摆放后，在原执行命令中加入 `--record-tracking`：

```bash
ros2 run onero_a1_examples arm_gripper_task --execute --record-tracking

ros2 run onero_a1_examples fixed_pick_place --execute --record-tracking

ros2 run onero_a1_examples mtc_pick_place --mass 0.3 --execute --record-tracking
```

沿用本次预览时的配置与运动参数，`--mass` 按实物填写。
默认输出到 `.logs/joint_tracking/`；也可在命令中指定 `--tracking-output .logs/joint_tracking/run01`，目录须尚不存在。

#### 双夹爪开合

```bash
# 左全开、右半开
ros2 run onero_a1_examples dual_gripper --left-opening 1 --right-opening 0.5 \
  --velocity 0.5 --acceleration 0.5 --execute

# 左全闭、右全开
ros2 run onero_a1_examples dual_gripper --left-opening 0 --right-opening 1 \
  --velocity 0.5 --acceleration 0.5 --execute

# 两侧全闭
ros2 run onero_a1_examples dual_gripper --left-opening 0 --right-opening 0 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 手臂与夹爪联合任务

```bash
# 先生成瓶子坐标并预览
ros2 run onero_a1_examples arm_gripper_task --velocity 0.5 --acceleration 0.5

# 按坐标在左右各摆一个水瓶后执行
ros2 run onero_a1_examples arm_gripper_task --velocity 0.5 --acceleration 0.5 --execute
```

#### 固定位置抓取放置

默认由右臂 A→B、左臂 B→C 完成单程接力：

```bash
# 先预览单程接力
ros2 run onero_a1_examples fixed_pick_place --velocity 0.5 --acceleration 0.5

# 在 A 点摆一个水瓶、清空 B/C 点后执行
ros2 run onero_a1_examples fixed_pick_place --velocity 0.5 --acceleration 0.5 --execute
```

需要把水瓶送回 A 点时，先在预览命令中加 `--round-trip`，预览完成后使用相同参数加 `--execute` 执行。点位调整见[固定位置抓取放置](README.md#固定位置抓取放置)。

#### MTC 抓取放置

```bash
# 预览右 A→B、左 B→C；在 RViz MTC Tasks 中选择方案，Ctrl+C 结束
ros2 run onero_a1_examples mtc_pick_place --velocity 0.3 --acceleration 0.3

# 在 A 点摆瓶、清空 B/C 点后执行；--mass 按实物填写
ros2 run onero_a1_examples mtc_pick_place --mass 0.3 \
  --velocity 0.3 --acceleration 0.3 --execute
```

回程加 `--round-trip`；平滑默认关闭，开启加 `--smoothing ruckig`。C 点见[任务配置](config/mtc_pick_place.yaml)，参数见 [README](README.md#mtc-抓取放置)。

#### 双臂规划器选择

```bash
ros2 run onero_a1_examples dual_joint_goal --pipeline ompl --planner RRTstar \
  --velocity 0.5 --acceleration 0.5

ros2 run onero_a1_examples dual_joint_goal --pipeline chomp \
  --velocity 0.5 --acceleration 0.5

ros2 run onero_a1_examples dual_joint_goal --pipeline stomp \
  --velocity 0.5 --acceleration 0.5
```

### 4. Servo 键盘遥操作

```bash
# 模拟服务（与真机服务二选一）
ros2 launch onero_a1_moveit_config dual_moveit.launch.py hardware_mode:=sim enable_servo:=true

# 真机服务：加载与现场一致的桌面模型
ros2 launch onero_a1_moveit_config dual_moveit.launch.py hardware_mode:=real enable_servo:=true \
  servo_debug:=true \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.dual_desktop.yaml"
```

```bash
# 另一终端：检查并打开遥操作窗口
source setup_env.sh
ros2 run onero_a1_examples servo_keyboard
ros2 run onero_a1_examples servo_keyboard --linear-speed 0.2 --angular-speed 0.5 --execute
```

### 5. 双夹爪标定

```bash
ros2 service call /left_gripper_controller/calibrate std_srvs/srv/Trigger '{}'
ros2 service call /right_gripper_controller/calibrate std_srvs/srv/Trigger '{}'
```
