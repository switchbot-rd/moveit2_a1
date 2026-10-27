# 功能示例运行指令

以下命令在工作区根目录运行。轨迹示例的速度和加速度缩放统一使用 `0.5`。
功能说明、源码及参数见 [Examples 指南](README.md)，启动参数见[服务启动参数](../../README.md#服务启动参数)。

- [单臂运行指令](#单臂运行指令)：右臂或左臂独立运行。
- [双臂运行指令](#双臂运行指令)：单侧控制、双臂联合运动及双夹爪控制。

## 共用约定

- 每个终端先执行 `source setup_env.sh`，自动准备依赖、构建并加载环境。
- 单臂和双臂服务每次选择一套；切换 sim / real 前结束上一套服务。
- 轨迹示例先去掉 `--execute` 规划预览，确认路径后再执行；执行时重新规划。
- 基础运动示例先用 `record_startup` 记录起始位置，完成后返回该位置。
- 相同机械臂配置下，服务重启后沿用已有记录；仅在需要更换起始位置时重新记录。
- 真机从已确认具有运动空间的悬空姿态启动。`ready` / `home` 是模型命名姿态。
- 默认参考系为 `world`：X 向前、Y 向左、Z 向上；位置对应 Link7 法兰。
- `environment_file` 默认留空。以下真机命令显式加载桌面与支架模型，使用前核对现场尺寸。

## 单臂运行指令

下面以右臂列出完整指令，左臂按[参数对照表](#左单臂运行指令)修改即可。
示例自动识别服务启动时选择的手臂，运行时省略 `--arm`。
转角和横向位移按对称展示选取数值；两臂仍使用相同的 world 坐标方向规则。

- [右单臂运行指令](#右单臂运行指令)
- [左单臂参数对照](#左单臂运行指令)

### 右单臂运行指令

#### 1. 启动服务

先加载环境，再选择模拟或真机模式。

```bash
source setup_env.sh
```

模拟模式：

```bash
ros2 launch onero_a1_moveit_config moveit.launch.py hardware_mode:=sim arm:=right
```

真机模式：

```bash
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real arm:=right \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

右臂默认 CAN 为 `can0`，安装方向为 `humanoid_right`。CAN 接口不同时通过 `can_channel` 指定。
需要仅预览时，启动命令添加 `allow_execution:=false`；真机仍会使能并保持实测姿态。

在另一终端加载环境，运行下面各项指令：

```bash
source setup_env.sh
```

#### 2. 记录与返回起始位置

首次使用或需要更换起始位置时，保持手臂静止，记录当前位置；该命令不产生运动：

```bash
ros2 run onero_a1_examples record_startup
```

每项独立运动完成后执行此命令，再进行下一项：

```bash
ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 3. 单关节抬臂

改变第 1 关节约 34°。

```bash
ros2 run onero_a1_examples joint_goal --delta 0.6 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 4. 法兰位置目标

向后 5 cm、向外侧 10 cm、向上 10 cm 到达新位置，多关节共同配合；中途路径可以弯曲。

```bash
ros2 run onero_a1_examples pose_goal \
  --dx -0.05 --dy -0.10 --dz 0.10 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 5. 法兰朝向目标

法兰目标位置保持，绕 world Z 轴旋转约 29°。

```bash
ros2 run onero_a1_examples pose_goal \
  --dx 0 --dy 0 --dz 0 --yaw 0.5 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 6. 笛卡尔直线

法兰竖直向上走 12 cm，并保持朝向。

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape line --dx 0 --dy 0 --dz 0.12 \
  --velocity 0.5 --acceleration 0.5 --execute
```

水平向外侧走 12 cm：

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape line --dx 0 --dy -0.12 --dz 0 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 7. 笛卡尔正方形

先到后方 5 cm、上方 2 cm 的绘制起点，在 YZ 竖直平面画边长 10 cm 的正方形。
按 world 方向依次向左、向上、向右、向下移动，结束于绘制起点。

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape square --plane yz --size 0.10 \
  --dx -0.05 --dy 0 --dz 0.02 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 8. 保持朝向搬运

法兰向后 4 cm、向外侧 10 cm、向上 10 cm 移动，路径朝向偏差限制为 0.05 rad。

```bash
ros2 run onero_a1_examples orientation_constraint \
  --dx -0.04 --dy -0.10 --dz 0.10 \
  --path-angle-tolerance 0.05 --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 9. 障碍物避让

在原路线中间加入直径 6 cm 的临时球体，重新规划避让，结束后清理球体。

```bash
ros2 run onero_a1_examples planning_scene \
  --scene-delta 0.4 --obstacle-radius 0.03 --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 10. 命名姿态

切换到模型中的 `ready` 姿态。模拟模式也可用它准备末端运动起点。

```bash
ros2 run onero_a1_examples named_pose --target ready --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 11. 夹爪标定与开合

空载且完整开合范围无遮挡时标定；需要中止标定或当前夹爪动作时调用停止服务。

```bash
ros2 service call /gripper_controller/calibrate std_srvs/srv/Trigger '{}'

# 需要停止时执行
ros2 service call /gripper_controller/stop std_srvs/srv/Trigger '{}'
```

标定完成后，依次全开、半开、全闭：

```bash
ros2 run onero_a1_examples gripper --opening 1 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples gripper --opening 0.5 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples gripper --opening 0 \
  --velocity 0.5 --acceleration 0.5 --execute
```

速度、加速度缩放作用于夹爪规划轨迹，实物开合由适配层开度斜坡执行。

#### 12. 碰撞查询

只查询，不产生运动。输出中的 `状态有效：True/False` 表示所查询姿态是否有效。

```bash
ros2 run onero_a1_examples collision_check
ros2 run onero_a1_examples collision_check --target ready
ros2 run onero_a1_examples collision_check --target home
```

#### 13. 规划器选择

以下只规划同一关节目标，选择一条运行。支持情况见[规划器选择](README.md#规划器选择)。

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

保持手臂静止并用手托住后进入拖动，轻推手臂到新位置。
停止拖动后按 Ctrl+C，恢复当前位置保持和轨迹控制；松手不会自动退出拖动模式。
该模式不经过 MoveIt 避障规划，重力补偿不完全时可能缓慢漂移。

```bash
# 只检查条件，不改变控制模式
ros2 run onero_a1_examples freedrive

# 进入拖动，Ctrl+C 退出
ros2 run onero_a1_examples freedrive --damping 0.3 --execute

# 也可指定 30 秒后自动退出
ros2 run onero_a1_examples freedrive --damping 0.3 --duration 30 --execute
```

拖动阻尼越大越难拖动；速度与加速度由手动动作决定，不使用轨迹缩放参数。

#### 15. 手臂与夹爪联合任务

需加载桌面环境；模拟启动也要添加 `environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"`。

```bash
# 先生成瓶子坐标并预览
ros2 run onero_a1_examples arm_gripper_task --velocity 0.5 --acceleration 0.5

# 按坐标摆好实体水瓶后执行；默认沿用 SDK 启动补偿
ros2 run onero_a1_examples arm_gripper_task --velocity 0.5 --acceleration 0.5 --execute
```

参数见[手臂与夹爪联合任务](README.md#手臂与夹爪联合任务)。

#### 16. 固定位置抓取放置

需加载桌面环境，模拟启动也要添加 environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"。

默认执行 A→B 松爪退开，再从 B 点重新夹取搬回 A 点；添加 `--one-way` 可只运行单程。

```bash
# 检查完整往返路径并预览
ros2 run onero_a1_examples fixed_pick_place --velocity 0.5 --acceleration 0.5

# 在 A 点摆好水瓶、清空 B 点后执行
ros2 run onero_a1_examples fixed_pick_place --velocity 0.5 --acceleration 0.5 --execute
```

### 左单臂运行指令

沿用右臂运行流程，按下表修改参数；CAN 通道和安装方向省略时会随 `arm:=left` 自动选择。

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
| 固定位置抓取放置 | 默认使用右臂预设 | 添加 `--config PATH`，配置 `arm: left`，并按左臂设置取放坐标和抓取朝向 |
| 其余示例及参数 | 使用上文命令 | 直接沿用，单臂服务自动识别左臂，无需添加 `--arm` |

## 双臂运行指令

“双臂服务”表示同时加载两只手臂；选臂方式由运行的**示例名称**决定：

| 示例 | `--arm` 用法 |
|---|---|
| `joint_goal`、`pose_goal`、`cartesian_path`、`orientation_constraint`、`planning_scene`、`gripper`、`arm_gripper_task`、`fixed_pick_place` | 用 `left` 或 `right` 选择单侧 |
| `named_pose`、`record_startup`、`return_startup`、`collision_check`、`freedrive` | 可选 `left`、`right` 或 `both` |
| `dual_joint_goal`、`dual_cartesian_square`、`dual_gripper` | 自动控制双臂，不传 `--arm` |

### 1. 启动服务

模拟模式：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config dual_moveit.launch.py hardware_mode:=sim
```

真机模式：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config dual_moveit.launch.py \
  hardware_mode:=real \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.dual_desktop.yaml"
```

需要仅预览时，启动命令添加 `allow_execution:=false`。
在另一终端加载环境，运行下面各项指令：

```bash
source setup_env.sh
```

### 2. 基础示例：使用 --arm 选臂

#### 控制单侧

以下以左臂为例。右臂将 `--arm left` 改为 `--arm right`；
第 1 关节镜像抬臂时，同时将 `--delta -0.2` 改为 `--delta 0.2`。
每次选择一项，另一只手臂保持当前目标。

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

这些基础示例指定 `--arm both` 时作用于两臂，也可用 `left` / `right` 单独操作一侧。
首次使用或需要更换起始位置时，保持两臂静止并记录；服务重启后无需重复记录。

```bash
# 同时记录两臂当前位置，不产生运动
ros2 run onero_a1_examples record_startup --arm both
```

每项独立运动结束后，用下面的 `return_startup` 回到两臂已记录的位置，再开始下一项。

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

#### 零重力拖动（仅 real）

选择一侧或两臂进入拖动。保持所选手臂静止并用手托住后进入；停止拖动后按 Ctrl+C 退出并保持当前位置。
手动拖动不经过 MoveIt 避障规划，应自行留出两臂、桌面和支架之间的空间。

```bash
# 左臂拖动，右臂继续保持
ros2 run onero_a1_examples freedrive --arm left --damping 0.3 --execute

# 右臂拖动，左臂继续保持
ros2 run onero_a1_examples freedrive --arm right --damping 0.3 --execute

# 同时进入双臂拖动
ros2 run onero_a1_examples freedrive --arm both --damping 0.3 --execute
```

### 3. 双臂专用示例：dual_* 自动控制双臂

以下三个示例已固定控制双臂，直接运行即可。

#### 双臂联合运动

同时改变左右第 1 关节的目标约 11°，由一条十四关节轨迹协调执行。

```bash
ros2 run onero_a1_examples dual_joint_goal \
  --joint-index 1 --left-delta -0.2 --right-delta 0.2 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 双臂同时沿 YZ 平面画正方形

两侧法兰分别从当前位置出发，先向上 10 cm，再各向外侧 10 cm、向下、回到起点，保持各自朝向。
两臂同时到达每个角点并短暂停顿。整圈路径不可达或存在碰撞时，不执行运动。

```bash
ros2 run onero_a1_examples dual_cartesian_square --size 0.10 \
  --velocity 0.5 --acceleration 0.5 --execute
```

#### 双夹爪开合

分别设置两侧开度，观察两个夹爪执行不同目标：

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

#### 双臂规划器选择

以下只规划同一组双臂关节目标，每次选择一条运行：

```bash
ros2 run onero_a1_examples dual_joint_goal --pipeline ompl --planner RRTstar \
  --velocity 0.5 --acceleration 0.5

ros2 run onero_a1_examples dual_joint_goal --pipeline chomp \
  --velocity 0.5 --acceleration 0.5

ros2 run onero_a1_examples dual_joint_goal --pipeline stomp \
  --velocity 0.5 --acceleration 0.5
```

### 4. 双夹爪标定

空载且完整开合范围无遮挡时，分别标定左右夹爪：

```bash
ros2 service call /left_gripper_controller/calibrate std_srvs/srv/Trigger '{}'
ros2 service call /right_gripper_controller/calibrate std_srvs/srv/Trigger '{}'
```
