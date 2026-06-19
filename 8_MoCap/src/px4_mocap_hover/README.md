# PX4 动捕悬停

该 ROS 2 Python 包将 VRPN 发布的 `geometry_msgs/PoseStamped` 动捕位姿转换为
PX4 1.15.4 使用的坐标约定，并通过 Offboard 模式自动执行起飞、悬停、圆轨迹和降落任务。
转换由 `mocap_bridge` 完成，悬停与圆轨迹任务分别由 `mocap_hover` 和 `mocap_circle` 完成。

包内还提供独立的 `mocap_tracker` 追踪节点。它自动起飞到相对起点 1.2 m 的高度，
保持该高度，并使用速度设定值在水平面追踪
`/vrpn_mocap/RigidBody_002/pose` 对应的目标机。目标机短时不可见时，追踪机保持
Offboard 心跳、将水平速度平滑减至零并定高悬停；目标连续恢复 5 帧后自动继续追踪。

`mocap_bridge` 只将动捕位姿转换后发布给 PX4，不会切换飞行模式、解锁或发送控制设定值。
`mocap_hover` 主节点订阅该节点输出的 `VehicleOdometry`，用它检查动捕数据是否连续、有效和
超时；目标点生成和到达判断仍使用 PX4 融合后的 `VehicleLocalPosition`。
该节点忽略动捕姿态，固定发布 PX4 `(w, x, y, z) = (1, 0, 0, 0)` 单位四元数；
因此 PX4 必须配置为只融合外部位置，不得融合该节点发布的姿态或航向。
当前场地的动捕轴定义为 `x = North`、`y = -Down`、`z = East`，所以
`mocap_bridge` 使用以下位置映射：

```text
[x_ned, y_ned, z_ned] = [x_mocap, z_mocap, -y_mocap]
```

即动捕 X 正方向对应 PX4 North，动捕 Z 正方向对应 PX4 East，动捕 Y 正方向对应上方。

## 命令速查

以下命令默认在 `/home/nvidia/ws_ros2` 工作空间中执行。每次打开新终端都需要先加载 ROS 2
和当前工作空间环境：

```bash
cd /home/nvidia/ws_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 构建与测试

```bash
# 构建本包
colcon build --packages-select px4_mocap_hover
source install/setup.bash

# 运行本包测试并查看详细结果
colcon test --packages-select px4_mocap_hover
colcon test-result --verbose
```

### 启动节点

先在终端 1 启动动捕中转，并在执行后续任务期间保持运行：

```bash
ros2 launch px4_mocap_hover mocap_bridge.launch.py
```

然后在已加载工作空间环境的终端 2 中，按测试目的选择一个命令：

```bash
# 自动切入 Offboard，解锁、起飞、悬停并降落
ros2 launch px4_mocap_hover mocap_hover.launch.py

# 起飞悬停 3 秒，以悬停点为最东端逆时针飞一圈后降落
ros2 launch px4_mocap_hover mocap_circle.launch.py

# 自动起飞并在固定高度水平追踪目标机
ros2 launch px4_mocap_hover mocap_tracker.launch.py

# 将目标机绝对 NED 轨迹逐帧记录到 CSV
ros2 launch px4_mocap_hover mocap_trajectory_recorder.launch.py

# 以固定高度和 60 Hz 逐行回放绝对 NED CSV 水平轨迹
ros2 launch px4_mocap_hover mocap_trajectory_replay.launch.py

# 仅测试解锁，默认解锁 3 秒后自动上锁；测试前仍须拆除螺旋桨
ros2 launch px4_mocap_hover mocap_arm_test.launch.py
```

### 话题与状态检查

```bash
# 检查 VRPN 原始位姿及更新频率
ros2 topic echo --once /vrpn_mocap/RigidBody/pose
ros2 topic hz /vrpn_mocap/RigidBody/pose

# 检查目标机 VRPN 位姿及更新频率
ros2 topic echo --once /vrpn_mocap/RigidBody_002/pose
ros2 topic hz /vrpn_mocap/RigidBody_002/pose

# 检查中转后的 NED 动捕数据及更新频率
ros2 topic echo --once /fmu/in/vehicle_visual_odometry
ros2 topic hz /fmu/in/vehicle_visual_odometry

# 检查 PX4 本地位置、飞行器状态和命令确认
ros2 topic echo --once /fmu/out/vehicle_local_position
ros2 topic echo --once /fmu/out/vehicle_status
ros2 topic echo /fmu/out/vehicle_command_ack
```

### 节点与参数检查

```bash
# 查看节点连接关系
ros2 node info /px4_mocap_bridge
ros2 node info /px4_mocap_hover
ros2 node info /px4_mocap_circle
ros2 node info /px4_mocap_tracker
ros2 node info /mocap_trajectory_recorder
ros2 node info /px4_mocap_trajectory_replay
ros2 node info /px4_mocap_arm_test

# 查看节点参数；只对当前正在运行的节点执行
ros2 param list /px4_mocap_bridge
ros2 param list /px4_mocap_hover
ros2 param list /px4_mocap_circle
ros2 param list /px4_mocap_tracker
ros2 param list /mocap_trajectory_recorder
ros2 param list /px4_mocap_trajectory_replay
ros2 param list /px4_mocap_arm_test
ros2 param get /px4_mocap_hover takeoff_height
```

所有节点可使用 `Ctrl-C` 停止。自动悬停任务会主动切换模式并解锁；首次联调和解锁测试必须
拆除螺旋桨，并准备可用的遥控接管或急停手段。

默认任务流程：

1. 等待连续 20 帧有效动捕数据、有效 PX4 本地位置、PX4 解锁前检查通过，并确认飞行器为未解锁的多旋翼。
2. 要求 PX4 本地位置估计在 `prearm_position_tolerance` 范围内连续稳定
   `prearm_stable_duration`（默认 10 s）；期间位置跳变、数据失效或超时都会重新计时。
3. 在预发送 Offboard 设定值、切入 Offboard 和请求解锁期间持续复核上述条件，只有位置估计
   仍然稳定才会发送解锁命令。
4. 记录当前 PX4 本地水平位置和航向，并将目标高度设置为当前位置上方 1.0 m。
5. 预发送 Offboard 设定值，然后自动切入 Offboard 并解锁。
6. 起飞至目标点，在 0.15 m 容差内稳定 1.0 s 后悬停 10 s。
7. 保持发送当前 Offboard 设定值并周期重发降落命令，直到 PX4 报告已进入降落模式；随后在
   PX4 报告已上锁后退出。如果 PX4 意外离开降落模式，节点会恢复发送降落命令。


## 坐标系定义

当前场地的动捕坐标系和 PX4 NED 坐标系均为右手坐标系：

| 坐标系 | 用途 | +X | +Y | +Z |
| --- | --- | --- | --- | --- |
| 动捕场地坐标 | VRPN 输入的世界坐标系 | North，北 | Up，上 | East，东 |
| NED | PX4 使用的世界坐标系 | North，北 | East，东 | Down，下 |

这里的“东/北”表示动捕场地中定义的方向，不一定对应真实地理东/北。桥接节点不使用
`PoseStamped.orientation`，因此不会转换动捕刚体姿态或机体系。

## 坐标转换流程

每收到一帧 VRPN `PoseStamped`，节点按以下流程处理：

1. 读取动捕场地坐标系中的位置 `(x, y, z)`，并检查三个位置分量是否为有限值。
2. 按 `(x, y, z) -> (x, z, -y)` 将位置映射到 PX4 NED 坐标系。
3. 忽略输入姿态，并将输出姿态固定为 PX4 `(w, x, y, z) = (1, 0, 0, 0)` 单位四元数。
4. 发布 `px4_msgs/VehicleOdometry` 到 EKF2 使用的
   `/fmu/in/vehicle_visual_odometry`：
   - `pose_frame = POSE_FRAME_NED`
   - `position` 使用 NED，单位为米
   - `q` 使用固定单位四元数
   - 速度和角速度未由动捕提供，因此设置为未知值


## PX4 配置

飞行前需要配置 EKF2 只融合外部视觉/动捕位置，不得融合该节点发布的姿态、航向或速度。
在 PX4 1.15 中，相关参数包括
`EKF2_EV_CTRL`、`EKF2_EV_DELAY`，以及在使用外部高度作为高度基准时的
`EKF2_HGT_REF`。只应启用节点实际提供的有效测量。

还应通过 `COM_OF_LOSS_T` 和 `COM_OBL_RC_ACT` 配置 Offboard 丢失后的行为。如果动捕数据
超过 `mocap_timeout` 未更新，节点会立即停止发送 Offboard 心跳和设定值，任务也不会自动
重新开始；随后由 PX4 执行已配置的 Offboard 丢失动作。

本包使用以下 PX4 1.15 DDS 话题：

- `/fmu/in/vehicle_visual_odometry`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_command_ack`

## 运行说明与参数

独立转换节点默认订阅 `/vrpn_mocap/RigidBody/pose`，并将转换后的
`VehicleOdometry` 发布到 `/fmu/in/vehicle_visual_odometry`。虽然输入来自动捕，
PX4 1.15 的 EKF2 只订阅内部 `vehicle_visual_odometry` 话题，而不订阅
`vehicle_mocap_odometry`。该节点仅检查和转换位置，
按 `(x, y, z) -> (x, z, -y)` 映射到 NED，忽略输入四元数并固定发布单位四元数
`(w, x, y, z) = (1, 0, 0, 0)`。可以直接检查输出：

```bash
ros2 topic echo --once /fmu/in/vehicle_visual_odometry
```

可通过参数覆盖输入、输出话题和测量方差：

```bash
ros2 run px4_mocap_hover mocap_bridge --ros-args \
  -p mocap_topic:=/vrpn_mocap/RigidBody/pose \
  -p px4_topic:=/fmu/in/vehicle_visual_odometry \
  -p position_variance:=0.0025 \
  -p orientation_variance:=0.01 \
  -p quality:=100
```

运行中转节点后，再启动自动起飞、悬停和降落任务：

```bash
ros2 launch px4_mocap_hover mocap_hover.launch.py
```

只测试解锁、不执行起飞时，保持中转节点运行并启动：

```bash
ros2 launch px4_mocap_hover mocap_arm_test.launch.py
```

该测试节点执行与主节点相同的动捕连续帧、PX4 本地位置、解锁前检查和机型检查，并只允许
在 `MANUAL`、`ALTCTL`、`POSCTL`、`ACRO` 或 `STAB` 等非自动模式下解锁。它不会切换
Offboard，也不会发布 Offboard 心跳或轨迹设定值。PX4 报告成功解锁后，节点默认等待 3 秒
并自动发送上锁命令；动捕超时、飞行模式变得不安全或终端收到 `Ctrl-C` 时也会请求上锁。
即使该节点不发送起飞命令，测试解锁前仍必须拆除螺旋桨。

需要时可通过命令行覆盖参数：

```bash
ros2 run px4_mocap_hover mocap_hover --ros-args \
  -p mocap_topic:=/fmu/in/vehicle_visual_odometry \
  -p takeoff_height:=1.0 \
  -p hover_duration:=10.0
```

主要参数：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `mocap_topic` | `/fmu/in/vehicle_visual_odometry` | 中转节点输出的 NED `VehicleOdometry` 话题 |
| `takeoff_height` | `1.0` | 相对当前高度向上起飞的距离，单位为米 |
| `hover_duration` | `10.0` | 到达目标后悬停时间，单位为秒 |
| `mocap_timeout` | `0.5` | 动捕数据超时阈值，单位为秒 |
| `local_position_timeout` | `0.5` | PX4 本地位置估计超时阈值，单位为秒 |
| `control_rate` | `10.0` | Offboard 控制循环频率，单位为 Hz，必须大于 2 Hz |
| `log_rate` | `1.0` | 状态日志频率，单位为 Hz |
| `prearm_stable_duration` | `10.0` | 解锁前位置估计必须连续稳定的时间，单位为秒 |
| `prearm_position_tolerance` | `0.15` | 解锁前稳定窗口内允许的位置变化范围，单位为米 |
| `position_tolerance` | `0.15` | 判定到达目标的位置容差，单位为米 |
| `stable_duration` | `1.0` | 在目标容差内持续稳定的时间，单位为秒 |
| `prestream_duration` | `1.0` | 请求 Offboard 前预发送设定值的时间，单位为秒 |
| `command_retry_interval` | `1.0` | PX4 命令重试间隔，单位为秒 |
| `required_mocap_samples` | `20` | 开始任务前要求的连续有效动捕帧数 |

主节点日志会输出任务状态变化、已发送命令、PX4 命令确认、中转节点输出的 NED 动捕位置、
PX4 当前 NED 位置以及当前目标点。

## 圆轨迹任务

保持 `mocap_bridge` 运行后启动：

```bash
ros2 launch px4_mocap_hover mocap_circle.launch.py
```

该节点复用悬停任务的动捕稳定检查、Offboard 切换、解锁和降落保护。默认相对起飞
`1.0 m`，到达目标并稳定后悬停 `3.0 s`。悬停点作为圆的最东端，圆心位于其西侧
`0.5 m`；飞行器从该点先向北运动，以俯视逆时针方向在 `10.0 s` 内完成一圈，同时保持
起飞时航向和高度。轨迹结束后节点发送 PX4 `NAV_LAND` 命令，并等待 PX4 自动上锁后退出。

圆轨迹参数可通过命令行覆盖：

```bash
ros2 run px4_mocap_hover mocap_circle --ros-args \
  -p takeoff_height:=1.0 \
  -p hover_duration:=3.0 \
  -p circle_radius:=0.5 \
  -p circle_duration:=10.0
```

其中 `circle_radius` 单位为米，`circle_duration` 单位为秒。飞行区域应至少为起飞点西侧
`1.0 m`、南北两侧各 `0.5 m` 的圆形轨迹预留额外安全空间。

## 水平追踪任务

启动追踪任务前，先保持现有追踪机动捕桥接节点运行：

```bash
ros2 launch px4_mocap_hover mocap_bridge.launch.py
```

确认追踪机 `RigidBody` 和目标机 `RigidBody_002` 的身份、更新频率和坐标方向正确后，
在另一个已加载工作空间环境的终端启动：

```bash
ros2 launch px4_mocap_hover mocap_tracker.launch.py
```

默认流程如下：

1. 只等待追踪机自身动捕、PX4 本地位置和解锁前检查满足要求，不要求目标机预先可见。
2. 记录追踪机当前 PX4 本地位置和航向，使用位置设定值自动切入 Offboard、解锁并向上
   起飞 1.2 m。
3. 到达起飞目标并稳定后切换为 NED 速度控制；目标不可见时发送零水平速度并通过垂直
   速度闭环保持起飞高度。
4. 目标动捕连续有效 5 帧后，根据两个刚体在同一动捕世界坐标系中的 NED 水平位置误差
   生成速度指令。目标高度和姿态均不参与控制，追踪机保持起飞时航向。
5. 目标超过 0.5 s 未更新时，按最大水平加速度限制将速度平滑减至零；目标重新连续有效
   5 帧后继续追踪。

追踪节点不会自动降落，也不会在人工切出 Offboard 后重新接管。结束任务时应先使用遥控器
切换到已验证可用的人工模式，确认 PX4 已离开 Offboard，再停止节点。直接停止节点会触发
PX4 的 Offboard-loss 行为，其结果取决于 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 等参数。
自身动捕或 PX4 本地位置失效时，节点同样停止 Offboard 输出并交由 PX4 failsafe 处理。

主要追踪参数：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `self_odometry_topic` | `/fmu/in/vehicle_visual_odometry` | 追踪机桥接后的 NED 动捕里程计 |
| `target_mocap_topic` | `/vrpn_mocap/RigidBody_002/pose` | 目标机原始 VRPN 位姿 |
| `takeoff_height` | `1.2` | 相对起点向上起飞并保持的高度，单位为米 |
| `target_mocap_timeout` | `0.5` | 目标机动捕超时阈值，单位为秒 |
| `target_reacquire_samples` | `5` | 目标恢复后重新追踪所需的连续有效帧数 |
| `xy_kp` | `0.8` | 水平位置误差到速度的比例增益 |
| `xy_deadband` | `0.10` | 水平误差死区半径，单位为米 |
| `max_xy_speed` | `0.5` | 最大水平合速度，单位为 m/s |
| `max_xy_acceleration` | `0.5` | 水平速度指令最大变化率，单位为 m/s² |
| `z_kp` | `1.0` | 定高垂直速度比例增益 |
| `max_z_speed` | `0.3` | 最大垂直速度绝对值，单位为 m/s |

其他解锁前稳定性、控制频率、超时和命令重试参数与 `mocap_hover` 含义一致。例如：

```bash
ros2 run px4_mocap_hover mocap_tracker --ros-args \
  -p target_mocap_topic:=/vrpn_mocap/RigidBody_002/pose \
  -p takeoff_height:=1.2 \
  -p max_xy_speed:=0.5
```

首次联调必须拆除螺旋桨，分别验证坐标转换、目标身份、速度方向、目标丢失悬停和遥控接管；
装桨测试时还必须为两机保留足够的垂直隔离和水平安全空间。默认 XY 完全重合控制并不提供
避碰能力。

## 目标机轨迹记录

轨迹记录节点独立订阅目标机原始 VRPN 位姿，不控制飞行器，也不要求 PX4 或桥接节点运行：

```bash
ros2 launch px4_mocap_hover mocap_trajectory_recorder.launch.py
```

节点收到第一帧有效位姿时才创建 CSV，并将该帧的 `elapsed_s` 记为 `0`。随后逐帧使用
节点接收时的 ROS 时钟记录数据，按 `(x, y, z) -> (x, z, -y)` 转换为场地绝对 NED
坐标。CSV 列为：

```text
elapsed_s,ros_time_ns,north_m,east_m,down_m
```

默认文件名为
`/home/nvidia/ws_ros2/trajectory_csv/target_trajectory_YYYYMMDD_HHMMSS.csv`。若同名文件已经存在，
节点会添加数字后缀，绝不覆盖已有记录。每帧写入后立即刷新；使用 `Ctrl-C` 停止节点并
关闭文件。若整个运行期间没有有效样本，则不会创建空 CSV。

可通过参数修改订阅话题、输出目录和文件名前缀：

```bash
ros2 run px4_mocap_hover mocap_trajectory_recorder --ros-args \
  -p mocap_topic:=/vrpn_mocap/RigidBody_002/pose \
  -p output_directory:=/home/nvidia/ws_ros2/trajectory_csv \
  -p file_prefix:=target_trajectory
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `mocap_topic` | `/vrpn_mocap/RigidBody_002/pose` | 目标机原始 VRPN 位姿 |
| `output_directory` | `/home/nvidia/ws_ros2/trajectory_csv` | CSV 输出目录 |
| `file_prefix` | `target_trajectory` | CSV 文件名前缀 |

## CSV 绝对 NED 轨迹回放

回放前必须确认 PX4 `VehicleLocalPosition` 的水平 NED 原点与 CSV 使用的场地绝对 NED
原点一致。保持追踪机自身的 `mocap_bridge` 运行后启动：

```bash
ros2 launch px4_mocap_hover mocap_trajectory_replay.launch.py
```

执行命令后，launch 会在当前终端明确询问要回放的 CSV 文件路径：

```text
请输入要回放的 CSV 文件路径，然后按 Enter：
> /home/nvidia/ws_ros2/trajectory_csv/target_trajectory_20260619_163716.csv
```

launch 不提供默认 CSV；空路径或不存在的文件会要求重新输入。获得有效路径后，节点会
一次性加载并验证整个 CSV。表头不匹配、没有数据行、列数错误或包含非法数值时，节点直接
启动失败，不会切换模式、解锁或发布飞行设定值。

默认任务先原地垂直起飞至相对起点 `1.2 m`，稳定后悬停 `3.0 s`，再以固定高度飞至 CSV
首行的绝对 `north_m/east_m` 并稳定 `1.0 s`。随后控制循环以 `60 Hz` 每周期严格推进
一行，不使用 `elapsed_s` 调度、不插值、不跳点。CSV 的 `down_m` 仅参与文件有效性检查；
实际高度始终固定为起飞前 PX4 本地 `z - 1.2 m`，航向保持为起飞前航向。全部行发送后
节点执行 `NAV_LAND`，等待 PX4 自动上锁后退出。

例如 `target_trajectory_20260619_163716.csv` 包含 4458 个坐标点，60 Hz 回放约需
`74.3 s`，轨迹水平范围约为 `3.36 m × 3.02 m`。装桨前必须确认所选 CSV 的整个轨迹
区域具有足够净空。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `trajectory_file` | 无 | 要完整加载并回放的 CSV；launch 启动时交互输入 |
| `takeoff_height` | `1.2` | 相对起点向上起飞并保持的高度，单位为米 |
| `hover_duration` | `3.0` | 垂直起飞稳定后的悬停时间，单位为秒 |
| `control_rate` | `60.0` | 回放和 Offboard 控制频率，单位为 Hz |
| `position_tolerance` | `0.15` | 起飞及进入首点的位置容差，单位为米 |
| `stable_duration` | `1.0` | 在目标容差内持续稳定的时间，单位为秒 |

可通过命令行选择其他同格式轨迹：

```bash
ros2 run px4_mocap_hover mocap_trajectory_replay --ros-args \
  -p trajectory_file:=/home/nvidia/ws_ros2/trajectory_csv/target_trajectory.csv \
  -p control_rate:=60.0
```

飞行中若自身动捕超时、PX4 本地位置失效、飞行器意外离开 Offboard 或意外上锁，节点会
停止 Offboard 输出并交由 PX4 failsafe 处理。必须预先验证
`COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 等参数和人工接管方式。
