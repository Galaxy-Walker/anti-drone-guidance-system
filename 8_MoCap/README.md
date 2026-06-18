# 8_MoCap — 室内动捕悬停

在 GPS 拒止环境下，通过外部运动捕捉系统（如 OptiTrack、Vicon）为 PX4 提供高精度（亚厘米级）位置估计，替代 GPS 定位。

## 包说明

`src/px4_mocap_hover` 是该模块唯一的 ROS 2 Python 包。它将 VRPN 发布的 `geometry_msgs/PoseStamped` 动捕位姿转换为 PX4 1.15.4 使用的 NED 坐标约定，并通过 Offboard 模式自动执行起飞、悬停和降落任务。

转换和任务控制分别由两个节点完成：

- **`mocap_bridge`**：只将动捕位姿转换后发布给 PX4，不切换飞行模式、不解锁、不发送控制设定值。
- **`mocap_hover`**：主任务节点，订阅 `mocap_bridge` 输出的 `VehicleOdometry`，检查动捕数据是否连续有效；目标点生成和到达判断使用 PX4 融合后的 `VehicleLocalPosition`。

> 该节点忽略动捕姿态，固定发布 PX4 单位四元数 `(w, x, y, z) = (1, 0, 0, 0)`。PX4 必须配置为只融合外部位置，不得融合该节点发布的姿态或航向。

当前场地的动捕轴定义为 `x = North`、`y = -Down`、`z = East`，因此 `mocap_bridge` 使用以下位置映射：

```text
[x_ned, y_ned, z_ned] = [x_mocap, z_mocap, -y_mocap]
```

即：动捕 X 正 → PX4 North，动捕 Z 正 → PX4 East，动捕 Y 正 → 上方。

## 命令速查

以下命令默认在 `/home/nvidia/ws_ros2` 工作空间中执行。每次打开新终端都需要先加载 ROS 2 和当前工作空间环境：

```bash
cd /home/nvidia/ws_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 启动 DDS Agent

**方式一：串口连接（实机 CH341 USB）**

```bash
# 注意：不要过快启动 DDS，等飞控完全就绪后再执行
sudo MicroXRCEAgent serial -D /dev/ttyCH341USB0 -b 460800
```

**方式二：UDP 连接（SITL 仿真）**

```bash
MicroXRCEAgent udp4 -p 8888
```

### 数据验证（预飞检查）

启动动捕中转节点后，逐条验证数据链路是否正常：

```bash
# 1. 原始动捕数据（VRPN → ROS 2）
ros2 topic echo /vrpn_mocap/RigidBody/pose

# 2. 中转节点输出（动捕 → PX4 NED）
ros2 topic echo /fmu/in/vehicle_visual_odometry

# 3. PX4 融合估计（EKF2 输出）
ros2 topic echo /fmu/out/vehicle_local_position
```

三条链路都应有持续数据输出且位置值合理，再继续后续步骤。

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

# 仅测试解锁，默认解锁 3 秒后自动上锁；测试前仍须拆除螺旋桨
ros2 launch px4_mocap_hover mocap_arm_test.launch.py
```

### 话题与状态检查

```bash
# 检查 VRPN 原始位姿及更新频率
ros2 topic echo --once /vrpn_mocap/RigidBody/pose
ros2 topic hz /vrpn_mocap/RigidBody/pose

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
ros2 node info /px4_mocap_arm_test

# 查看节点参数（只对当前正在运行的节点执行）
ros2 param list /px4_mocap_bridge
ros2 param list /px4_mocap_hover
ros2 param list /px4_mocap_arm_test
ros2 param get /px4_mocap_hover takeoff_height
```

所有节点可使用 `Ctrl-C` 停止。自动悬停任务会主动切换模式并解锁；首次联调和解锁测试必须拆除螺旋桨，并准备可用的遥控接管或急停手段。

## 默认任务流程

1. 等待连续 20 帧有效动捕数据、有效 PX4 本地位置、PX4 解锁前检查通过，并确认飞行器为未解锁的多旋翼。
2. 要求 PX4 本地位置估计在 `prearm_position_tolerance` 范围内连续稳定 `prearm_stable_duration`（默认 10 s）；期间位置跳变、数据失效或超时都会重新计时。
3. 在预发送 Offboard 设定值、切入 Offboard 和请求解锁期间持续复核上述条件，只有位置估计仍然稳定才会发送解锁命令。
4. 记录当前 PX4 本地水平位置和航向，并将目标高度设置为当前位置上方 1.0 m。
5. 预发送 Offboard 设定值，然后自动切入 Offboard 并解锁。
6. 起飞至目标点，在 0.15 m 容差内稳定 1.0 s 后悬停 10 s。
7. 保持发送当前 Offboard 设定值并周期重发降落命令，直到 PX4 报告已进入降落模式；随后在 PX4 报告已上锁后退出。如果 PX4 意外离开降落模式，节点会恢复发送降落命令。

## 坐标系定义

当前场地的动捕坐标系和 PX4 NED 坐标系均为右手坐标系：

| 坐标系 | 用途 | +X | +Y | +Z |
| --- | --- | --- | --- | --- |
| 动捕场地坐标 | VRPN 输入的世界坐标系 | North，北 | Up，上 | East，东 |
| NED | PX4 使用的世界坐标系 | North，北 | East，东 | Down，下 |

这里的"东/北"表示动捕场地中定义的方向，不一定对应真实地理东/北。桥接节点不使用 `PoseStamped.orientation`，因此不会转换动捕刚体姿态或机体系。

## 坐标转换流程

每收到一帧 VRPN `PoseStamped`，节点按以下流程处理：

1. 读取动捕场地坐标系中的位置 `(x, y, z)`，并检查三个位置分量是否为有限值。
2. 按 `(x, y, z) -> (x, z, -y)` 将位置映射到 PX4 NED 坐标系。
3. 忽略输入姿态，并将输出姿态固定为 PX4 单位四元数 `(w, x, y, z) = (1, 0, 0, 0)`。
4. 发布 `px4_msgs/VehicleOdometry` 到 EKF2 使用的 `/fmu/in/vehicle_visual_odometry`：
   - `pose_frame = POSE_FRAME_NED`
   - `position` 使用 NED，单位为米
   - `q` 使用固定单位四元数
   - 速度和角速度未由动捕提供，因此设置为未知值

## PX4 配置

飞行前需要配置 EKF2 只融合外部视觉/动捕位置，不得融合该节点发布的姿态、航向或速度。在 PX4 1.15 中，相关参数包括 `EKF2_EV_CTRL`、`EKF2_EV_DELAY`，以及在使用外部高度作为高度基准时的 `EKF2_HGT_REF`。只应启用节点实际提供的有效测量。

还应通过 `COM_OF_LOSS_T` 和 `COM_OBL_RC_ACT` 配置 Offboard 丢失后的行为。如果动捕数据超过 `mocap_timeout` 未更新，节点会立即停止发送 Offboard 心跳和设定值，任务也不会自动重新开始；随后由 PX4 执行已配置的 Offboard 丢失动作。

本包使用以下 PX4 1.15 DDS 话题：

- `/fmu/in/vehicle_visual_odometry`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_command_ack`

## 运行说明与参数

独立转换节点默认订阅 `/vrpn_mocap/RigidBody/pose`，并将转换后的 `VehicleOdometry` 发布到 `/fmu/in/vehicle_visual_odometry`。虽然输入来自动捕，PX4 1.15 的 EKF2 只订阅内部 `vehicle_visual_odometry` 话题，而不订阅 `vehicle_mocap_odometry`。该节点仅检查和转换位置，按 `(x, y, z) -> (x, z, -y)` 映射到 NED，忽略输入四元数并固定发布单位四元数。可以直接检查输出：

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

该测试节点执行与主节点相同的动捕连续帧、PX4 本地位置、解锁前检查和机型检查，并只允许在 `MANUAL`、`ALTCTL`、`POSCTL`、`ACRO` 或 `STAB` 等非自动模式下解锁。它不会切换 Offboard，也不会发布 Offboard 心跳或轨迹设定值。PX4 报告成功解锁后，节点默认等待 3 秒并自动发送上锁命令；动捕超时、飞行模式变得不安全或终端收到 `Ctrl-C` 时也会请求上锁。即使该节点不发送起飞命令，测试解锁前仍必须拆除螺旋桨。

需要时可通过命令行覆盖参数：

```bash
ros2 run px4_mocap_hover mocap_hover --ros-args \
  -p mocap_topic:=/fmu/in/vehicle_visual_odometry \
  -p takeoff_height:=1.0 \
  -p hover_duration:=10.0
```

### 主要参数

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

主节点日志会输出任务状态变化、已发送命令、PX4 命令确认、中转节点输出的 NED 动捕位置、PX4 当前 NED 位置以及当前目标点。
