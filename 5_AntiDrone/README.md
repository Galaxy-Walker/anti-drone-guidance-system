# 5_AntiDrone — PX4 Offboard 状态机与 PN 导引

`5_AntiDrone/src/anti_drone_guidance` 是当前 PX4 闭环拦截主实现。它把 PX4 Offboard 控制流程、安全监控、目标源和 PN 导引算法整合为一个 ROS 2 包。

## 目录结构

```text
5_AntiDrone/
└── src/anti_drone_guidance/
    ├── anti_drone_guidance_node.py  # 状态机、Offboard 控制、导引节点
    ├── pn_guidance_core.py          # 与 ROS/PX4 解耦的 3D PN 核心
    ├── target_provider.py           # 虚拟目标轨迹与目标源抽象
    ├── gazebo_evaluator.py          # Gazebo 闭环评估器
    ├── config/pn_guidance_params.yaml
    ├── launch/pn_guidance_launch.py
    └── test/                        # 单元测试
```

## 状态机

主飞行状态机位于 `anti_drone_guidance_node.py`：

```text
INIT -> REQUESTING_OFFBOARD -> ARMING -> OFFBOARD_ACTIVE -> RETURN_REQUESTED / ERROR
```

其中 `OFFBOARD_ACTIVE` 内部还有导引子阶段：

```text
TAKEOFF -> GUIDANCE -> LAND
```

各状态职责：

- `INIT`：持续发布 Offboard 心跳和初始 setpoint，满足 PX4 切入 Offboard 前的预热要求。
- `REQUESTING_OFFBOARD`：发送 Offboard/Arm 请求，并通过 `VehicleControlMode` 和 `VehicleStatus` 确认 Offboard 是否真正生效。
- `ARMING`：等待解锁确认，超时则进入 `ERROR`。
- `OFFBOARD_ACTIVE`：执行起飞、导引、拦截判定和降落流程；非仿真模式下会持续做安全监控。
- `RETURN_REQUESTED`：请求 PX4 执行 RTL/返航。
- `ERROR`：处理 Offboard 切入失败、解锁失败、意外退出导航模式、意外上锁等异常。

状态机的重点不是只"发送控制量"，而是先确认 PX4 真的进入 Offboard 并解锁，再执行导引；同时保留电池、模式、上锁状态、阶段超时等安全联动。

## 3D PN 导引

PN 核心位于 `pn_guidance_core.py`，与 ROS 2 和 PX4 通信解耦。导引输入是追踪机和目标的状态：

- `position`：三维位置
- `velocity`：三维速度
- `speed`：速度模长

核心计算流程：

1. 计算相对位置 `R_vec = target - tracker` 和相对速度 `V_rel`。
2. 计算距离 `R_mag`、视线角速率 `Omega = cross(R_vec, V_rel) / |R|^2`。
3. 计算接近速度 `Vc = -dot(R_vec, V_rel) / |R|`。
4. 通过速度策略得到期望速度大小。
5. 使用三维 PN 横向加速度 `a_c = N * Vc_eff * cross(Omega, R_unit)`。
6. 将 PN 方向与直接指向目标的方向混合，避免初始远离或速度过低时越飞越远。
7. 输出 PX4 可用的位置 setpoint 和速度 setpoint。

当前速度策略：

- `adaptive`：根据距离、接近速度和视线角速率动态调整追踪速度。
- `pursuit`：退化为直接追踪目标当前位置，便于和 PN 行为对照。

导引节点主要发布：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

导引节点主要订阅：

- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_control_mode`
- `/fmu/out/battery_status`

位置和状态话题同时兼容带 `_v1` 和不带 `_v1` 的命名，以适配不同 PX4 与 `px4_msgs` 组合。

## 目标源与评估器

`target_provider.py` 将目标数据来源和导引算法解耦。内置虚拟目标轨迹包括：

- `static`：静止悬停目标。
- `line`：匀速直线运动目标。
- `circle`：等高圆周运动目标。
- `circle_altitude`：水平圆周叠加正弦高度变化目标。

`gazebo_evaluator.py` 用于 PX4 Gazebo 闭环评估。它不会自动启动 PX4/Gazebo/uXRCE-DDS，而是在外部仿真环境就绪后，自动运行 case、启动导引节点、记录 rosbag，并输出 CSV 指标。

默认评估矩阵包括：

- 目标轨迹：`static`、`line`、`circle`、`circle_altitude`
- 追踪机最大速度：`10`、`20`、`30` m/s
- 目标初始航向：`0`、`90`、`180`、`270` deg，其中 `static` 只运行一个航向

## 快速开始

以下流程用于运行 PX4 SITL 仿真并启动导引节点。

### 1. 启动 QGroundControl

先打开 QGroundControl，用于查看飞行状态、连接情况和位置变化。

### 2. 启动 PX4 SITL

在新的终端中运行：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### 3. 启动 Micro XRCE-DDS Agent

在新的终端中运行：

```bash
MicroXRCEAgent udp4 -p 8888
```

### 4. 构建 ROS 2 工作空间

```bash
cd ~/Project/Anti-Drone/code/5_AntiDrone
colcon build
```

### 5. 加载工作空间环境

```bash
cd ~/Project/Anti-Drone/code/5_AntiDrone
source install/setup.bash
```

如果使用 `zsh`：

```zsh
cd ~/Project/Anti-Drone/code/5_AntiDrone
source install/setup.zsh
```

### 6. 启动比例导引节点

```bash
ros2 launch anti_drone_guidance pn_guidance_launch.py
```

## 参数配置

默认参数文件：

```text
5_AntiDrone/src/anti_drone_guidance/config/pn_guidance_params.yaml
```

参数按功能分组：

- `guidance`：PN 比例系数 `N`、速度上下限、速度策略。
- `flight`：起飞高度、拦截判定半径、控制频率。
- `target`：目标源类型、虚拟目标运动模式、初始位置、圆周/直线/高度变化轨迹参数。
- `evaluation`：评估 case id 与评估话题开关，普通运行默认关闭。
- `debug`：日志输出间隔和详细日志开关。

指定自定义配置文件：

```bash
ros2 launch anti_drone_guidance pn_guidance_launch.py config_file:=/path/to/custom.yaml
```

## Gazebo 闭环评估

评估器不会自动启动 PX4 Gazebo 或 Micro XRCE-DDS Agent。运行评估前，请先在外部终端启动：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

```bash
MicroXRCEAgent udp4 -p 8888
```

然后在已 source `5_AntiDrone` 工作空间的终端中运行：

```bash
cd ~/Project/Anti-Drone/code/5_AntiDrone
ros2 run anti_drone_guidance gazebo_evaluator
```

常用参数示例：

```bash
ros2 run anti_drone_guidance gazebo_evaluator \
  --motions static line \
  --speed-max 10 20 \
  --headings 0 90 180 270 \
  --output results.csv \
  --bag-dir bags
```

如果使用外部脚本负责重启 PX4/Gazebo，并且不希望每个 case 后等待手动确认，可以增加：

```bash
ros2 run anti_drone_guidance gazebo_evaluator --no-restart-prompt
```

## 已知前提

运行 PX4/ROS 2 相关部分前，请确认：

- PX4 SITL 已正常启动。
- QGroundControl 已成功连接 PX4。
- Micro XRCE-DDS Agent 正在监听 `udp4:8888`。
- ROS 2 环境与对应工作空间环境已经正确 source。
- `px4_msgs` 与当前 PX4/uXRCE-DDS 输出的话题版本匹配。

如果其中任一环节未完成，ROS 2 节点可能无法正常与 PX4 建立通信。
