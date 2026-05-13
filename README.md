# Anti-Drone 比例导引拦截与多算法仿真系统

基于 ROS 2、PX4 Offboard、Gazebo 和比例导引算法（Proportional Navigation, PN）的无人机追踪/拦截项目。

本仓库记录了从算法验证、飞控接口联调、ROS 2/PX4 闭环控制，到多导引算法对比仿真的完整演进。当前最值得关注的是：

- `5_AntiDrone/`：当前主要 PX4 Offboard 闭环实现，重点是状态机、安全控制和 3D PN 导引。
- `6_Simulation/`：当前主要算法对比平台，重点是 Direct pursuit、PN、PN+FOV、PN+FOV+CBF、PN+FOV+NMPC、PN+FOV+MPPI 的统一评估。

> 本项目主要用于学习、实验和交流。
> 代码由作者在学习过程中逐步完成，部分内容借助 AI 辅助生成，因此实现方式、工程规范和细节设计仍可能存在不足，欢迎提出问题和改进建议。

## 仓库结构

```text
.
├── 1_初期/                         # 纯 Python 算法阶段：2D/3D PN 思路验证
├── 2_中期/                         # MAVSDK 联调阶段
├── 3_ROS2/                         # 早期 ROS 2 + PX4 工作空间
├── 4_fsm/                          # Offboard 状态机探索
├── 5_AntiDrone/                    # PX4 Offboard + PN 闭环拦截主实现
│   └── src/anti_drone_guidance/
│       ├── anti_drone_guidance_node.py  # 状态机、Offboard 控制、导引节点
│       ├── pn_guidance_core.py          # 与 ROS/PX4 解耦的 3D PN 核心
│       ├── target_provider.py           # 虚拟目标轨迹与目标源抽象
│       ├── gazebo_evaluator.py          # Gazebo 闭环评估器
│       ├── config/pn_guidance_params.yaml
│       └── launch/pn_guidance_launch.py
└── 6_Simulation/                   # 轻量 Python 多算法仿真 + 双机 Gazebo 接入
    ├── main.py                     # 多算法对比 CLI 入口
    ├── src/pythonsimulation/       # 质点模型、目标轨迹、导引算法、指标、绘图、MCAP
    └── src/gazebosimulation/       # ROS2/PX4/Gazebo 双机追踪接入包
```

## 技术栈

- Ubuntu 24.04（作者环境为 WSL2）
- ROS 2 Jazzy
- PX4 Autopilot v1.16
- Micro XRCE-DDS Agent
- Gazebo SITL
- QGroundControl
- Python 3、NumPy、Matplotlib、rosbag2、MCAP/Foxglove
- `uv` 用于 Python 依赖同步，根目录 `pyproject.toml` 要求 `python >= 3.14`

## 第五部分：PX4 Offboard 状态机与 PN 导引

`5_AntiDrone/src/anti_drone_guidance` 是当前 PX4 闭环拦截主实现。它把 PX4 Offboard 控制流程、安全监控、目标源和 PN 导引算法整合为一个 ROS 2 包。

### 状态机

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

状态机的重点不是只“发送控制量”，而是先确认 PX4 真的进入 Offboard 并解锁，再执行导引；同时保留电池、模式、上锁状态、阶段超时等安全联动。

### 3D PN 导引

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

### 目标源与评估器

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

## 第六部分：多算法对比仿真

`6_Simulation/` 是轻量级 3D 追踪仿真平台，用于在相同追踪机约束、相同初始条件、相同目标轨迹下对比不同导引/轨迹规划算法。

它不是高保真四旋翼动力学仿真，而是算法验证和可视化对比工具。追踪机使用简化 3D 质点模型，主要关注：

- 是否捕获目标
- 捕获时间和最小距离
- 路径长度和控制能量
- 目标是否离开视场
- yaw/pitch 角速度约束下的可见性保持

### 对比算法

算法名称集中定义在 `6_Simulation/src/pythonsimulation/config.py`：

| 内部名称 | 图表名称 | 作用 |
| --- | --- | --- |
| `basic` | Direct pursuit | 直接朝目标当前位置飞行，作为最直观基线。 |
| `pn` | 3D PN | 使用相对位置、相对速度、LOS 角速率生成三维 PN 加速度。 |
| `pn_fov` | PN + FOV | 在 PN 上加入视场约束，目标丢失时使用 last-seen 匀速预测。 |
| `pn_fov_cbf` | PN + FOV + CBF | 在接近 FOV 边界时加入控制屏障函数风格的侧向安全修正。 |
| `pn_fov_nmpc` | PN + FOV + NMPC | 在 PN 趋势附近枚举候选加速度，滚动预测后选择综合代价最低的控制。 |
| `pn_fov_mppi` | PN + FOV + MPPI | 基于 PN 名义控制序列进行采样式预测控制，用指数权重融合第一步控制。 |

其中 `basic` 和 `pn` 按理想传感器处理，目标始终可用于导引；FOV 系列算法会显式建模目标是否还在机头/相机视场内。

### 算法差异

- Direct pursuit 简单稳定，但不预测目标未来位置，目标横向运动时容易走弯路。
- 3D PN 更像“拦截”而不是“追当前位置”，对直线或圆周运动目标通常更直接。
- PN + FOV 引入可见性约束，目标出视场后只使用最后一次观测状态做短时预测。
- PN + FOV + CBF 是轻量安全滤波层，不替代 PN，只在 FOV 风险增大时修正控制。
- PN + FOV + NMPC 使用 shooting-based candidate optimization，不依赖 CasADi/acados 等重型求解器。
- PN + FOV + MPPI 通过 NumPy 批量采样多条控制序列，通常能减少 FOV 丢失时间，但计算量高于 PN/CBF。

### 输出指标

每个场景会输出图表、CSV 和可选 MCAP 回放。重点指标包括：

- `capture_time`：第一次进入捕获半径的时间，未捕获则记为仿真总时长。
- `min_distance`：整个仿真中的最小距离。
- `mean_distance`：平均距离误差。
- `path_length`：追踪机轨迹长度。
- `control_energy`：控制能量近似值 `sum(||a_cmd||^2 * dt)`。
- `lost_duration`：目标不在视场内的累计时间。
- `visible_ratio`：目标可见时间占总时间比例。
- `yaw_rate_mean` / `yaw_rate_variance`：机头转向强度及波动。

### Python 仿真运行

首次运行前在仓库根目录同步依赖：

```bash
uv sync
```

运行全部目标场景：

```bash
cd 6_Simulation
uv run python main.py --scenario all
```

可选场景：

- `stationary`：静止目标
- `linear`：匀速直线目标
- `circle`：圆周机动目标
- `all`：依次运行全部场景

常用命令：

```bash
uv run python main.py --scenario circle --sim-time 50 --dt 0.05
uv run python main.py --scenario linear --save-dir outputs/linear
uv run python main.py --scenario all --show
uv run python main.py --scenario circle --export-mcap
```

默认只保存图片和指标，不弹出 matplotlib 窗口；加 `--show` 后会显示窗口。加 `--export-mcap` 会在场景输出目录额外生成 `replay.mcap`，可用 Foxglove Studio 打开做时间线回放。

### 输出文件

默认输出目录：

```text
6_Simulation/outputs/
├── stationary/
├── linear/
└── circle/
```

每个场景通常包含：

- `trajectory_3d.png`：三维轨迹对比图。
- `trajectory_xy.png`：水平面俯视图。
- `trajectory_xz.png`：侧视图。
- `distance_error.png`：距离误差与捕获半径曲线。
- `fov_visibility.png`：LOS 夹角和 FOV 可见性。
- `acceleration.png`：控制加速度模长。
- `metrics.png`：指标柱状图。
- `metrics.csv`：算法指标表。
- `replay.mcap`：可选 Foxglove 回放文件。

## 第六部分 Gazebo 双机接入

`6_Simulation/src/gazebosimulation` 是 ROS2/PX4/Gazebo 双机追踪接入包。它复用 `pythonsimulation.guidance.compute_guidance()`，不维护第二套 Gazebo 专用算法。

该包只负责：

- 订阅追踪机和目标机 PX4 odometry。
- 根据目标机真实 odometry 计算追踪机导引控制。
- 给目标机发布由 `pythonsimulation.target.target_state()` 生成的轨迹 setpoint。
- 给追踪机发布 Offboard control mode、trajectory setpoint 和 vehicle command。

该包不负责：

- 启动 Gazebo 世界。
- 创建 Gazebo 机体模型。
- 启动 PX4 SITL 实例。
- 启动 Micro XRCE-DDS Agent。
- 启动 QGroundControl。

默认约定：

- `/px4_1` 是追踪机。
- `/px4_2` 是目标机。
- 默认算法是 `pn_fov_mppi`。
- 默认场景是 `circle`。

构建并启动：

```bash
cd 6_Simulation
colcon build --base-paths src --packages-up-to gazebosimulation --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch gazebosimulation guidance.launch.py algorithm:=pn_fov_mppi scenario:=circle
```

常用覆盖参数：

```bash
ros2 launch gazebosimulation guidance.launch.py \
  algorithm:=pn_fov_mppi \
  scenario:=circle \
  control_rate_hz:=20.0 \
  pursuer_namespace:=/px4_1 \
  target_namespace:=/px4_2 \
  auto_arm:=true \
  auto_offboard:=true \
  offboard_warmup_cycles:=20
```

订阅话题：

- `/<pursuer_namespace>/fmu/out/vehicle_odometry`
- `/<target_namespace>/fmu/out/vehicle_odometry`

发布话题：

- `/<pursuer_namespace>/fmu/in/offboard_control_mode`
- `/<pursuer_namespace>/fmu/in/trajectory_setpoint`
- `/<pursuer_namespace>/fmu/in/vehicle_command`
- `/<target_namespace>/fmu/in/offboard_control_mode`
- `/<target_namespace>/fmu/in/trajectory_setpoint`
- `/<target_namespace>/fmu/in/vehicle_command`

坐标转换集中在 `coordinates.py`，ENU/NED 位置、速度、加速度按 `[x, y, z] <-> [y, x, -z]` 转换。

## 第五部分快速开始

以下流程用于运行 PX4 SITL 仿真并启动 `5_AntiDrone` 的 ROS 2 导引节点。

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

## 第五部分参数配置

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

## 第五部分 Gazebo 闭环评估

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

## 参考资料

1. [PX4 官方文档](https://docs.px4.io/main/en/)
2. [ROS 2 Jazzy 官方文档](https://docs.ros.org/en/jazzy/index.html)
3. [MAVSDK 官方文档](https://mavsdk.mavlink.io/main/en/index.html)
4. [PX4 消息定义仓库](https://github.com/PX4/px4_msgs)
5. [FishROS 安装脚本](https://github.com/fishros/install)
6. [QGroundControl 官方站点](https://qgroundcontrol.com/)
