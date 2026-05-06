# Anti-Drone 比例导引拦截系统

基于 ROS 2、PX4 Offboard 和比例导引算法（Proportional Navigation, PN）的无人机拦截控制项目。

本仓库记录了一个从算法验证到 ROS 2/PX4 闭环仿真的完整演进过程：

- `1_初期/`：纯 Python 算法验证，主要用于 2D/3D PN 思路验证。
- `2_中期/`：MAVSDK 联调与飞控接口测试。
- `3_ROS2/`：早期 ROS 2 + PX4 节点化实现。
- `4_fsm/`：Offboard 飞行状态机与安全机制探索。
- `5_AntiDrone/`：当前主要实现和推荐运行入口，整合 PN 导引、PX4 Offboard 状态机、虚拟目标源和 Gazebo 自动化评估器。

> 本项目主要用于学习、实验和交流。
> 代码由作者在学习过程中逐步完成，部分内容借助 AI 辅助生成，因此实现方式、工程规范和细节设计仍可能存在不足，欢迎提出问题和改进建议。

## 项目概述

当前核心实现位于 `5_AntiDrone/` 工作空间，主要目标是在 PX4 Gazebo SITL 环境中运行一个可闭环测试的反无人机比例导引系统。系统使用 PX4 uXRCE-DDS 话题与飞控通信，由 ROS 2 节点负责切入 Offboard、起飞、执行导引、判断拦截并触发降落。

第五部分的主要能力包括：

- 基于 3D PN 的目标拦截控制逻辑。
- PX4 Offboard 飞行状态机和基础安全监控。
- 起飞、导引、降落三个导引子阶段。
- 支持仿真目标和预留的 ROS 话题目标源。
- 支持静止、直线、等高圆周和变高度圆周目标轨迹。
- 支持 Gazebo 闭环评估器，自动生成 case、记录 rosbag 并输出 CSV 指标。

## 仓库结构

```text
.
├── 1_初期/                         # 纯算法阶段：2D/3D PN 验证
├── 2_中期/                         # MAVSDK 联调阶段
│   ├── guidance_system/
│   └── MAVSDK示例/
├── 3_ROS2/                         # 早期 ROS 2 + PX4 工作空间
├── 4_fsm/                          # Offboard 状态机探索
└── 5_AntiDrone/                    # 当前主要 ROS 2 + PX4 工作空间
    ├── src/
    │   ├── px4_msgs/               # PX4 ROS 2 消息定义
    │   └── anti_drone_guidance/
    │       ├── anti_drone_guidance/
    │       │   ├── anti_drone_guidance_node.py  # PX4 Offboard + PN 导引主节点
    │       │   ├── pn_guidance_core.py          # 与 ROS/PX4 解耦的 PN 核心算法
    │       │   ├── target_provider.py           # 虚拟目标轨迹与目标源抽象
    │       │   └── gazebo_evaluator.py          # Gazebo 闭环评估器
    │       ├── config/
    │       │   └── pn_guidance_params.yaml      # 默认参数文件
    │       ├── launch/
    │       │   └── pn_guidance_launch.py        # 导引节点启动文件
    │       ├── package.xml
    │       └── setup.py
    ├── bags/                       # 评估器生成的 rosbag 数据
    ├── results.csv                 # 评估器输出示例
    ├── build/
    ├── install/
    └── log/
```

## 技术栈

- Ubuntu 24.04（作者环境为 WSL2）
- ROS 2 Jazzy
- PX4 Autopilot v1.16
- Micro XRCE-DDS Agent
- Gazebo SITL
- QGroundControl
- Python 3、NumPy、rosbag2

## 第五部分核心代码

`5_AntiDrone/src/anti_drone_guidance` 是当前重点代码包，包名为 `anti_drone_guidance`。

### 导引主节点

`anti_drone_guidance_node.py` 将 PX4 Offboard 控制流程与 PN 导引算法合并到一个 ROS 2 节点中。

主飞行状态机为：

```text
INIT -> REQUESTING_OFFBOARD -> ARMING -> OFFBOARD_ACTIVE -> RETURN_REQUESTED / ERROR
```

其中 `OFFBOARD_ACTIVE` 内部包含导引子阶段：

```text
TAKEOFF -> GUIDANCE -> LAND
```

节点主要发布：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

节点主要订阅：

- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_control_mode`
- `/fmu/out/battery_status`

为了兼容不同 PX4 与 `px4_msgs` 组合，位置和状态话题同时兼容带 `_v1` 和不带 `_v1` 的命名。

### PN 核心算法

`pn_guidance_core.py` 实现与 ROS 2、PX4 解耦的比例导引核心。它输入追踪机和目标的 `position`、`velocity`、`speed`，输出：

- 期望位置设定点 `pos_cmd`
- 期望速度设定点 `vel_cmd`
- 当前距离、接近速度、视线角速率
- 期望速率和加速度指令等调试量

当前支持两种速度策略：

- `adaptive`：根据距离、接近速度和视线角速率动态调整追踪速度。
- `pursuit`：使用更简单的直接追击速度策略。

### 目标源

`target_provider.py` 将“目标数据从哪里来”和“导引算法如何使用目标状态”解耦。

当前内置的虚拟目标轨迹包括：

- `static`：静止悬停目标。
- `line`：匀速直线运动目标。
- `circle`：等高圆周运动目标。
- `circle_altitude`：水平圆周叠加正弦高度变化目标。

`RosTopicTarget` 作为真实传感器目标源的扩展入口保留，目前需要根据实际雷达、视觉或融合定位话题继续接入。

### Gazebo 自动化评估器

`gazebo_evaluator.py` 用于 PX4 Gazebo 闭环控制性能评估。目标机不创建 Gazebo 模型，而是在 ROS 2 中生成虚拟目标真值；追踪机使用 PX4 `gz_x500`，通过 `/fmu/in/*` 和 `/fmu/out/*` 话题闭环控制与记录。

评估器会为每个 case：

- 等待外部 PX4/uXRCE-DDS 话题就绪。
- 生成临时 ROS 2 参数文件。
- 启动 `anti_drone_guidance_node`。
- 启动 `ros2 bag record` 记录 PX4 和评估话题。
- 订阅真实追踪机位置与虚拟目标真值，在线计算性能指标。
- 写入 CSV 汇总结果，并输出分组统计。

评估模式下额外使用的话题：

- `/anti_drone_guidance/target_state`：虚拟目标位置和速度真值，类型为 `nav_msgs/Odometry`。
- `/anti_drone_guidance/evaluation_event`：评估事件，类型为 `std_msgs/String`，内容为 JSON 字符串。

## 环境准备

建议先完成以下环境安装：

1. Ubuntu 24.04
2. ROS 2 Jazzy
3. PX4 Autopilot v1.16
4. Micro XRCE-DDS Agent
5. QGroundControl

参考资料见文末官方链接。若你使用 WSL2，请额外确认 QGroundControl 与 PX4 SITL 的网络互通配置。

## 快速开始

以下流程用于运行 PX4 SITL 仿真并启动第五部分的 ROS 2 导引节点。

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

首次使用或修改 ROS 2 包代码后，在第五部分工作空间目录执行构建：

```bash
cd ~/Project/Anti-Drone/code/5_AntiDrone
colcon build
```

### 5. 加载工作空间环境

如果你使用 `bash`：

```bash
cd ~/Project/Anti-Drone/code/5_AntiDrone
source install/setup.bash
```

如果你使用 `zsh`：

```zsh
cd ~/Project/Anti-Drone/code/5_AntiDrone
source install/setup.zsh
```

### 6. 启动比例导引节点

```bash
ros2 launch anti_drone_guidance pn_guidance_launch.py
```

## 参数配置

比例导引相关参数通过 YAML 文件管理，默认配置文件位于：

```text
5_AntiDrone/src/anti_drone_guidance/config/pn_guidance_params.yaml
```

该文件按功能分为五组：

- `guidance`：导引比例系数 `N`、速度上下限、速度策略。
- `flight`：起飞高度、拦截判定半径、控制频率。
- `target`：目标源类型、虚拟目标运动模式、初始位置、圆周/直线/高度变化轨迹参数。
- `evaluation`：评估 case id 与评估话题开关，普通运行默认关闭。
- `debug`：日志输出间隔和详细日志开关。

默认情况下，launch 会优先读取 `src` 目录下的配置文件，因此：

- 修改参数文件后通常无需重新编译。
- 重启 launch 后即可加载最新配置。

如果需要指定自定义配置文件，可以运行：

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

默认评估矩阵包括：

- 目标轨迹：`static`、`line`、`circle`、`circle_altitude`
- 追踪机最大速度：`10`、`20`、`30` m/s
- 目标初始航向：`0`、`90`、`180`、`270` deg，其中 `static` 只运行一个航向

默认记录话题包括：

- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/anti_drone_guidance/target_state`
- `/anti_drone_guidance/evaluation_event`

CSV 默认输出到 `gazebo_evaluation_results.csv`，每行对应一个 case，主要字段包括：

- `case_id`
- `target_motion`
- `target_heading_deg`
- `speed_max`
- `success`
- `min_distance`
- `intercept_time`
- `max_actual_acceleration`
- `final_distance`
- `takeoff_time`
- `guidance_time`
- `total_time`
- `bag_path`

常用参数示例：

```bash
ros2 run anti_drone_guidance gazebo_evaluator \
  --motions static line \
  --speed-max 10 20 \
  --headings 0 90 180 270 \
  --output results.csv \
  --bag-dir bags
```

如果你用外部脚本负责重启 PX4/Gazebo，并且不希望每个 case 后等待手动确认，可以增加：

```bash
ros2 run anti_drone_guidance gazebo_evaluator --no-restart-prompt
```

## 已知前提

在运行本项目之前，请确认：

- PX4 SITL 已正常启动。
- QGroundControl 已成功连接 PX4。
- Micro XRCE-DDS Agent 正在监听 `udp4:8888`。
- ROS 2 环境与 `5_AntiDrone` 工作空间环境已经正确 source。
- `px4_msgs` 与当前 PX4/uXRCE-DDS 输出的话题版本匹配。

如果其中任一环节未完成，ROS 2 节点可能无法正常与 PX4 建立通信。

## 参考资料

1. [PX4 官方文档](https://docs.px4.io/main/en/)
2. [ROS 2 Jazzy 官方文档](https://docs.ros.org/en/jazzy/index.html)
3. [MAVSDK 官方文档](https://mavsdk.mavlink.io/main/en/index.html)
4. [PX4 消息定义仓库](https://github.com/PX4/px4_msgs)
5. [FishROS 安装脚本](https://github.com/fishros/install)
6. [QGroundControl 官方站点](https://qgroundcontrol.com/)
