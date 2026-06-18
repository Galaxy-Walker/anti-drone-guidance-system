# Anti-Drone — 比例导引拦截与多算法仿真系统

[![Python](https://img.shields.io/badge/python-%3E%3D3.14-blue?logo=python&logoColor=white)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314e?logo=ros)](https://docs.ros.org/en/jazzy/)
[![PX4](https://img.shields.io/badge/PX4-v1.16-231f20?logo=px4)](https://px4.io/)
[![License](https://img.shields.io/badge/license-MIT-green)](./LICENSE)

基于 ROS 2、PX4 Offboard、Gazebo 和比例导引算法（Proportional Navigation, PN）的无人机追踪/拦截系统。

> 核心亮点：3D PN 导引 + FOV 约束 + CBF / MPPI / NMPC 多算法对比，覆盖从纯 Python 原型仿真到 PX4 Gazebo SITL 闭环的完整链路。

## 目录

- [Anti-Drone — 比例导引拦截与多算法仿真系统](#anti-drone--比例导引拦截与多算法仿真系统)
  - [目录](#目录)
  - [项目简介](#项目简介)
  - [仓库结构](#仓库结构)
  - [核心特性](#核心特性)
    - [导引算法矩阵](#导引算法矩阵)
    - [评估指标](#评估指标)
    - [从仿真到真机](#从仿真到真机)
  - [技术栈与环境](#技术栈与环境)
  - [快速开始](#快速开始)
    - [Python 依赖](#python-依赖)
    - [方式一：纯 Python 仿真（无需 ROS/PX4）](#方式一纯-python-仿真无需-rospx4)
    - [方式二：PX4 SITL 闭环（需要完整环境）](#方式二px4-sitl-闭环需要完整环境)
  - [各模块文档](#各模块文档)
  - [许可证](#许可证)
  - [参考资料](#参考资料)

## 项目简介

本仓库记录了无人机追踪/拦截系统从算法验证到工程实现的完整演进历史：

```text
算法原型(纯Python) → MAVSDK联调 → ROS2探索 → 状态机设计 → 闭环拦截实现 → 多算法仿真平台
```

当前最主要的两个模块：

| 模块 | 用途 |
|------|------|
| [`5_AntiDrone`](5_AntiDrone/) | PX4 Offboard + 3D PN 闭环拦截主实现，含安全状态机和 Gazebo 评估器 |
| [`6_Simulation`](6_Simulation/) | 轻量 3D 质点仿真平台，统一对比 Direct pursuit / PN / PN+FOV / CBF / MPPI / NMPC 共 6 种算法 |

> 本项目主要用于学习、实验和交流。代码由作者在学习过程中逐步完成，部分内容借助 AI 辅助生成，欢迎提出问题和改进建议。

## 仓库结构

```text
code/
├── 1_初期/                 # 算法原型（纯 Python 2D/3D PN 验证）
├── 2_中期/                 # MAVSDK + PX4 SITL 联调探索
├── 3_ROS2/                 # 早期 ROS 2 工作空间（已废弃）
├── 4_fsm/                  # Offboard 有限状态机（C++ 原版 + Python 迁移版）
├── 5_AntiDrone/            # PX4 Offboard + PN 闭环拦截（当前主线）
├── 6_Simulation/           # 轻量 3D 多算法对比仿真 + Gazebo 双机接入（当前主线）
├── 7_2Dsimulation/         # 二维定高俯瞰追踪仿真
├── 8_MoCap/                # 室内动捕悬停与位姿桥接
├── pyproject.toml          # Python 项目配置与依赖
├── uv.lock                 # uv 依赖锁定文件
└── LICENSE                 # MIT 许可证
```

每个子目录都有独立的 `README.md`，详见[各模块文档](#各模块文档)。

## 核心特性

### 导引算法矩阵

| 算法 | 类型 | FOV 约束 | 预测控制 | 说明 |
|------|------|:---:|:---:|------|
| Direct pursuit | 基线 | — | — | 直接追踪目标当前位置 |
| 3D PN | 比例导引 | — | — | 经典三维比例导引拦截 |
| PN + FOV | FOV | ✓ | — | 加入视场约束与丢失预测 |
| PN + FOV + CBF | FOV | ✓ | — | 控制屏障函数安全滤波 |
| PN + FOV + MPPI | FOV+采样 | ✓ | ✓ | 采样式预测控制 |
| PN + FOV + NMPC | FOV+优化 | ✓ | ✓ | 候选枚举式滚动优化 |

### 评估指标

- 捕获时间与最小距离
- 路径长度与控制能量
- FOV 丢失时长与可见率
- Yaw 角速度均值与方差
- Foxglove MCAP 时间线回放

### 从仿真到真机

- **纯 Python 仿真**：`6_Simulation/main.py` — 一键运行 6 种算法的定点/直线/圆周场景对比
- **Gazebo SITL**：`6_Simulation/src/gazebosimulation` — 双机 PX4 Gazebo 闭环接入
- **实机预备**：`5_AntiDrone` — PX4 Offboard 安全状态机，含解锁确认/超时保护/返航回退

## 技术栈与环境

| 组件 | 版本/说明 |
|------|----------|
| 操作系统 | Ubuntu 24.04（开发环境为 WSL2） |
| Python | >= 3.14 |
| 包管理 | uv |
| 核心依赖 | NumPy >= 2.4, Matplotlib >= 3.10, Foxglove SDK >= 0.24 |
| 中间件 | ROS 2 Jazzy |
| 飞控 | PX4 Autopilot v1.16（SITL + Gazebo） |
| 通信桥 | Micro XRCE-DDS Agent |
| 地面站 | QGroundControl |
| 仿真器 | Gazebo |

## 快速开始

### Python 依赖

```bash
# 在仓库根目录执行，uv 会自动读取 pyproject.toml 并锁定版本
uv sync
```

### 方式一：纯 Python 仿真（无需 ROS/PX4）

```bash
cd 6_Simulation

# 运行全部场景（静止/直线/圆周目标，6 种算法对比）
uv run python main.py --scenario all

# 单独场景 + 弹出图表窗口 + 导出 Foxglove 回放
uv run python main.py --scenario circle --show --export-mcap
```

输出位置：`6_Simulation/outputs/<scenario>/`

### 方式二：PX4 SITL 闭环（需要完整环境）

前置条件：ROS 2 Jazzy、PX4 Autopilot、QGroundControl 均已安装。

```bash
# 终端 1 — 启动 QGroundControl（GUI 应用）

# 终端 2 — 启动 PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 终端 3 — 启动通信桥
MicroXRCEAgent udp4 -p 8888

# 终端 4 — 构建并启动导引节点
cd 5_AntiDrone
colcon build --packages-select anti_drone_guidance
source install/setup.bash
ros2 launch anti_drone_guidance pn_guidance_launch.py
```

## 各模块文档

每个子目录都有独立的 `README.md`，请按需查阅：

| 模块 | README | 内容概要 |
|------|--------|---------|
| 算法原型 | [`1_初期/README.md`](1_初期/README.md) | 最早的 PN 算法验证脚本说明 |
| MAVSDK 阶段 | [`2_中期/README.md`](2_中期/README.md) | MAVSDK 联调背景与过渡 |
| 早期 ROS 2 | [`3_ROS2/README.md`](3_ROS2/README.md) | 废弃原因与可继承部分 |
| 状态机探索 | [`4_fsm/README.md`](4_fsm/README.md) | FSM 设计概述与双实现说明 |
| 闭环拦截 | [`5_AntiDrone/README.md`](5_AntiDrone/README.md) | 状态机流程、PN 算法、参数配置、评估器 |
| 仿真平台 | [`6_Simulation/README.md`](6_Simulation/README.md) | 6 种算法详解、输出图表、Gazebo 双机接入 |
| 2D 仿真 | [`7_2Dsimulation/README.md`](7_2Dsimulation/README.md) | 二维仿真与 Gazebo 2D 接入 |
| 动捕悬停 | [`8_MoCap/README.md`](8_MoCap/README.md) | 室内动捕定位与自动任务 |

## 许可证

本项目采用 [MIT License](LICENSE) 开源，版权所有 (c) 2026 Yue Shi。

## 参考资料

- [PX4 官方文档](https://docs.px4.io/main/en/)
- [ROS 2 Jazzy 文档](https://docs.ros.org/en/jazzy/index.html)
- [MAVSDK 文档](https://mavsdk.mavlink.io/main/en/index.html)
- [PX4 消息定义仓库](https://github.com/PX4/px4_msgs)
- [QGroundControl](https://qgroundcontrol.com/)
- [uv 包管理器](https://docs.astral.sh/uv/)
