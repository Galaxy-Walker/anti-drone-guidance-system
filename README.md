# Anti-Drone — 比例导引拦截与多算法仿真系统

[![Python](https://img.shields.io/badge/python-%3E%3D3.14-blue?logo=python&logoColor=white)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314e?logo=ros)](https://docs.ros.org/en/jazzy/)
[![PX4](https://img.shields.io/badge/PX4-v1.16-231f20?logo=px4)](https://px4.io/)
[![License](https://img.shields.io/badge/license-MIT-green)](./LICENSE)

基于 ROS 2、PX4 Offboard、Gazebo 和比例导引算法（Proportional Navigation, PN）的无人机追踪/拦截系统。

## 目录

- [Anti-Drone — 比例导引拦截与多算法仿真系统](#anti-drone--比例导引拦截与多算法仿真系统)
  - [目录](#目录)
  - [项目简介](#项目简介)
  - [验证边界](#验证边界)
  - [仓库结构](#仓库结构)
  - [核心特性](#核心特性)
    - [导引算法矩阵](#导引算法矩阵)
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
算法原型(纯Python) → MAVSDK联调 → ROS2探索 → 状态机设计 → 3D 闭环拦截实现 → 3D 多算法仿真平台 → 2D 定高视觉闭环（主线）
```

当前模块定位：

| 模块 | 定位 | 用途 |
|------|------|------|
| [`7_2Dsimulation`](7_2Dsimulation/) | **主线** | 二维定高俯瞰追踪 + 下视相机 YOLO 视觉闭环 + 桌下遮挡丢失重获场景，含 6 种 2D 算法对比与 PX4/Gazebo 双机接入 |
| [`6_Simulation`](6_Simulation/) | 支撑 | 轻量 3D 质点仿真平台，统一对比 Direct pursuit / Direct pursuit+FOV / PN+FOV / CBF / MPPI / EMPC 共 6 种算法 |
| [`5_AntiDrone`](5_AntiDrone/) | 支撑 | PX4 Offboard + 3D PN 闭环拦截实现，含安全状态机和 Gazebo 评估器 |
| [`8_MoCap`](8_MoCap/) | 支撑 | 简单的动捕接入：动捕位姿桥接到 PX4 与悬停/绕飞/追踪/轨迹记录回放等基础任务 |

## 验证边界

- **主线是二维链路**：`7_2Dsimulation` 的定高俯瞰追踪、YOLO 视觉闭环与桌下遮挡丢失重获是本仓库的主要验证对象，文档中的闭环结果均出自该模块。
- **三维场景未做实机测试**：`5_AntiDrone` 与 `6_Simulation` 的三维算法只在纯 Python 仿真和 PX4 SITL + Gazebo 双机闭环中验证，本仓库没有三维链路的真机飞行记录。
- **无室外实测**：受禁飞限制无法进行室外试飞，所有实测数据均来自室内动捕场地或 PX4 SITL + Gazebo 仿真。

> [!note]
> 本项目主要用于学习、实验和交流。代码由作者在学习过程中逐步完成，部分内容借助 AI 辅助生成，欢迎提出问题和改进建议。

## 仓库结构

```text
code/
├── 1_初期/                 # 算法原型（纯 Python 2D/3D PN 验证）
├── 2_中期/                 # MAVSDK + PX4 SITL 联调探索
├── 3_ROS2/                 # 早期 ROS 2 工作空间（已废弃）
├── 4_fsm/                  # Offboard 有限状态机（C++ 原版 + Python 迁移版）
├── 5_AntiDrone/            # PX4 Offboard + PN 闭环拦截（仿真验证，未实机）
├── 6_Simulation/           # 轻量 3D 多算法对比仿真 + Gazebo 双机接入（仿真验证，未实机）
├── 7_2Dsimulation/         # 主线：二维定高追踪 + YOLO 视觉闭环与桌下遮挡重获场景
├── 8_MoCap/                # 室内动捕接入：悬停、绕飞、追踪与位姿桥接
├── pyproject.toml          # Python 项目配置与依赖
├── uv.lock                 # uv 依赖锁定文件
└── LICENSE                 # MIT 许可证
```

每个子目录都有独立的 `README.md`，详见[各模块文档](#各模块文档)。

## 核心特性

### 导引算法矩阵

**主线 2D 算法**（`7_2Dsimulation`，定高俯瞰，控制律只作用于 XY）：

| 算法 | 内部名称 | 类型 | 说明 |
|------|------|------|------|
| Direct Pursuit | `basic` | 基线 | 速度指向目标当前位置 |
| 2D PN | `pn` | 比例导引 | 水平面比例导引 + 主动接近项 |
| 2D PN + MPPI | `pn_mppi` | 采样预测 | PN 名义序列 + 随机采样加权 |
| 2D PN + EMPC | `pn_nmpc` | 枚举预测 | 候选枚举式滚动优化（含画面保持 FOV 惩罚） |
| PID tracking | `pid` | 闭环基线 | 单环位置 PID，直接输出加速度 |
| PID + EMPC | `pid_nmpc` | 混合 | PID 名义参考 + EMPC 枚举修正 |

## 技术栈与环境

| 组件 | 版本/说明 |
|------|----------|
| 操作系统 | Ubuntu 24.04（开发环境为 WSL2） |
| Python | >= 3.14 |
| 包管理 | uv |
| 核心依赖 | NumPy >= 2.4, Matplotlib >= 3.10, Foxglove SDK >= 0.24 |
| 中间件 | ROS 2 Jazzy |
| 飞控 | PX4 Autopilot v1.16（SITL + Gazebo） |
| 消息定义 | px4_msgs release/1.16（须与所用 PX4 版本一致，见 AGENTS.md） |
| 通信桥 | Micro XRCE-DDS Agent |
| 地面站 | QGroundControl |
| 仿真器 | Gazebo |
| 视觉推理 | YOLO（conda `ultralytics` 环境，`.engine`/`.pt` 权重） |
| 动捕真机（仅 `8_MoCap`） | Jetson + ROS 2 Humble + PX4 1.15.4 + VRPN 动捕 |

## 快速开始

### Python 依赖

```bash
# 在仓库根目录执行，uv 会自动读取 pyproject.toml 并锁定版本
uv sync
```

### 方式一：纯 Python 仿真（无需 ROS/PX4）

```bash
# 主线：2D 定高追踪仿真（静止/直线/圆周/桌下遮挡，6 种算法对比）
cd 7_2Dsimulation
uv run main.py --scenario all

# 3D 多算法对比（静止/直线/圆周目标，6 种算法）
cd 6_Simulation
uv run python main.py --scenario all

# 单独场景 + 弹出图表窗口 + 导出 Foxglove 回放
uv run python main.py --scenario circle --show --export-mcap
```

输出位置：`7_2Dsimulation/outputs/<scenario>/`、`6_Simulation/outputs/<scenario>/`

### 方式二：PX4 SITL 闭环（需要完整环境）

前置条件：ROS 2 Jazzy、PX4 Autopilot、Micro XRCE-DDS Agent、Gazebo（QGC 可选）均已安装。2D 视觉闭环还需要 conda `ultralytics` 环境与 YOLO 权重，详见 [`7_2Dsimulation/README.md`](7_2Dsimulation/README.md)。

```bash
# 终端 1（可选）— 启动 QGroundControl（GUI 应用）监控

# 终端 2 — 启动 Gazebo（2D 视觉闭环使用仓库内置无阴影世界）与 PX4 SITL 双机
#          具体启动顺序与环境变量见 7_2Dsimulation/README.md

# 终端 3 — 启动通信桥
MicroXRCEAgent udp4 -p 8888

# 终端 4 — 构建并启动导引节点（主线：2D 导引 + 视觉闭环）
cd 7_2Dsimulation
colcon build --packages-up-to gazebosimulation2d
source install/setup.bash
ros2 launch gazebosimulation2d guidance.launch.py

# 3D 闭环拦截（SITL + Gazebo 单机链路）
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
| 闭环拦截 | [`5_AntiDrone/README.md`](5_AntiDrone/README.md) | 状态机流程、PN 算法、参数配置、评估器（仅仿真验证） |
| 仿真平台 | [`6_Simulation/README.md`](6_Simulation/README.md) | 6 种 3D 算法详解、输出图表、Gazebo 双机接入（仅仿真验证） |
| **2D 主线** | [`7_2Dsimulation/README.md`](7_2Dsimulation/README.md) | 二维定高追踪、YOLO 视觉闭环、桌下遮挡重获、Gazebo 2D 接入 |
| 动捕接入 | [`8_MoCap/README.md`](8_MoCap/README.md) | 室内动捕位姿桥接与悬停/绕飞/追踪等基础任务（简单接入验证） |

## 许可证

本项目采用 [MIT License](LICENSE) 开源，版权所有 (c) 2026 Yue Shi。

## 参考资料

- [PX4 官方文档](https://docs.px4.io/main/en/)
- [ROS 2 Jazzy 文档](https://docs.ros.org/en/jazzy/index.html)
- [MAVSDK 文档](https://mavsdk.mavlink.io/main/en/index.html)
- [PX4 消息定义仓库](https://github.com/PX4/px4_msgs)
- [QGroundControl](https://qgroundcontrol.com/)
- [uv 包管理器](https://docs.astral.sh/uv/)
