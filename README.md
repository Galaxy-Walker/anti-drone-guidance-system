# Anti-Drone 比例导引系统

基于 ROS 2、PX4 Offboard 和比例导引算法（Proportional Navigation, PN）的无人机拦截控制项目。

本仓库记录了一个从算法验证到工程化部署的完整演进过程：

- `1_初期/`：纯 Python 算法验证
- `2_中期/`：MAVSDK 联调与飞控接口测试
- `3_ROS2/`：ROS 2 + PX4 节点化实现

> 本项目主要用于学习、实验和交流。
> 代码由作者在学习过程中逐步完成，部分内容借助 AI 辅助生成，因此实现方式、工程规范和细节设计仍可能存在不足，欢迎提出问题和改进建议。

## 项目概述

本项目的目标是构建一个可在 PX4 仿真环境中运行的比例导引拦截系统。当前核心实现位于 `3_ROS2/` 工作空间，主要能力包括：

- 基于 PN 的目标拦截控制逻辑
- 基于 ROS 2 Launch 的节点化启动方式
- 通过 PX4 Offboard 模式执行控制指令
- 通过参数文件快速调整导引系数、速度限制和目标运动模型
- 支持仿真目标与 ROS 话题目标两类目标源

## 仓库结构

```text
.
├── 1_初期/                         # 纯算法阶段：2D/3D PN 验证
├── 2_中期/                         # MAVSDK 联调阶段
│   ├── guidance_system/
│   └── MAVSDK示例/
└── 3_ROS2/                         # ROS 2 + PX4 工作空间
	├── src/
	│   ├── px4_msgs/
	│   ├── px4_ros_com/
	│   └── ros2_guidance_system/
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

## 环境准备

建议先完成以下环境安装：

1. Ubuntu 24.04
2. ROS 2 Jazzy
3. PX4 Autopilot v1.16
4. Micro XRCE-DDS Agent
5. QGroundControl

参考资料见文末官方链接。若你使用 WSL2，请额外确认 QGroundControl 与 PX4 SITL 的网络互通配置。

## 快速开始

以下流程用于运行 PX4 SITL 仿真并启动 ROS 2 导引节点。

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

首次使用或修改 ROS 2 包代码后，建议在工作空间目录执行构建：

```bash
cd ~/Project/Anti-Drone/code/3_ROS2
colcon build
```

### 5. 加载工作空间环境

如果你使用 `bash`：

```bash
cd ~/Project/Anti-Drone/code/3_ROS2
source install/setup.bash
```

如果你使用 `zsh`：

```zsh
cd ~/Project/Anti-Drone/code/3_ROS2
source install/setup.zsh
```

### 6. 启动比例导引节点

```bash
ros2 launch ros2_guidance_system pn_guidance_launch.py
```

## 参数配置

比例导引相关参数通过 YAML 文件管理，默认配置文件位于：

```text
3_ROS2/src/ros2_guidance_system/config/pn_guidance_params.yaml
```

该文件中可调整的内容包括：

- 导引比例系数 `N`
- 最小/最大速度限制
- 起飞高度与拦截判定半径
- 控制频率
- 目标源类型
- 仿真目标运动模式与轨迹参数
- 调试日志输出频率

默认情况下，launch 会优先读取 `src` 目录下的配置文件，因此：

- 修改参数文件后通常无需重新编译
- 重启 launch 后即可加载最新配置

如果需要指定自定义配置文件，可以运行：

```bash
ros2 launch ros2_guidance_system pn_guidance_launch.py config_file:=/path/to/custom.yaml
```

## 已知前提

在运行本项目之前，请确认：

- PX4 SITL 已正常启动
- QGroundControl 已成功连接 PX4
- Micro XRCE-DDS Agent 正在监听 `udp4:8888`
- ROS 2 环境与本工作空间环境已经正确 source

如果其中任一环节未完成，ROS 2 节点可能无法正常与 PX4 建立通信。


## 参考资料

1. PX4 官方文档：https://docs.px4.io/main/en/
2. ROS 2 Jazzy 官方文档：https://docs.ros.org/en/jazzy/index.html
3. MAVSDK 官方文档：https://mavsdk.mavlink.io/main/en/index.html
4. PX4 消息定义仓库：https://github.com/PX4/px4_msgs
5. FishROS 安装脚本：https://github.com/fishros/install
6. QGroundControl 官方站点：https://qgroundcontrol.com/
