# 3_ROS2 — ROS 2 早期探索

最早期的 ROS 2 + PX4 工作空间。包含一个基本的 `ament_python` 导引包和 offboard 测试节点。

## 内容

- `src/ros2_guidance_system/` — 早期 ROS 2 导引节点（PN 核心 + 目标源 + 状态机）
- `src/offboard_test/` — PX4 offboard 控制测试
- `src/px4_msgs/` — PX4-ROS 2 消息定义

## 演进路径

本阶段的架构思想（ROS 2 offboard 节点、PN 核心模块化）被 `5_AntiDrone` 继承并大幅改进。此工作空间已废弃，不建议继续使用。
