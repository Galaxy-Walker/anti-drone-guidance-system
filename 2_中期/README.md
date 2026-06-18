# 2_中期 — MAVSDK 联调阶段

从纯仿真向真实飞控对接的过渡阶段。尝试通过 MAVSDK（MAVLink）与 PX4 SITL 建立通信。

## 内容

- `guidance_system/` — 重构成的 PN 导引系统（`PNGuidance.py`），支持通过 MAVSDK 与 PX4 通信
- `MAVSDK示例/` — MAVSDK 学习参考（offboard 轨迹跟随、follow-me 等示例）

## 演进路径

MAVSDK 路线后续被 ROS 2 路线取代（见 `3_ROS2` → `5_AntiDrone`）。本阶段的核心价值在于验证了通过外部 API 控制 PX4 的可行性，为后续 ROS 2 offboard 控制积累了经验。
