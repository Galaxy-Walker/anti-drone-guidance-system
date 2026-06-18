# 8_MoCap — 室内动捕悬停

在 GPS 拒止环境下，通过外部运动捕捉系统（如 OptiTrack、Vicon）为 PX4 提供高精度位置估计。

## 内容

- [`src/px4_mocap_hover/`](src/px4_mocap_hover/) — ROS 2 Python 包，包含：
  - `mocap_bridge`：将 VRPN 动捕位姿转换为 PX4 NED 坐标系并发布
  - `mocap_hover`：自动起飞、悬停、降落的完整任务节点

## 典型用途

在室内飞行场地为 `5_AntiDrone` 或其它 PX4 应用提供亚厘米级定位，替代 GPS。
