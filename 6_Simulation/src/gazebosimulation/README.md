# gazebosimulation

`gazebosimulation` 是 `6_Simulation` 的 ROS2/PX4/Gazebo 双机追踪接入包。完整说明已整合到上级文档：

- `6_Simulation/README.md`
- 根目录 `README.md`

核心边界保持不变：本包只发布 PX4 Offboard setpoint 并计算导引，不启动 Gazebo、PX4 SITL、Micro XRCE-DDS Agent、QGC，也不创建 Gazebo 机体模型。
