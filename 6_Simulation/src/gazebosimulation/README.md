# gazebosimulation

ROS2/PX4/Gazebo 双机追踪接入包。该包只发布 PX4 offboard setpoint 并计算导引，不启动 Gazebo、PX4 SITL、MicroXRCEAgent、QGC，也不创建 Gazebo 机体模型。

## 手动启动边界

先人工完成：

- 启动 Gazebo 仿真世界。
- 在 Gazebo 中创建两架 PX4 vehicle。
- 启动对应 PX4 SITL 实例。
- 启动 MicroXRCEAgent，确保 `px4_msgs` 话题桥接到 ROS2。
- 连接 QGC 并确认两架机状态正常。
- 再启动本 ROS2 节点。

默认约定：

- `/px4_1` 是追踪机。
- `/px4_2` 是目标机。
- namespace、算法、场景和控制频率都可通过 launch 参数覆盖。

## 构建与启动

从仓库根目录构建：

```bash
# 从零开始构建
colcon build --base-paths src --packages-up-to gazebosimulation --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 增量编译
colcon build --base-paths src --packages-select gazebosimulation
```

加载工作空间后启动：

```bash
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

## 话题

订阅：

- `/<pursuer_namespace>/fmu/out/vehicle_odometry`
- `/<target_namespace>/fmu/out/vehicle_odometry`

发布：

- `/<pursuer_namespace>/fmu/in/offboard_control_mode`
- `/<pursuer_namespace>/fmu/in/trajectory_setpoint`
- `/<pursuer_namespace>/fmu/in/vehicle_command`
- `/<target_namespace>/fmu/in/offboard_control_mode`
- `/<target_namespace>/fmu/in/trajectory_setpoint`
- `/<target_namespace>/fmu/in/vehicle_command`

## 控制逻辑

- 目标机 setpoint 来自 `pythonsimulation.target.target_state(scenario, elapsed_time, config)`。
- 追踪机导引输入使用目标机真实 PX4 odometry，不直接使用理想目标参考。
- 导引算法复用 `pythonsimulation.guidance.compute_guidance()`，不维护第二套 Gazebo 专用算法。
- 坐标转换集中在 `coordinates.py`，ENU/NED 位置、速度、加速度按 `[x, y, z] <-> [y, x, -z]` 转换。
- 节点等待双机 odometry 后，先连续发布 offboard control mode 和 trajectory setpoint，预热后再按参数发送 arm/offboard 命令。
