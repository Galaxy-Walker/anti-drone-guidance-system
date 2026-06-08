# gazebosimulation2d

`gazebosimulation2d` 是 `7_2Dsimulation` 的 ROS2/PX4/Gazebo 双机追踪接入包，仿照 `6_Simulation/src/gazebosimulation` 实现。

边界：本包只发布 PX4 Offboard setpoint 并调用 `pythonsimulation2d` 中的 2D 导引算法；不启动 Gazebo、PX4 SITL、Micro XRCE-DDS Agent、QGC，也不创建 Gazebo 机体模型。

2D 模式约定：

- 启动阶段目标机飞到场景起点，追踪机保持准备阶段开始时的当前 XY 并起飞到 `pursuer_fixed_altitude`；两方都满足位置和速度阈值后才开始 2D 追踪和数据记录。
- 追踪机进入追踪阶段后使用 `velocity + acceleration` setpoint；2D 导引输出的水平加速度会作为 PX4 acceleration 前馈发布，position 字段保持未启用。
- 追踪阶段的 z 速度和 z 加速度指令为 0；导引、距离和指标只使用 XY 分量。
- 目标机沿用 6 中目标机/话题/模型；目标参考 XY 来自 `pythonsimulation2d.target_state`，z 固定为 `target_base_altitude`，默认 1m。
- `pursuer_fixed_altitude` 默认 8m，用于 2D 仿真配置和结果标注；当前追踪阶段不再用 position setpoint 强制拉高度。

示例：

```bash
cd 7_2Dsimulation
colcon build --packages-select gazebosimulation2d
source install/setup.bash
ros2 launch gazebosimulation2d guidance.launch.py algorithm:=pn_mppi scenario:=circle
```

开启 0.2s 周期 ROS 日志调试输出：

```bash
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn_nmpc \
  scenario:=circle \
  pursuer_fixed_altitude:=8.0 \
  sim_time:=40.0 \
  debug_log:=true
```

调试日志以 `debug_2d` 开头，包含：

- `target_odom`：目标机实际 odometry 位置、速度、加速度。
- `target_cmd`：目标机参考位置、速度、加速度和 yaw setpoint。
- `pursuer_odom`：追踪机实际 odometry 位置、速度、加速度。
- `pursuer_cmd`：追踪机 `velocity + acceleration` 控制指令、原始导引加速度和限幅后加速度。

启动阶段会以 `startup_2d` 低频输出目标机/追踪机的就位误差、速度和命令状态；默认周期为 1s，可用 `startup_log_period_s` 调整。

如需调整日志周期：

```bash
ros2 launch gazebosimulation2d guidance.launch.py debug_log:=true debug_log_period_s:=0.1
```

节点退出时默认保存 CSV：

```text
outputs/gazebo2d/<scenario>/<algorithm>/gazebo_samples.csv
```
