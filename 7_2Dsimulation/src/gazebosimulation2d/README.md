# gazebosimulation2d

`gazebosimulation2d` 是 `7_2Dsimulation` 的 ROS2/PX4/Gazebo 双机追踪接入包，仿照 `6_Simulation/src/gazebosimulation` 实现。

边界：本包只发布 PX4 Offboard setpoint 并调用 `pythonsimulation2d` 中的 2D 导引算法；不启动 Gazebo、PX4 SITL、Micro XRCE-DDS Agent、QGC，也不创建 Gazebo 机体模型。

2D 模式约定：

- 追踪机使用 XY 位置 setpoint，z 固定为 `pursuer_fixed_altitude`，默认 8m。
- 目标机沿用 6 中目标机/话题/模型；目标参考 XY 来自 `pythonsimulation2d.target_state`，z 固定为 `target_base_altitude`，默认 1m。
- 导引、距离和 CSV 指标均按 XY 水平面计算。

示例：

```bash
cd 7_2Dsimulation
colcon build --packages-select gazebosimulation2d
source install/setup.bash
ros2 launch gazebosimulation2d guidance.launch.py algorithm:=pn_mppi scenario:=circle
```

节点退出时默认保存 CSV：

```text
outputs/gazebo2d/<scenario>/<algorithm>/gazebo_samples.csv
```
