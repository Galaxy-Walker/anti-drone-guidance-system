# gazebosimulation

`gazebosimulation` 是 `6_Simulation` 的 ROS2/PX4/Gazebo 双机追踪接入包。完整说明已整合到上级文档：

- `6_Simulation/README.md`
- 根目录 `README.md`

核心边界保持不变：本包只发布 PX4 Offboard setpoint 并计算导引，不启动 Gazebo、PX4 SITL、Micro XRCE-DDS Agent、QGC，也不创建 Gazebo 机体模型。

节点退出时默认只把 Gazebo/PX4 闭环数据保存到 `outputs/gazebo/<scenario>/<algorithm>/gazebo_samples.csv`。

仿真结束后，在普通 Python/uv 环境中运行后处理脚本生成指标和图片：

```bash
cd 6_Simulation
uv run python plot_gazebo_csv.py outputs/gazebo/circle/pn_fov_nmpc/gazebo_samples.csv
```

可用 launch 参数关闭或改 CSV 路径：`record_data:=false`、`record_output_dir:=outputs/my_gazebo_run`。
