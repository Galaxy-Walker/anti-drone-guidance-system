# 7_2Dsimulation

`7_2Dsimulation` 是二维定高俯瞰追踪仿真。当前版本不使用深度相机和 FOV 约束，追踪机固定高度飞行，导引和指标按 XY 平面计算。

## 目录

- `main.py`：纯 Python 离线仿真入口，运行后生成指标 CSV 和图片。
- `src/pythonsimulation2d/`：2D 目标、动力学、导引、指标和绘图代码。
- `src/gazebosimulation2d/`：ROS2/PX4/Gazebo Offboard 接入包。
- `outputs/`：默认仿真输出目录。

## 纯 Python 仿真

进入目录：

```bash
cd 7_2Dsimulation
```

运行默认场景：

```bash
uv run main.py
```

运行全部场景：

```bash
uv run main.py --scenario all
```

指定场景、仿真时长和时间步长：

```bash
uv run main.py --scenario circle --sim-time 40 --dt 0.05
```

指定输出目录：

```bash
uv run main.py --scenario linear --save-dir outputs/linear_test
```

保存图片并弹出显示窗口：

```bash
uv run main.py --scenario stationary --show
```

输出文件默认保存到：

```text
outputs/<scenario>/
```

## ROS2/PX4/Gazebo 接入

进入 ROS2 工作区目录：

```bash
cd 7_2Dsimulation
```

编译 Gazebo 接入包：

```bash
colcon build --packages-select gazebosimulation2d
```

加载环境：

```bash
source install/setup.bash
```

启动 2D 导引节点：

```bash
ros2 launch gazebosimulation2d guidance.launch.py
```

指定算法和场景：

```bash
ros2 launch gazebosimulation2d guidance.launch.py algorithm:=pn_mppi scenario:=circle
```

常用参数示例：

```bash
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn_mppi \
  scenario:=circle \
  pursuer_fixed_altitude:=8.0 \
  sim_time:=40.0
```

Gazebo 接入包只发布 PX4 Offboard setpoint，不负责启动 Gazebo、PX4 SITL、Micro XRCE-DDS Agent 或 QGC。运行前需要先启动对应 PX4/Gazebo 双机环境。

## 可选算法和场景

算法：

```text
basic, pn, pn_mppi, pn_nmpc
```

场景：

```text
stationary, linear, circle
```

## ROS2 记录输出

节点退出时默认保存 CSV：

```text
outputs/gazebo2d/<scenario>/<algorithm>/gazebo_samples.csv
```

## Gazebo CSV 绘图

`plot_gazebo_csv.py` 用于把 Gazebo 记录的 `gazebo_samples.csv` 转成与纯 Python 仿真相同类型的指标和图片，不包含 FOV 相关输出。

按场景目录汇总绘图，输出到 `outputs/circle/`：

```bash
uv run plot_gazebo_csv.py \
  outputs/gazebo2d/circle \
  --output-dir outputs/circle
```

也可以只绘制单个算法的 CSV：

```bash
uv run plot_gazebo_csv.py \
  outputs/gazebo2d/circle/pn_mppi/gazebo_samples.csv
```

输出文件包括：

```text
metrics.csv
trajectory_xy.png
distance_error.png
acceleration.png
yaw_rate.png
metrics.png
```
