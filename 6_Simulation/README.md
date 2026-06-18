# 6_Simulation 多算法追踪仿真

`6_Simulation` 是轻量级 3D 无人机追踪仿真与 ROS2/PX4/Gazebo 双机接入工作区，用于对比不同导引/轨迹规划算法在相同追踪机约束、相同初始条件、相同目标轨迹下的表现。

本项目不是高保真四旋翼动力学仿真，而是算法验证和可视化对比工具。追踪机使用简化 3D 质点模型，主要关注追踪路径、捕获时间、视场丢失和控制强度。

## 组成

```text
6_Simulation/
├── main.py                         # Python 多算法对比 CLI 入口
├── outputs/                        # 默认图表、CSV、MCAP 输出目录
└── src/
    ├── pythonsimulation/
    │   ├── config.py               # 场景、算法和参数配置
    │   ├── state.py                # 状态和仿真结果数据结构
    │   ├── target.py               # 静止、直线、圆周目标轨迹
    │   ├── dynamics.py             # 简化追踪机动力学
    │   ├── guidance.py             # 导引算法与预测控制
    │   ├── simulation.py           # 统一仿真循环
    │   ├── metrics.py              # 指标计算和 CSV 输出
    │   ├── plotting.py             # 图表生成
    │   └── replay.py               # Foxglove/MCAP 回放导出
    └── gazebosimulation/           # ROS2/PX4/Gazebo 双机追踪接入包
```

## Python 仿真运行

项目使用 Python 3.14，并通过根目录 `uv.lock` 锁定依赖。首次运行前在仓库根目录执行：

```bash
uv sync
```

运行全部目标场景：

```bash
cd 6_Simulation
uv run python main.py --scenario all
```

可选场景：

- `stationary`：静止目标
- `linear`：匀速直线目标
- `circle`：圆周机动目标
- `all`：依次运行全部场景

常用参数：

```bash
uv run python main.py --scenario circle --sim-time 50 --dt 0.05
uv run python main.py --scenario linear --save-dir outputs/linear
uv run python main.py --scenario all --show
uv run python main.py --scenario circle --export-mcap
```

默认只保存图片和指标，不弹出 matplotlib 窗口；加 `--show` 后会显示窗口。加 `--export-mcap` 会在场景输出目录额外生成 `replay.mcap`，可用 Foxglove Studio 打开做时间线回放。

## 对比算法

算法统一定义在 `src/pythonsimulation/config.py`，仿真循环、绘图、CSV 和 Gazebo 接入包都复用同一组名称。

| 内部名称 | 图表名称 | 说明 |
| --- | --- | --- |
| `basic` | Direct pursuit | 追踪机始终朝目标当前位置飞行，作为最直观基线。 |
| `pn` | 3D PN | 使用相对位置、相对速度和 LOS 角速率生成三维 PN 加速度。 |
| `pn_fov` | PN + FOV | 加入视场约束，目标离开视场后使用 last-seen 匀速预测。 |
| `pn_fov_cbf` | PN + FOV + CBF | 在接近 FOV 边界时加入控制屏障函数风格的侧向安全修正。 |
| `pn_fov_mppi` | PN + FOV + MPPI | 基于 PN 名义控制序列随机采样多条控制序列，用指数权重融合第一步控制。 |
| `pn_fov_nmpc` | PN + FOV + NMPC | 在 PN 趋势附近枚举候选加速度，滚动预测后选择综合代价最低的控制。 |

> **📖 算法原理与设计分析**：各算法的详细原理、设计动机和对比分析见 [`docs/algorithm_framework_rationale.md`](docs/algorithm_framework_rationale.md)。整体仿真方法论见 [`docs/simulation_overview.md`](docs/simulation_overview.md)。

## 输出文件

每个场景目录包含以下文件：

| 文件 | 内容 | 用途 |
|------|------|------|
| `trajectory_3d.png` | 三维轨迹对比（黑色虚线=目标，彩色=各算法追踪机） | 观察路径形状与接近效果 |
| `trajectory_xy.png` | 水平面俯视图 | 观察横向拦截效果 |
| `trajectory_xz.png` | 侧视图 | 检查高度响应与限制 |
| `distance_error.png` | 距离误差曲线 + 捕获半径线 | 判断捕获时间与收敛速度 |
| `fov_visibility.png` | LOS 夹角与可见性 0/1 状态 | 查看 FOV 丢失时段 |
| `acceleration.png` | 控制加速度模长曲线 | 比较控制强度与剧烈程度 |
| `metrics.png` | 指标柱状图（各指标独立小图） | 多算法一目了然对比 |
| `metrics.csv` | 结构化指标表 | 论文表格 / 进一步分析 |
| `replay.mcap` | Foxglove 时间线回放（需 `--export-mcap`） | 交互式 3D 动画回放 |

指标字段：`capture_time`、`min_distance`、`mean_distance`、`path_length`、`control_energy`、`lost_duration`、`visible_ratio`、`yaw_rate_mean`、`yaw_rate_variance`。

> **📖 读图与分析方法**：每张图的详细读法、指标含义以及如何从中提取结论，见 [`docs/simulation_overview.md`](docs/simulation_overview.md)。

## Foxglove 回放

生成短时间回放文件用于快速检查：

```bash
uv run python main.py --scenario circle --sim-time 5 --save-dir outputs_verify/circle --export-mcap
```

打开 `outputs_verify/circle/replay.mcap` 后，Foxglove 的时间线可以播放、暂停、拖动仿真过程。推荐添加 3D 面板和 Plot 面板：

- `/target/scene` 显示目标当前位置和目标轨迹
- `/<algorithm>/scene` 显示对应导引算法的追踪机、轨迹、机头方向和 LOS 线
- `/<algorithm>/telemetry` 可绘制 `distance`、`visible`、`los_angle_deg`、`acceleration_norm`、`yaw_rate`

不同导引算法通过 Foxglove 的 channel 开关自由选择，例如只打开 `/pn/scene` 和 `/pn_fov_mppi/scene` 即可对比这两个算法。

## ROS2/PX4/Gazebo 双机接入

`src/gazebosimulation` 是 ROS2/PX4/Gazebo 双机追踪接入包。该包只发布 PX4 Offboard setpoint 并计算导引，不启动 Gazebo、PX4 SITL、Micro XRCE-DDS Agent、QGC，也不创建 Gazebo 机体模型。

### 启动流程

按以下顺序逐个终端执行：

**1. 打开 QGroundControl**

启动 QGC 桌面应用，用于监控两架飞行器状态。

**2. 启动 Micro XRCE-DDS Agent**

```bash
MicroXRCEAgent udp4 -p 8888
```

**3. 启动追踪机 PX4 SITL（namespace `/px4_1`）**

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_UXRCE_DDS_NS=px4_1 \
./build/px4_sitl_default/bin/px4 -i 0
```

**4. 启动目标机 PX4 SITL（namespace `/px4_2`）**

```bash
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
PX4_GZ_MODEL_POSE="0,5,0,0,0,0" \
PX4_UXRCE_DDS_NS=px4_2 \
./build/px4_sitl_default/bin/px4 -i 1
```

**5. 构建并启动导引节点**

```bash
cd 6_Simulation
source install/setup.bash

ros2 launch gazebosimulation guidance.launch.py \
  algorithm:=pn_fov_nmpc \
  scenario:=circle \
  control_rate_hz:=20.0 \
  pursuer_namespace:=/px4_1 \
  target_namespace:=/px4_2 \
  target_teleport_enabled:=true \
  target_gazebo_world:=default \
  target_gazebo_model:=x500_1 \
  auto_arm:=true \
  auto_offboard:=true \
  offboard_warmup_cycles:=20 \
  record_data:=true \
  record_output_dir:=outputs/gazebo
```

> `PX4_GZ_MODEL_POSE="0,5,0,0,0,0"` 仅作为目标机的临时初始位姿。导引开始前节点会调用 Gazebo 的 `/world/<world>/set_pose` 将目标机瞬移到轨迹起点，因此初始位姿无需精确匹配。如果目标机 Gazebo 模型名不是 `x500_1`，用 `target_gazebo_model:=<实际模型名>` 覆盖。

### 边界说明

默认约定：

- `/px4_1` 是追踪机。
- `/px4_2` 是目标机。
- namespace、算法、场景和控制频率都可通过 launch 参数覆盖。

### 构建

从 `6_Simulation` 目录构建：

```bash
# 从零开始构建
colcon build --base-paths src --packages-up-to gazebosimulation --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 增量编译
colcon build --base-paths src --packages-select gazebosimulation
```

### 所有 launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `algorithm` | `pn_fov_mppi` | 导引算法：`basic`、`pn`、`pn_fov`、`pn_fov_cbf`、`pn_fov_mppi`、`pn_fov_nmpc` |
| `scenario` | `circle` | 目标场景：`stationary`、`linear`、`circle` |
| `control_rate_hz` | `20.0` | 控制循环频率 |
| `pursuer_namespace` | `/px4_1` | 追踪机 namespace |
| `target_namespace` | `/px4_2` | 目标机 namespace |
| `target_teleport_enabled` | `true` | 导引开始前将目标机瞬移到轨迹起点 |
| `target_gazebo_world` | `default` | Gazebo 世界名称（用于 set_pose） |
| `target_gazebo_model` | `x500_1` | 目标机 Gazebo 模型名 |
| `auto_arm` | `true` | 自动发送解锁命令 |
| `auto_offboard` | `true` | 自动请求 Offboard 模式 |
| `offboard_warmup_cycles` | `20` | Offboard 预热周期数 |
| `record_data` | `true` | 退出时保存 CSV 数据 |
| `record_output_dir` | `outputs/gazebo` | CSV 输出根目录 |

### Gazebo 数据记录与绘图

`guidance_node` 默认会在节点退出时保存一次 Gazebo/PX4 闭环 CSV。输出路径为：

```text
outputs/gazebo/<scenario>/<algorithm>/gazebo_samples.csv
```

CSV 内容包含控制周期采样的追踪机/目标机 ENU 位置、速度、导引加速度、yaw、pitch、距离、FOV 可见性和 LOS 角。

仿真结束后，在普通 Python/uv 环境中单独生成指标和图片：

```bash
cd 6_Simulation
uv run python plot_gazebo_csv.py outputs/gazebo/circle/pn_fov_nmpc/gazebo_samples.csv
```

后处理输出目录默认就是 CSV 所在目录，包含：

- `metrics.csv`：复用 `pythonsimulation.metrics` 计算的指标。
- `trajectory_3d.png`、`trajectory_xy.png`、`trajectory_xz.png`、`distance_error.png`、`fov_visibility.png`、`acceleration.png`、`yaw_rate.png`、`metrics.png`：复用 `pythonsimulation.plotting` 生成的图。

后处理脚本也支持显式参数，例如：

```bash
uv run python plot_gazebo_csv.py outputs/my_gazebo_run/circle/pn_fov_nmpc/gazebo_samples.csv \
  --scenario circle \
  --algorithm pn_fov_nmpc \
  --output-dir outputs/my_gazebo_run_plots
```

ROS 节点常用记录参数：

- `record_data:=false`：关闭 ROS 节点 CSV 记录。
- `record_output_dir:=outputs/my_gazebo_run`：修改 CSV 输出根目录，实际结果仍按 `<scenario>/<algorithm>` 分目录。

### 话题

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

### 控制逻辑

- 目标机 setpoint 来自 `pythonsimulation.target.target_state(scenario, elapsed_time, config)`。
- 追踪机导引输入使用目标机真实 PX4 odometry，不直接使用理想目标参考。
- 导引算法复用 `pythonsimulation.guidance.compute_guidance()`，不维护第二套 Gazebo 专用算法。
- 坐标转换集中在 `coordinates.py`，ENU/NED 位置、速度、加速度按 `[x, y, z] <-> [y, x, -z]` 转换。
- 节点等待双机 odometry 后，先连续发布 Offboard control mode 和 trajectory setpoint，预热后再按参数发送 arm/offboard 命令。
