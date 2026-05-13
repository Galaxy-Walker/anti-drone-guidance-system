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
| `pn_fov_nmpc` | PN + FOV + NMPC | 在 PN 趋势附近枚举候选加速度，滚动预测后选择综合代价最低的控制。 |
| `pn_fov_mppi` | PN + FOV + MPPI | 基于 PN 名义控制序列随机采样多条控制序列，用指数权重融合第一步控制。 |

### Direct pursuit

基础追踪法。追踪机始终朝目标当前位置飞行：先计算“追踪机到目标”的方向，再给出沿该方向的期望速度，最后用加速度去追踪这个期望速度。

优点是直观、稳定、容易理解；缺点是不会预测目标未来位置。目标横向移动时，它会一直追着目标当前位置跑，轨迹可能更弯，拦截效率较低。

### 3D PN

三维比例导引。PN 会利用相对位置、相对速度和视线角速度来生成加速度，让追踪机更像是在“拦截”目标，而不是单纯追目标当前位置。

相比 Direct pursuit，PN 对移动目标通常更有效，尤其是目标存在横向速度时，轨迹会更直接。当前实现还加入了径向闭合项，避免初始相对速度较小时接近过慢。

### PN + FOV

在 PN 基础上加入视场约束。追踪机有 yaw/pitch 表示机头或相机朝向，目标方向与机头前向的夹角小于半视场角时才认为目标可见。

当目标可见时，算法使用真实目标状态进行 PN 导引；当目标离开视场时，算法只使用最后一次看到的目标位置和速度做短时匀速预测，并降低导引强度。这会产生更接近视觉追踪的行为，例如目标快速横穿或近距离绕飞时可能丢失。

### PN + FOV + CBF

在 PN + FOV 基础上加入控制屏障函数（Control Barrier Function, CBF）风格的安全滤波器。它不完全替代 PN，而是在目标接近 FOV 边界时加入侧向修正，并在必要时混入速度匹配项，降低近距离飞越后目标被甩出视场的概率。

当前实现是适配本项目质点模型的轻量近似，不引入二次规划求解器。它更像一个在线安全修正层：如果一步预测发现修正反而让 LOS/FOV 更差，就回退到原 PN + FOV 控制。

### PN + FOV + NMPC

在 PN + FOV 基础上加入轻量级预测外环。这里的 NMPC 不依赖 CasADi/acados 等重型求解器，而是使用 shooting-based candidate optimization：

1. 以 PN 加速度为趋势方向。
2. 构造一组候选加速度，例如更大/更小 PN 增益、混入直接接近目标的控制、匹配目标速度、向 LOS 垂直方向轻微绕行。
3. 对每个候选在短预测窗口内前向仿真。
4. 用距离、FOV 违反、控制能量、平滑性、偏离 PN 趋势等代价评分。
5. 选择代价最低的候选，只执行第一步，下一帧重新滚动优化。

它的目标不是绝对最快捕获，而是在追踪目标的同时尽量减少 FOV 丢失。当前参数下，NMPC 往往更保守，可能牺牲一些捕获速度来换取更好的视场保持。

### PN + FOV + MPPI

在 PN + FOV 基础上加入 MPPI（Model Predictive Path Integral）采样式预测控制。它以 PN 加速度为名义控制序列，在预测窗口内随机采样多条加速度序列，批量前向仿真后根据代价的指数权重对第一步控制做加权平均。

代价函数复用 NMPC 的目标：最终距离、窗口内路径距离、FOV 违反、控制能量、平滑性和偏离 PN 趋势。为了保持可复现性，MPPI 随机数由 `GuidanceConfig.mppi_seed` 固定；为了避免全场景仿真过慢，滚动预测使用 NumPy 批量计算。

当前参数下，MPPI 通常能显著降低 FOV 丢失时间，同时保持较快捕获，但代价是计算量高于 PN/CBF。

## 输出目录

运行后默认生成：

```text
outputs/
  stationary/
  linear/
  circle/
```

每个场景目录包含以下文件。

### `replay.mcap`

可选输出文件，仅在运行时传入 `--export-mcap` 生成。它包含 Foxglove 可读取的 3D 场景和各算法 telemetry，用于时间线动画回放和交互式算法选择。

### `trajectory_3d.png`

三维轨迹对比图。黑色虚线是目标轨迹，其余曲线是各算法下追踪机轨迹。图中可以观察：

- 追踪机是否成功接近目标
- 路径是否绕远
- PN 是否比基础追踪更直接
- FOV/CBF/NMPC/MPPI 是否为了保持视场产生不同机动

### `trajectory_xy.png`

水平面俯视图，只看 x-y 平面运动。它适合观察横向拦截效果，尤其是 `linear` 和 `circle` 场景中，能更清楚地看到算法是否提前切入目标运动方向。

### `trajectory_xz.png`

侧视图，只看 x-z 平面运动。它用于检查高度响应，例如追踪机是否合理爬升/下降，以及是否触碰 `z_min`、`z_max` 高度限制。

### `distance_error.png`

距离误差曲线，纵轴是追踪机和目标之间的距离。虚线 `Capture radius` 是捕获半径；曲线第一次低于该线的时间就是 `capture_time`。

读图时重点看：

- 曲线下降速度：越快说明收敛越快
- 最小距离：越小说明越接近目标
- 后续是否再次增大：可能表示飞过目标或追踪不稳定

### `fov_visibility.png`

FOV 可见性图。上半部分是 LOS 夹角，下半部分是 `visible` 的 0/1 状态。

读图方式：

- LOS 夹角低于 FOV 半角时，目标在视场内
- LOS 夹角高于 FOV 半角时，FOV 算法认为目标丢失
- `visible = 0` 的时间越长，`lost_duration` 越大

对于没有 FOV 约束的 `Direct pursuit` 和 `3D PN`，结果按理想传感器处理，目标始终可用于导引。

### `acceleration.png`

控制加速度模长曲线。它展示每种算法需要多强的控制输入。

读图时可以关注：

- 峰值是否接近加速度上限
- 控制是否频繁剧烈变化
- CBF/NMPC/MPPI 是否因为视场、控制和平滑代价而更保守

### `metrics.png`

指标柱状图，汇总各算法在同一场景下的表现。由于不同指标量纲不同，每个指标单独一个小图。

### `metrics.csv`

指标表，适合后续做论文表格或进一步分析。字段含义：

- `capture_time`：第一次进入捕获半径的时间；未捕获则记为仿真总时长
- `min_distance`：整个仿真中的最小距离
- `mean_distance`：平均距离误差
- `path_length`：追踪机轨迹长度
- `control_energy`：控制能量近似值 `sum(||a_cmd||^2 * dt)`
- `lost_duration`：目标不在视场内的累计时间
- `visible_ratio`：目标可见时间占总时间比例
- `yaw_rate_mean`：平均 yaw 角速度
- `yaw_rate_variance`：yaw 角速度方差

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

### 手动启动边界

先人工完成：

- 启动 Gazebo 仿真世界。
- 在 Gazebo 中创建两架 PX4 vehicle。
- 启动对应 PX4 SITL 实例。
- 启动 Micro XRCE-DDS Agent，确保 `px4_msgs` 话题桥接到 ROS 2。
- 连接 QGC 并确认两架机状态正常。
- 再启动本 ROS 2 节点。

默认约定：

- `/px4_1` 是追踪机。
- `/px4_2` 是目标机。
- namespace、算法、场景和控制频率都可通过 launch 参数覆盖。

### 构建与启动

从 `6_Simulation` 目录构建：

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
