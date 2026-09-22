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
# 首次构建：本仓库不跟踪 src/px4_msgs，需用 --packages-up-to 把 px4_msgs 一并编译（约 3~4 分钟）
colcon build --packages-up-to gazebosimulation2d --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 增量编译：install/ 中已有 px4_msgs 后，只编译导引包
colcon build --packages-select gazebosimulation2d
```

> `src/px4_msgs` 不随仓库跟踪，需自行放入，且**必须与所用 PX4 版本一致**：开发机 PX4 v1.16 对应 `release/1.16`（`392e831`）。版本不一致时，字段布局变化的 `VehicleLocalPosition` 会被 Fast DDS 直接丢弃（订阅端 0 帧，日志刷 `RTPS_READER_HISTORY: payload 220 > history 207`）；本包导引与视觉链路只订 `vehicle_odometry`，两边布局一致，不受影响。

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

### QGroundControl 双机接入备忘录（WSL2 + Windows）

PX4 SITL 的 GCS MAVLink 本地端口是 `18570 + 实例号`（`ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`）：追踪机（实例 0）在 `18570`，目标机（实例 1）在 `18571`。两个实例都只把心跳发往 `127.0.0.1:14550`，而 WSL2 默认 NAT 模式下 **WSL2 → Windows 的 `127.0.0.1` UDP 不通**（实测 Windows 侧监听收不到任何包），所以 Windows 上跑的 QGC 只能靠手动链路主动连 WSL 的 IP，且**每个实例一条**：只配一条时 QGC 只会识别到该端口对应的那一架（例如只配 `18570` 就只看到追踪机）。配置方法：

1. QGC → 应用设置（Application Settings）→ Comm Links → Add：
   - 追踪机：Name `WSL-1`，Type `UDP`，Listening Port `18570`，Target Host = WSL IP，Target Port `18570`，勾选 Automatically Connect。
   - 目标机：Name `WSL-2`，同上，端口改为 `18571`。
2. WSL IP 用 `hostname -I` 取第一个地址（本机曾为 `172.23.197.198`）。

注意：

- WSL IP 在 NAT 模式下每次 `wsl --shutdown` 后都可能变化，变了要同步改 QGC 的 Target Host。

当前 2D Gazebo 接入行为：

- 目标机使用位置 + 速度 setpoint 跟随 `pythonsimulation2d.target_state` 生成的二维参考轨迹。
- 追踪机准备/解锁阶段参考 `6_Simulation`：只发布当前位置 hold setpoint，不提前执行导引。
- 追踪机进入追踪阶段后使用速度 + 加速度 setpoint；二维导引输出的水平加速度作为 PX4 acceleration 前馈发布，position 字段不启用。
- 导引、记录距离和指标均按 XY 平面计算；追踪阶段 z 速度和 z 加速度指令为 0。
- `pursuer_fixed_altitude` 默认 8m，用于 2D 仿真配置和结果标注；当前追踪阶段不再通过 position setpoint 强制拉高度。

开启 0.2s 周期 ROS 调试日志：

```bash
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn_nmpc \
  scenario:=circle \
  pursuer_fixed_altitude:=8.0 \
  sim_time:=40.0 \
  debug_log:=true
```

调试日志以 `debug_2d` 开头，包含目标机/追踪机 odometry 状态，以及目标机参考指令和追踪机速度 + 加速度控制指令。日志周期可通过 `debug_log_period_s` 调整：

```bash
ros2 launch gazebosimulation2d guidance.launch.py debug_log:=true debug_log_period_s:=0.1
```

## 下视相机与视觉 truth 旁路

本轮只完成旁路基础设施：追踪机下视相机可订阅、纯几何投影可离线测试、ROS 伪检测可转换成独立位置量测。**导引闭环不消费视觉量测，不实现 YOLO，视觉不控制飞机。**

依赖（由使用者安装）：

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-vision-msgs ros-jazzy-rqt-image-view
```

追踪机改用 PX4 自带 `x500_mono_cam_down`（airframe 4014），目标机不变。外部终端启动追踪机示例：

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4014 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_UXRCE_DDS_NS=px4_1 \
./build/px4_sitl_default/bin/px4 -i 0
```

启动桥接和 truth 适配器：

```bash
# 只启用相机桥接（/camera/image_raw + /camera/camera_info）
ros2 launch gazebosimulation2d guidance.launch.py enable_camera:=true

# 启用 truth 适配器；不强制启用仓库桥接，允许外部桥接提供 CameraInfo
ros2 launch gazebosimulation2d guidance.launch.py vision_source:=truth

# 两者一起
ros2 launch gazebosimulation2d guidance.launch.py enable_camera:=true vision_source:=truth
```

默认 `enable_camera:=false`、`vision_source:=off`，启动行为与之前一致。桥接配置 `config/camera_bridge.yaml` 固定了实测 gz 话题名（world 名或模型实例名变化时需同步修改）。

验收命令：

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
# 图像是 SENSOR_DATA（BEST_EFFORT）桥接，echo 需要匹配 QoS 才收得到
ros2 topic echo --once --qos-reliability best_effort --field encoding /camera/image_raw
ros2 run rqt_image_view rqt_image_view /camera/image_raw
gz stats   # 记录 RTF；RTF 不足 1 时壁钟帧率会低于 30 Hz
```

已核验：相机 link 相对机体的安装平移为 `(0, 0, 0.10)` m、绕 y 轴 90°；Gazebo 相机为 1280×960、水平 FOV 1.74 rad、30 Hz、RGB_INT8；桥接 `frame_id` 覆盖为 `camera_link_optical`。30 Hz 是仿真时间频率，RTF 不足 1 时壁钟观测频率会更低。

输出：

- `/camera/detections_truth`：`vision_msgs/Detection2DArray`，`class_id="drone"`、score=1；bbox 中心有效，尺寸仅为占位。
- `/vision/target_pose`：`geometry_msgs/PoseWithCovarianceStamped`，frame=`enu`；姿态用单位四元数占位，不提供姿态观测。
- `outputs/gazebo2d_vision/vision_samples.csv`：每个定时器周期一行，含有效标志、拒绝原因、odometry 年龄/配对诊断和像素/位置数据。视觉 CSV 没有图像级检测延迟或独立像素误差，`plot_gazebo_csv.py` 不用于该文件。

视觉节点参数（launch 参数同名，`vision_` 前缀仅用于与导引节点重名的调试参数）：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `vision_source` | off | 本轮只支持 off/truth，其他值报错 |
| `enable_camera` | false | 仅 launch 使用，控制桥接，不控制相机渲染 |
| `camera_frame_id` | camera_link_optical | 与 `camera_bridge.yaml` 的 `frame_id` 一致 |
| `camera_mount_xyz` | [0.0, 0.0, 0.10] | 相机 link 相对机体 FLU 的安装平移 |
| `camera_mount_rpy_deg` | [0.0, 90.0, 0.0] | 安装旋转，不含光学轴转换 |
| `truth_rate_hz` | 10.0 | 独立定时器，不与图像同频 |
| `pose_timeout_s` / `pose_pair_tolerance_s` | 0.2 / 0.05 | 接收时间新鲜度与两机配对容差 |
| `pixel_noise_px` | 3.0 | 协方差假设，不代表实际添加随机噪声 |
| `target_plane_sigma_m` | 0.1 | 输出 z 不确定度假设 |
| `vision_record_data` / `vision_record_output_dir` | true / outputs/gazebo2d_vision | CSV 开关与目录 |
| `vision_debug_log` / `vision_debug_log_period_s` | false / 0.5 | 映射到节点 `debug_log` / `debug_log_period_s` |

离线几何与坐标测试（不需要 ROS/PX4）：

```bash
uv run python tests/test_camera_geometry.py
```

truth 只是 odometry 参考位置的自洽旁路，**不验证渲染、目标识别、图像同步或 YOLO 精度**；真实图像外参验证需要独立观测，目前未验证。时间同步、FOV 丢失处理和视觉闭环均在后续计划中。

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
