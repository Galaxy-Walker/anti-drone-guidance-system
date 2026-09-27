# 7_2Dsimulation

`7_2Dsimulation` 是二维定高俯瞰追踪仿真。当前版本不使用深度相机和 FOV 约束，追踪机固定高度飞行，导引和指标按 XY 平面计算。

## 目录

- `main.py`：纯 Python 离线仿真入口，运行后生成指标 CSV 和图片。
- `src/pythonsimulation2d/`：2D 目标、动力学、导引、指标和绘图代码。
- `src/gazebosimulation2d/`：ROS2/PX4/Gazebo Offboard 接入包。
- `outputs/`：默认仿真输出目录。
- [算法说明](docs/2d_simulation_guidance_overview.md)：算法原理与已有结果。
- [视觉设计参考](docs/vision_design.md)：相机几何、进程协议与消息约定。
- [视觉验证记录](docs/yolo_vision_closed_loop_results.md)：历史验证结果与待实测项目。

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

## 下视相机与视觉闭环

追踪机下视相机 → YOLO 检测 → 反投影量测 → 导引的完整闭环。三种视觉来源：

| `vision_source` | 内容 | 是否参与导引 |
| --- | --- | --- |
| `off` | 不创建视觉节点 | 否 |
| `truth` | 用目标 odometry 参考位置投影成伪检测，走与 yolo 相同的反投影链路 | `target_source:=vision` 时是 |
| `yolo` | `vision_detector` 订阅图像，conda 常驻 worker 推理，输出 `/camera/detections` | `target_source:=vision` 时是 |

默认 `enable_camera:=false`、`vision_source:=off`、`target_source:=odometry`，启动方式与接入视觉前一致。导引节点在 `target_source:=vision` 时只消费 `/vision/target_pose`：漏检 → coast（α-β 预测），超时 → hold（悬停），不允许静默回退 odometry。

依赖（由使用者准备）：

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-vision-msgs ros-jazzy-rqt-image-view
# YOLO 推理在 conda ultralytics 环境；权重路径与设备由 launch 参数显式给出
```

追踪机使用 PX4 自带 `x500_mono_cam_down`（airframe 4014，Gazebo 模型实例 `x500_mono_cam_down_0`），目标机用 `x500`（airframe 4001，实例 `x500_1`）。以下终端由使用者手动启动，本仓库的 launch 不会拉起它们。

视觉闭环要求两机 spawn 在目标场景起点：circle 的起点是 `(47, 0)`（`circle_center + (12, 0)`，见 `pythonsimulation2d/config.py`）。起飞前两机都在地面等待（追踪机随后定高 8 m、目标机 1 m），所以两个 `PX4_GZ_MODEL_POSE` 相同：

```bash
# 终端 1（可选）：QGC 监控两机；WSL2 下按上一节备忘录配置 18570/18571 两条链路

# 终端 2：Micro XRCE-DDS Agent（先于 PX4 启动）
MicroXRCEAgent udp4 -p 8888

# 终端 3：追踪机（airframe 4014 = x500_mono_cam_down，实例 0，/px4_1）
# 无显示器时加 HEADLESS=1（无头渲染走软件 EGL，相机仍能出图）；需要 Gazebo GUI 时去掉
cd /home/srcbit/anti-drone/PX4-Autopilot
HEADLESS=1 PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL_POSE="47,0,0,0,0,0" PX4_UXRCE_DDS_NS=px4_1 \
  ./build/px4_sitl_default/bin/px4 -i 0
# 等本终端出现 "INFO  [init] Gazebo world is ready" 和 "Spawning model" 后再启动目标机

# 终端 4：目标机（airframe 4001 = x500，实例 1，/px4_2）
# PX4_GZ_STANDALONE=1：复用终端 3 已启动的 Gazebo，不再另起 server
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="47,0,0,0,0,0" PX4_UXRCE_DDS_NS=px4_2 \
  ./build/px4_sitl_default/bin/px4 -i 1
```

说明：

- 只由终端 3 启动 Gazebo；终端 4 必须带 `PX4_GZ_STANDALONE=1`，否则第二个实例可能再起一个 Gazebo server 造成冲突。
- airframe 自带 `PX4_GZ_WORLD=default`，所以 `config/camera_bridge.yaml` 里的 `/world/default/model/x500_mono_cam_down_0/...` 话题名成立；**不要再额外传 `PX4_SIM_MODEL`**（例如 `PX4_SIM_MODEL=gz_x500` 会把 4014 的相机模型覆盖成 `x500`，桥接就收不到图像）。
- 新环境首次运行相机机型前需 `make px4_sitl gz_x500_mono_cam_down`（本机 SITL 已编译）。
- 两个实例的 GCS MAVLink 本地端口分别是 18570/18571（见上一节 QGC 备忘录）。

启动桥接与视觉节点：

```bash
# 只启用相机桥接（/camera/image_raw + /camera/camera_info + /clock）
ros2 launch gazebosimulation2d guidance.launch.py enable_camera:=true

# truth 旁路：不消费图像，验证几何/消息链路
ros2 launch gazebosimulation2d guidance.launch.py enable_camera:=true vision_source:=truth

# YOLO 视觉闭环（目标机在外部终端同步启动）
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn scenario:=circle \
  enable_camera:=true vision_source:=yolo target_source:=vision use_sim_time:=true \
  yolo_python:=/home/srcbit/miniconda3/envs/ultralytics/bin/python \
  yolo_model_path:=/home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_caa_p3_dysample_detfly/weights/best.engine \
  record_output_dir:=outputs/gazebo2d_vision_runs
```

`use_sim_time:=true` 是视觉闭环的硬性要求：图像 stamp、位姿缓存和量测时间都以 `/clock`（Gazebo 仿真时间）为基准；检测/适配/导引三个节点都挂了墙钟防呆，2 s 内收不到 `/clock` 会打印 FATAL 并退出，而不是静默不动作。先启动 Gazebo 再启动 launch。

### vision_detector（YOLO 检测节点）

系统 Python 节点，不 import torch；推理在 conda 常驻子进程 `scripts/yolo_worker.py` 完成（stdin/stdout 二进制协议，`.engine` 加载失败时自动回落到 `model_fallback_path` 的 `.pt`）。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `yolo_python` | 空（必填） | conda ultralytics 环境 python |
| `yolo_worker_script` | share 内 `scripts/yolo_worker.py` | 留空使用安装副本，可覆盖为源码路径 |
| `model_path` | 空（必填） | `.engine` 优先 |
| `model_fallback_path` | 空 | 引擎加载失败时的 `.pt` |
| `imgsz` / `conf` / `iou` | 640 / 0.25 / 0.7 | ultralytics 推理参数 |
| `device` / `half` | 0 / true | GPU 与 FP16（引擎已内置 FP16，自动忽略 `half`） |
| `max_det` / `class_name` | 5 / uav | 单目标契约：只发布最高分一条（类别名大小写不敏感） |
| `process_hz` / `max_frame_age_s` | 10.0 / 0.2 | 节流与过期丢帧（忙时不排队） |
| `frame_format` | raw | `raw|jpeg`（jpeg 需要系统 Python 有 cv2） |
| `inference_timeout_s` / `worker_startup_timeout_s` / `worker_restart_limit` | 1.0 / 30.0 / 3 | 超时重启与上限 |
| `save_frame_hz` / `dataset_output_dir` | 0.0 / outputs/gazebo2d_vision/dataset | 数据集帧录制（0 关闭） |
| `stats_csv` | outputs/gazebo2d_vision/yolo_detections.csv | 逐处理帧统计 |
| `yolo_debug_log` / `yolo_debug_log_period_s` | false / 1.0 | launch 参数名（节点内为 `debug_log`） |

### vision_adapter 新增参数（yolo 模式）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `vision_source` | off | `off|truth|yolo`；yolo 要求 `use_sim_time=true` |
| `min_score` | 0.25 | 检测最低分 |
| `pose_match_tolerance_s` | 0.05 | 图像 stamp 与位姿缓存的时间容差；插值跨空洞超限也会拒绝 |
| `pose_cache_max_age_s` | 0.5 | 位姿缓存保留时长 |
| `pose_cache_interpolate` | true | 按图像 stamp 插值（false 用最近样本） |
| `extrapolate_pose` | false | 是否允许在最新样本之后外推（默认拒绝 `pose_cache_future`） |
| `record_dataset` | false | 数据集标注开关 |
| `vision_dataset_output_dir` | outputs/gazebo2d_vision/dataset | 标注目录（launch 参数名；节点内为 `dataset_output_dir`） |
| `dataset_label_box_size_m` | 0.35 | 标注用目标盒假设边长（仅离线标注） |

### 导引节点视觉参数（`target_source:=vision`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `target_source` | odometry | 导引输入：`odometry|vision` |
| `vision_topic` | /vision/target_pose | 视觉量测话题 |
| `vision_fallback` | none | 本轮只接受 none；odometry 预留给远距离捕获，设置会报错 |
| `vision_max_age_s` | 0.5 | 量测延迟超限视为无效 |
| `vision_alpha` / `vision_beta` | 0.85 / 0.25 | α-β 系数（β 需满足稳定域） |
| `vision_accel_tau_s` | 0.5 | 加速度低通时间常数（0 关闭前馈） |
| `vision_gate_sigma` | 0.0 | 马氏门控（0 关闭） |
| `vision_coast_s` / `vision_loss_s` | 0.3 / 1.0 | tracking → coast → lost 阈值 |
| `vision_hold_on_loss` | true | lost 后零速零加速度悬停、保持 yaw |
| `min_dt_s` / `max_dt_s` | 0.01 / 0.5 | α-β 的 dt 夹取范围 |

### 输出

- `/camera/detections`：`vision_msgs/Detection2DArray`（BEST_EFFORT/VOLATILE），bbox 中心为原图坐标，`class_id="drone"`，未检出发布空数组。
- `/camera/detections_truth`：truth 伪检测，score=1，bbox 尺寸仅为占位。
- `/vision/target_pose`：`geometry_msgs/PoseWithCovarianceStamped`，frame=`enu`；XY 协方差来自像素噪声传播，姿态用单位四元数占位。
- `outputs/gazebo2d_vision/vision_samples.csv`：逐量测/拒绝记录（truth/yolo 共用；truth 行的新增列全为 NaN）。
- `outputs/gazebo2d_vision/yolo_detections.csv`：逐处理帧的检测与延迟。
- `outputs/gazebo2d_vision_runs/<scenario>/<algorithm>/gazebo_samples.csv`：导引记录（含 `target_est_*`、`vision_valid`、`vision_error_xy` 等列；`target_x/y` 仍是 odometry 真值）。
- `outputs/gazebo2d_vision/dataset/{frames,labels}/`：P3 门槛评估与 P7 微调数据。

### 数据集采集与 P3 零样本门槛评估

闭环跑到目标场景后另开终端，按 `save_frame_hz == process_hz` 采集（保证帧与标注一一对应），覆盖不同相位；正样本 ≥ 300 帧、背景帧 ≥ 100 帧：

```bash
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn scenario:=circle \
  enable_camera:=true vision_source:=yolo target_source:=vision use_sim_time:=true \
  yolo_python:=/home/srcbit/miniconda3/envs/ultralytics/bin/python \
  yolo_model_path:=.../best.engine \
  record_dataset:=true yolo_save_frame_hz:=10.0
```

离线评估（conda python，不需要 ROS；`--dataset` 指向 `.../dataset`）：

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python tools/vision_offline_eval.py \
  --dataset outputs/gazebo2d_vision/dataset \
  --model /home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_caa_p3_dysample_detfly/weights/best.engine \
  --output outputs/gazebo2d_vision/eval
```

输出 `eval_report.csv` / `eval_report.md`：Recall@IoU0.3/0.5、匹配框中心像素误差 p50/p95、背景帧误检，并扫描 conf ∈ [0.1, 0.5]。门槛：Recall@IoU0.3 ≥ 0.8（conf=0.25）可直接闭环；0.5～0.8 降 conf + 门控后继续；< 0.5 进入域适配微调。

### 绘图

```bash
# 导引记录：含估计列时自动追加 vision_estimate.png
uv run plot_gazebo_csv.py outputs/gazebo2d_vision_runs/circle --output-dir outputs/circle_vision

# 视觉链路：检测率、像素残差、延迟、丢失时段与拒绝原因
uv run plot_vision_csv.py outputs/gazebo2d_vision --output-dir outputs/vision_report
```

### 验收命令

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /clock
ros2 topic echo --once --qos-reliability best_effort --field encoding /camera/image_raw
ros2 topic hz /camera/detections            # ≈ yolo_process_hz
ros2 topic echo --once /camera/detections   # 字段/坐标正确
ros2 topic echo --once /vision/target_pose
ros2 run rqt_image_view rqt_image_view /camera/image_raw
gz stats   # 记录 RTF；RTF 不足 1 时壁钟帧率会低于 30 Hz
```

worker 离线自检（不需要 ROS/Gazebo，坐标应与 `ultralytics-main/predict_one_image.py` 一致）：

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python src/gazebosimulation2d/scripts/yolo_worker.py \
  --model .../best.engine --self-test /home/srcbit/Det-Fly-YOLO-1third/images/val/0207134.jpg
```

已核验（2026-09-26，无头 Gazebo 实测）：相机 link 相对机体的安装平移为 `(0, 0, 0.10)` m、绕 y 轴 90°（光轴朝下）；Gazebo 相机为 1280×960、水平 FOV 1.74 rad、RGB_INT8，桥接输出 `rgb8`；`CameraInfo` 内参 `K=[539.936, 0, 640; 0, 539.936, 480]`、畸变 D 全 0、frame 覆盖为 `camera_link_optical`（`ros_gz_bridge` 1.0.24 支持逐桥接 `frame_id`/`qos_profile`/`lazy`）。30 Hz 是无相机负载下的仿真时间设定值；本机无头渲染走软件 EGL 路径，实测约 13–15 Hz，此时 RTF≈1.00。**不要用 `gz topic -e` 直接订阅图像话题**（实测会拖慢渲染并撑大 gz sim 进程直至 OOM）。

已知边界：

- `truth` 只是 odometry 参考位置的自洽旁路，不验证渲染、目标识别或 YOLO 精度；`pixel_error_vs_truth_px` / `position_error_vs_odom_m` 与量测同源，不是独立标定。
- 目标可见顶面比 1 m 控制平面高约 0.15 m，反投影有 1.4%～2.2% 的径向偏置，记录在指标中、不做隐蔽补偿。
- 未实现 TF、多目标跟踪、标注图发布、视觉伺服导引与远距离捕获；本轮不做算法对比实验。

离线几何与坐标测试（不需要 ROS/PX4）：

```bash
uv run python tests/test_camera_geometry.py
uv run python tests/test_target_filter.py
```

ROS 节点测试（假 worker，不需要 torch/GPU）：

```bash
colcon test --packages-select gazebosimulation2d && colcon test-result --verbose
```

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

视觉闭环建议用独立目录避免覆盖 odometry 基线（见 `record_output_dir`）：

```text
outputs/gazebo2d_vision_runs/<scenario>/<algorithm>/gazebo_samples.csv
```

## Gazebo CSV 绘图

`plot_gazebo_csv.py` 用于把 Gazebo 记录的 `gazebo_samples.csv` 转成与纯 Python 仿真相同类型的指标和图片，不包含 FOV 相关输出；记录含视觉估计列时（`target_est_x` 等）额外生成 `vision_estimate.png`。

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
vision_estimate.png   # 仅当 CSV 含视觉估计列
```

视觉链路 CSV（检测率、像素残差、延迟、丢失时段、拒绝原因）：

```bash
uv run plot_vision_csv.py outputs/gazebo2d_vision --output-dir outputs/vision_report
```
