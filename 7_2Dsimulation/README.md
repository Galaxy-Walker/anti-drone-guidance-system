# 7_2Dsimulation

`7_2Dsimulation` 是二维定高俯瞰追踪仿真。追踪机固定高度飞行，导引和指标按 XY 平面计算，不建立完整的深度/FOV 可见性模型；EMPC（`pn_nmpc`）的代价函数包含固定下视相机的软性画面保持（FOV）惩罚，目标接近画幅边缘时主动回中（见 [算法说明](docs/2d_simulation_guidance_overview.md) 7.4.1 节）。追踪机下视单目相机的 YOLO 视觉闭环已接入（`vision_source:=yolo` + `target_source:=vision`），启动方式与参数见下文。

## 目录

- `main.py`：纯 Python 离线仿真入口，运行后生成指标 CSV 和图片。
- `plot_gazebo_csv.py`：Gazebo 记录 CSV 后处理，生成与离线仿真同类的指标和图片；轨迹图用论文版式网格图。
- `plot_vision_csv.py`：视觉链路 CSV 后处理，生成检测率、量测误差和时序图。
- `src/pythonsimulation2d/`：2D 目标、动力学、导引、估计器和绘图代码。
- `src/gazebosimulation2d/`：ROS2/PX4/Gazebo Offboard 接入包，含视觉检测、适配与相机记录节点。
- `tools/vision_offline_eval.py`：YOLO 数据集离线评估（Recall、像素误差）。
- `tools/prelabel_yolo_images.py`：合并相机截图，用已有权重生成供人工修正的 YOLO 预标注。
- `tools/vision_live_view.py`：实时查看相机画面与检测框（发布标注图供 rqt_image_view）。
- `tests/`：纯 Python 相机几何、目标估计器与 EMPC 画面保持（FOV）惩罚测试。
- `worlds/default.sdf`：视觉实验用无阴影 Gazebo 世界。
- `worlds/table_occlusion.sdf`：桌下遮挡世界，配合 `scenario:=table_occlusion`。
- `outputs/`：默认仿真输出目录（生成物不入库）。
- [算法说明](docs/2d_simulation_guidance_overview.md)：算法原理与已有结果。
- [视觉设计参考](docs/vision_design.md)：相机几何、进程协议与消息约定。
- [视觉验证记录](docs/yolo_vision_closed_loop_results.md)：离线验证结果与 2026-10-04 闭环复测记录。

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

> `src/px4_msgs` 不随仓库跟踪，需自行放入，且**必须与所用 PX4 版本一致**：开发机 PX4 v1.16 对应 `release/1.16`（`392e831`）。版本不一致时，字段布局变化的 `VehicleLocalPosition` 会被 Fast DDS 直接丢弃（订阅端 0 帧，日志刷 `RTPS_READER_HISTORY: payload 220 > history 207`）；本包在 px4_msgs 消息里只订阅 `vehicle_odometry`，两边布局一致，不受影响。

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

### 不启动 QGC 直接起飞（NAV_DLL_ACT）

QGC 只用于监控，解锁和 Offboard 由导引节点自己发送（`auto_arm` / `auto_offboard` 默认开启）；但在没有任何 MAVLink 地面站发送 GCS 心跳时，两机都会被 PX4 拒绝解锁：

```text
Preflight Fail: No connection to the ground control station
Arming denied: Resolve system health failures first
```

原因是 Gazebo 机型 4001 的 airframe 设了 `param set-default NAV_DLL_ACT 2`（`ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`，4014 会 source 4001、同样继承），而 `NAV_DLL_ACT > 0` 时 PX4 把收到过 GCS 心跳作为解锁前置条件（`src/modules/commander/HealthAndArmingChecks/checks/rcAndDataLinkCheck.cpp`）：commander 启动时 `gcs_connection_lost` 为 true，只有 GCS 心跳能清除它。本仓库的 ROS 2 链路走 XRCE-DDS、不产生 MAVLink 心跳；导引节点预热后只发一次 ARM、不检查 ack 也不重试，所以会一直停在准备阶段（`startup_2d ... pursuer_ready=false`），不会起飞。

不接 QGC 时，在每个实例的 PX4 终端（`pxh>`）各执行一次。要在 `ros2 launch` 首次发 ARM 之前完成；已经被拒的话，改完参数后重启 launch：

```text
param set NAV_DLL_ACT 0
param show NAV_DLL_ACT
param save                # 可选，参数变更后 PX4 会自动保存
```

参数按实例存储（`PX4-Autopilot/build/px4_sitl_default/rootfs/<实例号>/parameters.bson`），设置一次后重启仍生效。代价是关闭 GCS 链路丢失失效保护，仅用于 SITL，不要照搬到 8_MoCap 真机。

想保留 failsafe 时可提供任意 MAVLink GCS 心跳源，不必是 QGC：在 WSL 里运行 Linux 版 QGC（默认连 `127.0.0.1:14550`，绕开 Windows NAT 问题），或用 pymavlink 定时向 `127.0.0.1:18570` / `18571` 发送 `HEARTBEAT`（`MAV_TYPE_GCS`）。仓库提供等价的纯标准库脚本，无需安装 pymavlink：`python3 tools/px4_gcs_heartbeat.py`（默认向 18570/18571 各发 1 Hz 心跳，Ctrl-C 退出）。`COM_DLL_EXCEPT` 只在飞行中生效，解锁检查仍然看 `NAV_DLL_ACT`，不能解决本问题。

当前 2D Gazebo 接入行为：

- 目标机使用位置 + 速度 setpoint 跟随 `pythonsimulation2d.target.target_state()` 生成的二维参考轨迹。
- 追踪机准备/解锁阶段参考 `6_Simulation`：只发布起飞保持点 setpoint，不提前执行导引；`target_source=vision` 时该保持点取场景起点上方（初始捕获），odometry 模式仍为当前 spawn 位置。
- 追踪机进入追踪阶段后使用速度 + 加速度 setpoint；二维导引输出的水平加速度作为 PX4 acceleration 前馈发布，position 字段不启用。
- 导引、记录距离和指标均按 XY 平面计算；追踪阶段 z 速度和 z 加速度指令为 0。
- `pursuer_fixed_altitude` 默认 8m，用于 2D 仿真配置和结果标注；当前追踪阶段不再通过 position setpoint 强制拉高度。
- `target_speed_scale`（默认 1.0）只缩放目标机参考轨迹的速度：`circle` 缩放角速度（半径不变）、`linear` 缩放速度矢量、`table_occlusion` 缩放巡航速度（默认 0.5 m/s）、`stationary` 不受影响；起止点与控制算法参数不变。

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

视觉模式使用仓库内置的**无阴影世界** `worlds/default.sdf`：`<scene><shadows>` 与太阳 `cast_shadows` 都改为 `false`，其余与 PX4 v1.16 的 `default.sdf` 逐字一致。原因：下视相机 8 m 高度、目标平面 1 m 时太阳仰角约 51°，两架无人机的影子会偏移约 5.7 m 投到画面里，YOLO 容易把影子误检成目标。世界名保持 `default`，`src/gazebosimulation2d/config/camera_bridge.yaml` 的 `/world/default/...` 话题不变。

**Gazebo 先于 PX4 手动启动**：PX4 检测到已运行的世界后不会再拉起 Gazebo/GUI，两机只做连接；这样也便于把无阴影世界固定为实验配置。手动启动时 PX4 进程需要自己 source `gz_env.sh`（`px4-rc.gzsim` 只在由它拉起 Gazebo 的分支里 source，否则 `PX4_GZ_MODELS` 为空、模型 spawn 会失败）：

```bash
# 终端 1（可选）：QGC 监控两机（不接 QGC 时按上一节设置 NAV_DLL_ACT）；WSL2 下按 QGC 备忘录配置 18570/18571 两条链路

# 终端 2：Micro XRCE-DDS Agent（先于 PX4 启动）
MicroXRCEAgent udp4 -p 8888

# 终端 3：Gazebo（无阴影世界；仅 server，需要 GUI 时去掉 -s）
export GZ_CONFIG_PATH=/usr/share/gz
cd /home/srcbit/anti-drone/PX4-Autopilot
source build/px4_sitl_default/rootfs/gz_env.sh
gz sim -r -s /home/srcbit/anti-drone/anti-drone-guidance-system/7_2Dsimulation/worlds/default.sdf
# 等本终端打印 "Serving world [default]"（或至少无报错）后再启动 PX4

# 终端 4：追踪机（airframe 4014 = x500_mono_cam_down，实例 0，/px4_1）
export GZ_CONFIG_PATH=/usr/share/gz
cd /home/srcbit/anti-drone/PX4-Autopilot
source build/px4_sitl_default/rootfs/gz_env.sh
PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL_POSE="48,0,0,0,0,0" PX4_UXRCE_DDS_NS=px4_1 \
  ./build/px4_sitl_default/bin/px4 -i 0
# 等本终端出现 "INFO  [init] Gazebo world is ready" 和 "Spawning model" 后再启动目标机

# 终端 5：目标机（airframe 4001 = x500，实例 1，/px4_2）
export GZ_CONFIG_PATH=/usr/share/gz
cd /home/srcbit/anti-drone/PX4-Autopilot
source build/px4_sitl_default/rootfs/gz_env.sh
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="47,0,0,0,0,0" PX4_UXRCE_DDS_NS=px4_2 \
  ./build/px4_sitl_default/bin/px4 -i 1
```

说明：

- Gazebo 由终端 3 手动启动，PX4 两机都只连接：实例 0 自动检测已运行的世界，实例 1 带 `PX4_GZ_STANDALONE=1`；不会再起 server 造成冲突。
- 两机 spawn 的 XY 决定各自 PX4 本地原点。现在可通过 `pursuer_origin_xy` / `target_origin_xy` 指定出生点的世界 ENU XY：导引节点与视觉适配器统一转换到公共 ENU，位置 setpoint 再转换回各机本地 NED。默认 `[0.0, 0.0]` 保持旧行为；上述 `48,0` / `47,0` 示例应分别传入 `'[48.0, 0.0]'` / `'[47.0, 0.0]'`。旧记录未做转换，跨机误差含约 1 m 的原点偏差（见 [视觉验证记录](docs/yolo_vision_closed_loop_results.md) 4.1），新增参数不会修正历史 CSV。
- `target_source=vision` 时追踪机准备阶段会自动飞至场景起点上方（circle 为公共 ENU `(47, 0)`，即 `circle_center + (12, 0)`），目标机同时被送往同一起点；传入正确原点后，两机在世界中的实际 XY 也相同。
- airframe 自带 `PX4_GZ_WORLD=default`，所以 `src/gazebosimulation2d/config/camera_bridge.yaml` 里的 `/world/default/model/x500_mono_cam_down_0/...` 话题名成立；**不要再额外传 `PX4_SIM_MODEL`**（例如 `PX4_SIM_MODEL=gz_x500` 会把 4014 的相机模型覆盖成 `x500`，桥接就收不到图像）。
- 新环境首次运行相机机型前需 `make px4_sitl gz_x500_mono_cam_down`（本机 SITL 已编译）。
- 两个实例的 GCS MAVLink 本地端口分别是 18570/18571（见上一节 QGC 备忘录）。
- 不想手动起 Gazebo 时，也可把 `worlds/default.sdf` 复制覆盖到 PX4 的 `Tools/simulation/gz/worlds/default.sdf`，仍按旧流程由 PX4 拉起；缺点是 PX4 checkout 会带本地改动。

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
  yolo_model_path:=/home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.engine \
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

### vision_adapter 参数（yolo 模式）

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

### camera_recorder（相机画面记录，排查 YOLO 用）

`camera_recorder` 独立订阅图像并按固定周期落盘 JPEG，不依赖 `vision_detector` 与 YOLO worker，也不参与导引。在真值/odometry 制导下记录追踪机实际看到的画面，用于区分“YOLO 检测失败”是画面里没有目标、还是模型识别不到；也可以用它确认无阴影世界是否生效（画面里只应有目标机机体，不应有偏移约 5.7 m 的黑色影子）。

| launch 参数 | 节点参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `record_camera` | — | false | 是否启动记录节点 |
| `camera_image_topic` | `image_topic` | /camera/image_raw | 订阅的图像话题 |
| `camera_record_output_dir` | `output_dir` | outputs/gazebo2d_vision/camera_frames | 输出目录 |
| `camera_record_hz` | `save_hz` | 1.0 | 保存频率（按图像 `header.stamp` 节流） |
| `camera_jpeg_quality` | `jpeg_quality` | 90 | JPEG 质量 |
| `camera_max_frames` | `max_frames` | 0 | 保存上限，0 表示不限制 |

文件名使用图像 `header.stamp`（`<stamp_ns>.jpg`），与数据集帧命名一致；重启后已存在的文件跳过。

```bash
# 真值制导 + 1 Hz 画面记录（不启动 YOLO）
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn scenario:=circle \
  enable_camera:=true vision_source:=truth target_source:=vision \
  record_camera:=true record_output_dir:=outputs/gazebo2d_vision_runs
```

记录后可用 worker 自检单帧，快速区分“画面问题”与“模型问题”：

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python src/gazebosimulation2d/scripts/yolo_worker.py \
  --model /home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.engine \
  --self-test outputs/gazebo2d_vision/camera_frames/<stamp_ns>.jpg
```

### 实时查看画面与检测框（tools/vision_live_view.py）

`vision_detector` 只发布检测框数据，不发布标注图。`tools/vision_live_view.py` 是独立的监控旁路：订阅图像与检测，按 `header.stamp` 回查同一帧画面（检测节点的 stamp 就是它处理的那帧图像 stamp），用 OpenCV 画框、中心点和分数，再发布 `/camera/image_annotated`；不进入导引链路，未检出时画面原样透传。用系统 Python 运行，不需要 colcon build（需要 `numpy`、`cv2`）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/vision_live_view.py

# 另开终端
ros2 run rqt_image_view rqt_image_view /camera/image_annotated
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--image-topic` | /camera/image_raw | 相机图像话题 |
| `--detections-topic` | /camera/detections | 检测话题；truth 模式传 /camera/detections_truth |
| `--output-topic` | /camera/image_annotated | 标注图输出（bgr8，RELIABLE + KEEP_LAST(1)） |
| `--cache-size` | 60 | 图像缓存帧数（检测按 stamp 回查） |
| `--match-tolerance-s` | 0.05 | 没有严格同 stamp 帧时允许配对的最近帧时间差；0 表示只接受严格同 stamp |
| `--show` | false | 额外打开 OpenCV 窗口（WSL2 需 WSLg） |

输出频率约等于 `yolo_process_hz`（检测节点每个处理帧都会发消息），显示的是与框同 stamp 的那帧画面，不会把上一帧的框画到当前帧上；truth 伪检测的 stamp 来自生成时刻，走 0.05 s 容差的最近帧配对。

### 输出

三个 CSV 输出参数 `record_output_dir`、`vision_record_output_dir`、`yolo_stats_csv`（节点参数为 `stats_csv`）的相对路径统一以 `7_2Dsimulation/` 为基准。因此从仓库根目录启动时，`outputs/...` 也会写入 `7_2Dsimulation/outputs/...`；显式绝对路径保持不变，`ros2 run` 直接运行节点也遵循同一规则。数据集和截图路径仍相对于启动目录，建议按本文示例在 `7_2Dsimulation/` 下运行。

导引记录（节点退出时保存）：

```text
outputs/gazebo2d/<scenario>/<algorithm>/gazebo_samples.csv               # 默认记录 odometry 基线
outputs/gazebo2d_vision_runs/<scenario>/<algorithm>/gazebo_samples.csv   # 视觉闭环示例，避免覆盖 odometry 基线
```

话题与视觉输出：

- `/camera/detections`：`vision_msgs/Detection2DArray`（BEST_EFFORT/VOLATILE），bbox 中心为原图坐标，`class_id="drone"`，未检出发布空数组。
- `/camera/detections_truth`：truth 伪检测，score=1，bbox 尺寸仅为占位。
- `/camera/image_annotated`：`tools/vision_live_view.py` 输出的实时标注图（bgr8，RELIABLE），仅在该工具运行时存在。
- `/vision/target_pose`：`geometry_msgs/PoseWithCovarianceStamped`，frame=`enu`；XY 协方差来自像素噪声传播，姿态用单位四元数占位。
- `outputs/gazebo2d_vision/vision_samples.csv`：逐量测/拒绝记录（truth/yolo 共用；truth 行的新增列全为 NaN）。
- `outputs/gazebo2d_vision/yolo_detections.csv`：逐处理帧的检测与延迟。
- `outputs/gazebo2d_vision_runs/<scenario>/<algorithm>/gazebo_samples.csv`：导引记录（含 `target_est_*`、`vision_valid`、`vision_error_xy` 等列；`target_x/y` 仍是 odometry 真值）。
- `outputs/gazebo2d_vision/dataset/{frames,labels}/`：P3 门槛评估与 P7 微调数据。
- `outputs/gazebo2d_vision/camera_frames/<stamp_ns>.jpg`：`camera_recorder` 的周期截图（`record_camera:=true` 时），不依赖 YOLO。

### 数据集采集与 P3 零样本门槛评估

闭环跑到目标场景后另开终端，按 `save_frame_hz == process_hz` 采集（保证帧与标注一一对应），覆盖不同相位；正样本 ≥ 300 帧、背景帧 ≥ 100 帧：

```bash
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn scenario:=circle \
  enable_camera:=true vision_source:=yolo target_source:=vision use_sim_time:=true \
  yolo_python:=/home/srcbit/miniconda3/envs/ultralytics/bin/python \
  yolo_model_path:=/home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.engine \
  record_dataset:=true yolo_save_frame_hz:=10.0 \
  record_output_dir:=outputs/gazebo2d_vision_runs
```

离线评估（conda python，不需要 ROS；`--dataset` 指向 `.../dataset`）：

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python tools/vision_offline_eval.py \
  --dataset outputs/gazebo2d_vision/dataset \
  --model /home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.engine \
  --output outputs/gazebo2d_vision/eval
```

输出 `eval_report.csv` / `eval_report.md`：Recall@IoU0.3/0.5、匹配框中心像素误差 p50/p95、背景帧误检，并扫描 conf ∈ [0.1, 0.5]。门槛：Recall@IoU0.3 ≥ 0.8（conf=0.25）可直接闭环；0.5～0.8 降 conf + 门控后继续；< 0.5 进入域适配微调。

### 相机截图合并与微调预标注

用已有 conda `ultralytics` 环境运行，不需要 ROS，也不向根 uv 项目添加依赖。在 `7_2Dsimulation` 下执行：

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python tools/prelabel_yolo_images.py \
  --source outputs/table_occlusion_frames \
  --runs run1 run2 \
  --weights /home/srcbit/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.pt \
  --output /home/srcbit/table_occlusion_prelabel \
  --conf 0.10 --imgsz 640 --device 0
```

脚本复制原图，将两个批次展平到 `images/`，文件名加 `run1_` / `run2_` 前缀以避免重名；`labels/` 保存每张图片同名的五列 YOLO 归一化标签（不含置信度），没有检测时也创建空标签；`previews/` 保存画框预览，`classes.txt` 保留模型类别顺序，`manifest.csv` 记录源文件映射与检测数量，`summary.json` 记录推理配置与汇总。

默认 `conf=0.10` 用于减少预标注漏检，需人工删除误检、补充漏检并修正框。空标签须复核，不能直接当作背景真值；人工标注和微调都使用 `images/` 原图，预览图仅供检查。人工修订完成后再划分训练集/验证集，避免相邻视频帧随机分到两边造成数据泄漏。

已有输出目录会被拒绝，以保护人工修改过的标签。重跑时用 `--output` 指定新目录；快速检查可加 `--limit 16`，省略预览可加 `--no-previews`，CPU 推理可用 `--device cpu`。

### 绘图

`plot_gazebo_csv.py`（导引记录）与 `plot_vision_csv.py`（视觉链路）的用法和输出文件见 [记录后处理与绘图](#记录后处理与绘图)。

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
  --model /home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.engine \
  --self-test /home/srcbit/Det-Fly-YOLO-1third/images/val/0207134.jpg
```

已核验（2026-09-26，无头 Gazebo 实测）：相机 link 相对机体的安装平移为 `(0, 0, 0.10)` m、绕 y 轴 90°（光轴朝下）；Gazebo 相机为 1280×960、水平 FOV 1.74 rad、RGB_INT8，桥接输出 `rgb8`；`CameraInfo` 内参 `K=[539.936, 0, 640; 0, 539.936, 480]`、畸变 D 全 0、frame 覆盖为 `camera_link_optical`（`ros_gz_bridge` 1.0.24 支持逐桥接 `frame_id`/`qos_profile`/`lazy`）。30 Hz 是无相机负载下的仿真时间设定值；本机无头渲染走软件 EGL 路径，实测约 13–15 Hz，此时 RTF≈1.00。**不要用 `gz topic -e` 直接订阅图像话题**（实测会拖慢渲染并撑大 gz sim 进程直至 OOM）。

已知边界：

- `truth` 只是 odometry 参考位置的自洽旁路，不验证渲染、目标识别或 YOLO 精度；`pixel_error_vs_truth_px` / `position_error_vs_odom_m` 与量测同源，不是独立标定。
- 目标可见顶面比 1 m 控制平面高约 0.15 m，反投影有 1.4%～2.2% 的径向偏置，记录在指标中、不做隐蔽补偿。
- 未实现 TF、多目标跟踪、标注图发布（已由 `tools/vision_live_view.py` 旁路提供）、视觉伺服导引与远距离捕获；未做视觉链路与 odometry 基线的对比实验。

离线几何与坐标测试（不需要 ROS/PX4）：

```bash
uv run python tests/test_camera_geometry.py
uv run python tests/test_target_filter.py
uv run python tests/test_fov_penalty.py
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
stationary, linear, circle, table_occlusion
```

`pn_nmpc` 是候选枚举式预测控制（文档称 EMPC），代价函数除距离、控制、平滑和 PN 趋势项外还包含画面保持（FOV）惩罚：把预测目标投影到标称下视相机的图像平面，归一化偏移超过软边界后加重回中、接近边缘时惩罚最强。权重与相机参数在 `src/pythonsimulation2d/config.py` 的 `nmpc_w_fov`、`fov_*` 字段中，原理与 Gazebo 验证见 [算法说明](docs/2d_simulation_guidance_overview.md) 7.4.1 和 12.5 节。

## 桌下遮挡与重新找回目标

`table_occlusion` 使用不透明的 2 × 2 m 桌面（厚 0.15 m、下表面离地 2.5 m）与四条带碰撞体的桌腿。桌子中心是世界 ENU `(6, 0)`，目标机在 1 m 高度从 `(0, 0)` 沿 +X 飞至 `(12, 0)`。巡航速度默认 **0.5 m/s**，以 0.5 m/s² 的梯形速度参考起步、减速并停在桌子中心。参考到达后，目标实际位置误差 ≤ 0.15 m 且实际速度 ≤ 0.10 m/s 时开始计时，连续停稳 **3 秒仿真时间**后再继续前进；期间不稳定就重新计时。终点保持悬停。理想轨迹约 29 秒，默认记录 40 秒，留出停稳和出桌恢复的观察时间。

实验先使用 `pn_nmpc`（EMPC）。真实图像遮挡使 YOLO 漏检，追踪机沿用现有 `tracking → coast → lost/hold`；出桌后接受到视觉量测即恢复导引，不增加主动搜索或重获成功阈值。`truth` 旁路不消费图像、不处理桌子遮挡，不能用于此实验。纯 Python 的同名场景仅复用目标参考轨迹，不模拟真实桌面遮挡或实际停稳等待。

按前面的五个外部终端准备环境，做以下调整（Gazebo 世界名仍为 `default`，相机桥接话题无需更改）：

```bash
# Gazebo 终端：用新世界替换 default.sdf；不与旧世界同时运行
gz sim -r -s /home/srcbit/anti-drone/anti-drone-guidance-system/7_2Dsimulation/worlds/table_occlusion.sdf

# 追踪机终端：在 PX4 目录、source gz_env.sh 后运行，出生点错开 2 m
PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL_POSE="-2,0,0,0,0,0" PX4_UXRCE_DDS_NS=px4_1 \
  ./build/px4_sitl_default/bin/px4 -i 0

# 目标机终端：在 PX4 目录、source gz_env.sh 后运行
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_UXRCE_DDS_NS=px4_2 \
  ./build/px4_sitl_default/bin/px4 -i 1
```

在 `7_2Dsimulation/` 中构建并 source 后启动导引，原点参数必须与上述出生点配套：

```bash
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn_nmpc scenario:=table_occlusion \
  enable_camera:=true vision_source:=yolo target_source:=vision use_sim_time:=true \
  pursuer_origin_xy:='[-2.0, 0.0]' target_origin_xy:='[0.0, 0.0]' \
  target_speed_scale:=1.0 sim_time:=40.0 \
  yolo_python:=/home/srcbit/miniconda3/envs/ultralytics/bin/python \
  yolo_model_path:=/home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.engine \
  record_camera:=true debug_log:=true \
  record_output_dir:=outputs/gazebo2d_vision_runs \
  vision_record_output_dir:=outputs/table_occlusion_vision \
  yolo_stats_csv:=outputs/table_occlusion_vision/yolo_detections.csv

# 观察出桌后的检测与追踪恢复，满 40 秒后 Ctrl-C 保存导引记录，再绘图
uv run plot_gazebo_csv.py \
  outputs/gazebo2d_vision_runs/table_occlusion/pn_nmpc/gazebo_samples.csv
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `pursuer_origin_xy` | `[0.0, 0.0]` | 追踪机本地原点的世界 ENU XY，导引与视觉适配共用 |
| `target_origin_xy` | `[0.0, 0.0]` | 目标机本地原点的世界 ENU XY，导引与视觉适配共用 |

桌子几何与任务默认值在 `config.py` 的 `TableOcclusionConfig`；改变桌子尺寸/位置时也要同步 `worlds/table_occlusion.sdf`。这些 XY 原点参数适用于地面出生且 NED 轴向一致的当前双机配置，不包含旋转或高度原点补偿。

## 记录后处理与绘图

`plot_gazebo_csv.py` 用于把 Gazebo 记录的 `gazebo_samples.csv` 转成与纯 Python 仿真相同类型的指标和图片，不包含 FOV 相关输出。按场景目录汇总绘图：

```bash
uv run plot_gazebo_csv.py \
  outputs/gazebo2d_vision_runs/circle \
  --output-dir outputs/circle_vision
```

也可以只绘制单个算法的 CSV：

```bash
uv run plot_gazebo_csv.py \
  outputs/gazebo2d/circle/pn_mppi/gazebo_samples.csv
```

输出文件包括：

```text
metrics.csv
trajectories_2x2.png             # 论文版式轨迹网格图（单算法记录时为单面板）
distance_error.png
acceleration.png
yaw_rate.png
metrics.png
vision_estimate.png              # 单算法记录且含视觉估计列时
vision_estimate_<algorithm>.png  # 多算法场景目录且含视觉估计列时
```

所有 Gazebo 追踪场景的 `metrics.png` 用 **平均水平追踪误差**（`mean_distance`，m）替换拦截时间；其余三个面板保持最小距离、平均 yaw rate 和 yaw rate 方差。纯 Python 的 `metrics.png` 保留捕获时间，`metrics.csv` 仍保留 `capture_time` 列以兼容已有后处理。

`distance_error.png` 显示追踪机与目标机 odometry 真值的 XY 距离随时间变化。`table_occlusion` 的桌下样本按实际位置记录为 `target_under_table=1`：曲线留空、最小/平均距离排除这一段，其他控制指标使用完整记录。出桌后即恢复曲线，即使此时尚未重新检测到目标也保留误差，以显示恢复过程。旧 CSV 缺少桌下标志时按默认桌子几何和实际目标位置补算；其他场景使用全部有效距离样本。平均误差采用样本算术平均；整段均在桌下时距离指标为 NaN，柱状图标为 N/A。原始 `distance_xy` 始终完整保留，桌下标志不参与控制或检测。

轨迹图由 `src/pythonsimulation2d/publication_plots.py` 绘制：衬线字体、等比例面板、四个算法共用一组
坐标范围，尺寸按英寸排版（不受 `tight_layout` 拉伸）。其余面板沿用离线仿真的默认样式，两套样式互不影响。
默认画完整记录，需要截断时用 `--trajectory-window-s`：

```bash
# 只画前 20 s：圆周轨迹留有缺口、不闭合成整圆，docs/assets 里的插图就是这一口径
uv run plot_gazebo_csv.py \
  outputs/gazebo2d_vision_runs/circle \
  --output-dir outputs/circle_vision \
  --trajectory-window-s 20
```

视觉链路 CSV（检测率、像素残差、延迟、丢失时段、拒绝原因）由 `plot_vision_csv.py` 处理：

```bash
uv run plot_vision_csv.py outputs/gazebo2d_vision --output-dir outputs/vision_report
```
