# 7_2Dsimulation YOLO 视觉闭环仿真接入计划

> 状态：待实施。本计划接续 `docs/camera_vision_integration_plan.md` 已完成的 P1–P3（相机桥接、纯几何、truth 旁路），把
> `/home/srcbit/anti-drone/ultralytics-main` 的 YOLO26 检测器接入 `7_2Dsimulation` 的 PX4/Gazebo 闭环。
>
> 本轮任务定义：**追踪机全程保持在目标机正上方**，目标始终位于下视相机足印内；导引输入使用机载相机的视觉估计，
> **不使用目标 odometry 作为导引输入**。不做远距离捕获/视觉移交，不做 odometry/truth/多算法对比实验，不新增导引算法。
> 目标环境：ROS 2 Jazzy + PX4 v1.16 SITL + Gazebo Harmonic（gz-sim 8），与现有 2D Gazebo 接入一致。

## 1. 目标与判定标准

闭环数据流：

```text
追踪机 PX4 Offboard ──► x500_mono_cam_down 下视相机（gz imager）
                                │ ros_gz_bridge
                                ▼
                    /camera/image_raw + /camera/camera_info
                                │
                     vision_detector（新节点，系统 Python）
                     ├─ stdio 协议 ──► yolo_worker（conda ultralytics 环境，常驻）
                     └─ /camera/detections（vision_msgs/Detection2DArray）
                                │
        PX4 位姿缓存（按图像 stamp 插值取相机位姿）
                                ▼
                     vision_adapter（vision_source=yolo）
                     pixel_to_ground(z = target_base_altitude)
                                │
                     /vision/target_pose（ENU，PoseWithCovarianceStamped）
                                │
                     guidance_node_2d（target_source=vision）
                     α-β 估计 + coast/hold ──► compute_guidance()（算法不改）
                                │
                     PX4 速度 + 加速度 setpoint ──► 追踪机
```

闭环成立的判定标准：

1. **导引输入只来自视觉**：`target_source=vision` 时，`compute_guidance()` 的 `target` 状态只能来自 `/vision/target_pose`；
   目标 odometry 仅用于目标机自身控制、启动就绪判定与误差评估。
2. **时间基准一致**：相机位姿按图像 `header.stamp`（仿真时间）插值取出，而不是用“最新位姿”反投影；
   视觉链路节点统一 `use_sim_time=true`，`/clock` 由桥接提供。
3. **丢失有明确行为**：漏检/丢帧 → coast（预测），超时 → hold（悬停），全过程记录、可量化，
   不允许“静默改用 odometry 兜底”。
4. **默认行为不变**：`vision_source=off`、`target_source=odometry` 时，启动与运行方式与当前一致。
5. **验收产物**：一次 `circle + pn` 的闭环记录（CSV + 图 + 指标表）与可复现命令，能证明“相机 → 检测 → 量测 → 导引”整链路在环。

## 2. 现状盘点

### 2.1 已有基础（P1–P3，已提交）

| 组件 | 现状 |
| --- | --- |
| `config/camera_bridge.yaml` | `/camera/image_raw`、`/camera/camera_info` 两条桥接，gz 话题名已实测固定，`frame_id=camera_link_optical` |
| `src/pythonsimulation2d/camera_geometry.py` | `ground_to_pixel` / `pixel_to_ground` / 像素雅可比，含离线测试 |
| `coordinates.py` | FRD→FLU→光学系完整位姿与安装外参，`camera_pose_from_odometry()` |
| `vision_adapter.py` | truth 旁路：伪检测 + `/vision/target_pose` + `vision_samples.csv`；`vision_source` 目前仅支持 off/truth |
| `guidance_node.py` | 双机 Offboard 20 Hz、启动预热/解锁、记录 `gazebo_samples.csv`、`plot_gazebo_csv.py` 出图 |

### 2.2 缺口

- 没有检测节点；`vision_adapter` 不支持 yolo；`/clock` 未桥接；guidance 没有 `target_source` 开关；
- 没有目标估计器与 coast/hold 状态机；没有视觉指标与绘图；Gazebo 合成图的检测可行性未验证。

### 2.3 本机核查结果（2026-09-26 复核）

| 项 | 结果 |
| --- | --- |
| ROS 2 Jazzy、`ros-jazzy-cv-bridge`、`ros-jazzy-rqt-image-view` | 已安装 |
| `ros-jazzy-vision-msgs`、`ros-jazzy-ros-gz-bridge` | 已安装（4.1.1 / 1.0.24，`ros2 pkg prefix` 验证；`ros-gz-interfaces` 随依赖装入） |
| Gazebo Harmonic | gz-sim 8.15.0，`sensors-system` 与 `gz-rendering8-ogre2` 齐全；已用自建世界实测相机出图（1280×960）与 `/clock` 发布 |
| PX4-Autopilot | `/home/srcbit/anti-drone/PX4-Autopilot`（`release/1.16`，`v1.16.2-9-g8714f2442a`），25 个子模块完整，`x500_mono_cam_down`/`x500` 模型与 `4001/4014` airframe 均在；SITL 已编译（`build/px4_sitl_default/bin/px4`，2026-09-26） |
| MicroXRCEAgent | 已安装 v3.0.2（`/usr/local/bin/MicroXRCEAgent`；源码在 `/home/srcbit/anti-drone/Micro-XRCE-DDS-Agent`） |
| QGroundControl | 已安装 AppImage：`/home/srcbit/anti-drone/QGroundControl-x86_64.AppImage` |
| 构建依赖 / 工具 | `ninja` 1.11.1、cmake、gcc/g++-multilib、ccache 等已就位；仅 `exiftool` 未装（SITL 编译不需要） |
| `uv` | **未安装**；根项目 `.venv` 未建立，纯 Python 侧的 `uv sync` / `uv run`（P4 单测与绘图）前需先安装 |
| conda 环境 `ultralytics` | py3.11.16 + torch 2.11.0+cu128 + tensorrt 10.16.1.11，`torch.cuda.is_available()=True`（RTX PRO 5000 48 GB） |
| 检测权重 | `runs/detect/yolo26_caa_p3_dysample_detfly/weights/{best.pt,best.engine,best.onnx}`，以及 baseline 同套；`best.engine` 已实测可加载并在 GPU 上推理 |
| 数据集 | Det-Fly 位于 `/home/srcbit/Det-Fly-YOLO-1third`（约 11 GB）：真实天空背景侧视小目标；与 Gazebo 俯视渲染存在域差异（见 P3/P7） |
| 系统 Python | `/usr/bin/python3` 为 3.12，`rclpy`/`cv2` 可导入；已装 ROS 节点 shebang 为 `/usr/bin/python3` |

结论：P0 所需的系统环境已具备；剩余一次性补齐项为安装 `uv`。

## 3. 关键设计决策

### 3.1 推理进程：conda 常驻 worker 子进程（方案 B）

ROS 包保持系统 Python（3.12），**不 import torch**；模型推理在 conda `ultralytics` 环境的常驻子进程里完成，
两者用 stdin/stdout 二进制协议通信。理由：

- 直接复用已验证的环境与 FP16 TensorRT 引擎（2.36 ms/帧），不新增第二套 torch 环境；
- ROS 包依赖仍由 rosdep/apt 提供（`sensor_msgs`、`vision_msgs`、`geometry_msgs`）；
- ultralytics 仓库不改动（除 P7 可选微调脚本外），worker 脚本随 `gazebosimulation2d` 安装到 share。

协议（大端 u32 长度前缀，原始帧默认不压缩）：

```text
节点 → worker : [u32 头长度][JSON 头][帧字节]
  JSON: {"seq":12,"stamp_ns":123456789,"width":1280,"height":960,"encoding":"rgb8","format":"raw"}
worker → 节点 : [u32 头长度][JSON]
  JSON: {"seq":12,"stamp_ns":123456789,"inference_ms":2.4,
         "boxes":[[u,v,w,h,score], ...],"class_name":"uav","ok":true}
stderr        : 人员可读日志，节点转发到 ROS 日志
```

- 启动握手：worker 先输出 `{"ready":true,"model":"...","device":"0","imgsz":640}`，节点校验后开始处理图像；
- 节流与丢帧：`process_hz`（默认 10 Hz）、`max_frame_age_s`（跳过阻塞后过期的帧），忙时不排队；
- 超时与恢复：`select()` 读回包，超过 `inference_timeout_s` 或 EOF → 重启 worker（不超过 `worker_restart_limit`），
  期间不发布检测；
- 模型回退：`.engine` 加载失败时若 `model_fallback_path` 存在则自动回落到 `.pt` 并打日志；
- `yolo_python`、`yolo_worker_script`、`model_path` 全部走参数，不在仓库里硬编码机器路径。

### 3.2 时间基准与位姿缓存

- `config/camera_bridge.yaml` 增加 `/clock` 桥接（`gz.msgs.Clock → rosgraph_msgs/msg/Clock`）。
- 视觉链路三节点（`vision_detector`、`vision_adapter`、`guidance_node_2d`）在视觉模式下统一 `use_sim_time:=true`；
  `vision_source=yolo` 时若 `use_sim_time=false`，`vision_adapter` 直接报错退出。
- `vision_adapter` 维护追踪机位姿缓存 `(t_sim_ns, position_ned, quaternion_wxyz)`：
  收到检测后用图像 `stamp` 二分查找，位置线性插值、四元数最短弧插值后归一化，再经
  `camera_pose_from_odometry()` 合成相机位姿；超出 `pose_match_tolerance_s`、缓存落后于 `pose_cache_max_age_s`
  或落在最新样本之后（future）时拒绝量测并记录原因。
- PX4 消息 `timestamp` 字段改用宿主墙钟（`time.time_ns() // 1000`），把 `use_sim_time` 从 PX4 时间里解耦；
  默认路径（`use_sim_time=false`）下与现有行为等价。
- 启动防呆：`use_sim_time=true` 时若 2 s 墙钟内节点时钟仍无推进，打印明确错误并退出（避免“静默不动作”）。
- 已知残差：odometry 接收时刻 ≠ 采样时刻，量级为传输抖动（毫秒级）；不宣称采样同步，误差在 CSV 中量化。

### 3.3 任务几何与足印

- 相机 1280×960、$f_x=f_y\approx539.94$ px、水平 FOV 1.74 rad；相机高 8.1 m（机体 8 m + 安装 0.10 m），目标平面 1 m。
- 水平姿态下足印约 ±8.4 m（u）×±6.3 m（v）；中心像素尺度约 13.2 mm/px；0.35 m 目标约占 27 px。
- 机体倾斜平移足印：10° → 1.25 m；$a=6\ \mathrm{m/s^2}$（约 31°）→ 4.3 m。站立保持段加速度小（圆目标约 0.75 m/s²，约 4°），
  足印平移 < 0.6 m，因此“目标始终可见”由任务几何保证；但**漏检与大倾角瞬时越界仍需 coast/hold 兜底**。
- 起始条件：追踪机 spawn 在目标场景起点正上方（追踪机 8 m、目标 1 m）。各场景目标起点：
  `stationary=(40, 20)`、`linear=(25, -20)`、`circle=(47, 0)`（phase=0 时 `center+(12,0)`）。
- 已知几何细节：目标可见顶面比控制平面高约 0.15 m，用 $z=1.0$ 平面反投影会产生 1.4%～2.2% 的径向偏置
  （偏移 8 m 处约 0.15～0.2 m）；该偏置记录在指标中，不做隐蔽补偿。

### 3.4 检测契约

| 项 | 约定 |
| --- | --- |
| 输入 | `/camera/image_raw`（BEST_EFFORT/SENSOR_DATA），按 `encoding` 处理（rgb8/bgr8/rgba8/mono8） |
| 输出 | `/camera/detections`，`vision_msgs/Detection2DArray`，BEST_EFFORT + VOLATILE |
| Header | 数组与检测元素复制源图像 `stamp`/`frame_id` |
| 类别 | ultralytics 的 `uav` → `drone`；score ∈ [0,1]；单目标取最高分 |
| bbox | `bbox.center.position.x/y` 使用**原图坐标**（ultralytics 已做 letterbox 反变换，禁止二次缩放）；`size_x/size_y>0` |
| 未检出 | 发布空数组，下游据此区分“没有目标”与“检测节点挂了” |
| 统计 | `yolo_detections.csv`：每处理帧一行 `stamp_s,u,v,w,h,score,inference_ms,e2e_ms,detections` |

### 3.5 目标估计器与丢失状态机（纯 Python）

新增 `src/pythonsimulation2d/target_filter.py`（无 ROS，可离线单测），供 `guidance_node_2d` 使用：

$$\hat p_k^-=\hat p_{k-1}+\hat v_{k-1}\Delta t,\qquad
\hat p_k=\hat p_k^-+\alpha\,(z_k-\hat p_k^-),\qquad
\hat v_k=\hat v_{k-1}+\frac{\beta}{\Delta t}(z_k-\hat p_k^-)$$

- 滤波与预测均只作用于 XY（z 固定为目标平面高度）；$\Delta t$ 用**量测 stamp 差**（仿真时间）并夹在 `[min_dt, max_dt]`；
- 加速度由速度一阶低通差分给出（`vision_accel_tau_s`，供 `pn_mppi`/`pn_nmpc` 的预测使用）；
- 可选马氏门控（`vision_gate_sigma`，用 `/vision/target_pose` 的 XY 协方差块）拒绝离群量测；
- 状态机：`tracking → coast`（无新量测超过 `vision_coast_s=0.3 s`）`→ lost`（超过 `vision_loss_s=1.0 s`）`→ tracking`（重捕获）；
- `lost` 时 `guidance_node_2d` 进入 hold：`velocity=[0,0,0]`、`accel=0`、保持 yaw（现有 `_publish_pursuer_setpoint` 在
  `accel=0` 时会发当前速度，hold 必须单独发零速 setpoint）。

### 3.6 误差预算与精度预期

| 误差源 | 量级（7 m 高度） |
| --- | --- |
| 像素噪声（3 px，仅协方差假设） | 中心约 4 cm，足印边缘约 8 cm |
| 目标可见高度与平面假设差（约 0.15 m） | 径向 1.4%～2.2%，偏移 8 m 处约 0.15～0.2 m |
| 位姿/时间同步残差 | 毫秒级，cm 级 |
| YOLO bbox 中心误差 | 由 P3 实测（最大不确定项） |

结论：量测精度预期为中心区域 0.1～0.2 m、足印边缘 0.3～0.5 m。`pixel_error_vs_truth_px` 与
`position_error_vs_odom_m` 只在同一几何模型下自洽，**不是独立标定**，文档不得当作精度验收依据。

## 4. 分阶段实施

### P0 环境与桥接就绪（前置阻塞项）

1. 复核前置 ROS 包已安装：`ros-jazzy-vision-msgs`、`ros-jazzy-ros-gz-bridge`（2026-09-26 已用 `ros2 pkg prefix` 验证）；缺失环境用 `sudo apt install` 补齐。
2. `config/camera_bridge.yaml` 增加 `/clock` 桥接条目。
3. 相机与时钟验收：

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /clock
ros2 topic echo --once --qos-reliability best_effort --field encoding /camera/image_raw
ros2 run rqt_image_view rqt_image_view /camera/image_raw
gz stats
```

**验收**：三条话题稳定；记录 RTF、实际帧率、编码、分辨率。

### P1 检测节点与 worker

- 新增 `src/gazebosimulation2d/gazebosimulation2d/vision_detector.py`（系统 Python，唯一新 ROS 节点）。
- 新增 `src/gazebosimulation2d/scripts/yolo_worker.py`（安装到 share，用 conda python 执行；支持 `--self-test`）。
- `setup.py` 增加入口 `vision_detector` 与 worker 数据文件；`package.xml` 依赖不新增。
- 单测 `src/gazebosimulation2d/test/test_vision_detector.py`：用**假 worker 脚本**（回固定 bbox，不需要 torch）覆盖协议收发、
  空结果、超时重启、header 复制、`uav→drone` 映射、原图坐标不缩放、节流与过期丢帧。

**验收**：`ros2 topic hz /camera/detections` ≈ `process_hz`；`ros2 topic echo --once /camera/detections` 字段正确；
杀掉 worker 能自动重启；`--self-test` 在 Det-Fly 图上能出框且坐标与 `predict_one_image.py` 一致。

### P2 视觉量测接入（`vision_adapter` yolo 模式）

- `SUPPORTED_VISION_SOURCES = ("off", "truth", "yolo")`；yolo 模式要求 `use_sim_time=true`。
- yolo 模式新增：`/camera/detections` 订阅（BEST_EFFORT）、位姿缓存与插值、以检测回调驱动量测、`min_score` 门限；
  truth 定时器只在 truth 模式创建。
- 反投影复用现有 `pixel_to_ground()` 与 `_select_drone_detection()`；`/vision/target_pose` 消息格式不变。
- `vision_samples.csv` 扩展字段（truth 行保持原字段，新字段置 NaN）：
  `source, image_stamp_s, detection_age_ms, pose_match_dt_ms, pose_interpolated, score, bbox_w, bbox_h, n_detections,`
  `pixel_error_vs_truth_px, position_error_vs_odom_m`。
- 拒绝原因集合：`no_camera_info/invalid_pursuer_odometry/no_pursuer_odometry/pose_cache_miss/pose_cache_stale/`
  `pose_cache_future/no_drone_detection/low_score/backprojection_failed`。
- 数据集录制（供 P3 门槛与 P7 微调）：`record_dataset:=false` 默认关闭；开启后
  检测器按 `save_frame_hz` 落盘 `frames/<stamp_ns>.jpg`，适配器在位姿缓存有效且真值投影有效时落盘
  `labels/<stamp_ns>.txt`（YOLO 归一化格式，框由目标 8 角点投影 AABB 加 8% margin 得到）。无标签帧视为背景负样本。
- 单测扩展 `test_vision_adapter.py`：yolo 量测、缓存插值与未来/过期/空洞拒绝、`min_score`、空检测仍记录行、
  **truth 模式全量回归**；`test_unsupported_source_is_rejected` 的非法值改为 `"radar"`。

**验收**：离线注入合成 odometry + 检测，反投影几何自洽（同模型往返 < 1e-6 m）；truth 回归全绿。

### P3 零样本门槛评估（决定 P7）

- 新增 `tools/vision_offline_eval.py`（用 conda python 跑，不属于 uv 项目依赖）：读 `frames/` + `labels/`，
  统计 Recall@IoU0.3/0.5、中心像素误差 p50/p95，并扫描 conf ∈ [0.1, 0.5]。
- 采集：跑一次 truth 或 yolo 记录（circle + pn，追踪机在目标正上方），`record_dataset:=true`，
  覆盖各场景起点与圆周不同相位，正样本 ≥ 300 帧、背景帧 ≥ 100 帧。
- 门槛：Recall@IoU0.3 ≥ 0.8（conf=0.25）→ 直接进入 P4/P5；
  0.5～0.8 → 降低 conf + 门控后继续，并记录风险；
  < 0.5 → 进入 P7，P4/P5 仍用 `best.pt` 搭链路。

**产出**：`docs/` 记录表（帧数、Recall、像素误差、所选 conf）与结论。

### P4 目标估计器

- 新增 `src/pythonsimulation2d/target_filter.py`（`TargetFilterConfig` / `TargetEstimate` / `VisionTargetTracker`），
  行为见 §3.5。
- 单测 `tests/test_target_filter.py`（标准库 `unittest`，与 `tests/test_camera_geometry.py` 同风格）：
  匀速收敛、圆周跟踪稳态误差、门控拒绝、coast/lost/hold 边界、时间跳变与重复 stamp。

**验收**：`uv run python tests/test_target_filter.py` 全绿。

### P5 导引闭环接线

- `guidance_node.py` 新增参数（详见 §6）：`target_source`（默认 `odometry`）、`vision_topic`、`vision_fallback`
  （本轮默认 `none`，保留 `odometry` 供未来远距离场景回归）、`vision_*` 滤波与丢失参数。
- `target_source=vision` 时：订阅 `/vision/target_pose`；每个控制周期
  `update(量测 stamp) → predict(现在)`；`compute_guidance()` 使用估计状态；`lost` 时 hold。
- 记录 CSV 扩展列：`guidance_target_source, vision_valid, vision_age_s, vision_latency_s, vision_measurements,`
  `target_est_x/y/vx/vy/ax/ay, vision_error_xy`；`target_x/y` 仍为 odometry 真值（既有图/metrics 不受影响）。
- `px4_utils.timestamp_us()` 改为宿主墙钟（行为等价重构）。
- `plot_gazebo_csv.py`：当 CSV 含估计列时追加 `vision_estimate.png`（真值 vs 估计轨迹、误差、有效标志/年龄）；
  新增 `plot_vision_csv.py`（uv 跑）画检测率、像素残差、延迟、丢失时段。
- launch：新增参数与条件节点（`vision_source=yolo` 时启动检测节点与适配器；`use_sim_time` 统一传给视觉链路）。

**验收**：`target_source=odometry` 回归不受影响；`target_source=vision` 时导引输入切换、hold、记录列全部可观察。

### P6 Gazebo 闭环验收

外部终端（按 `README.md`）：QGC、追踪机 `4014 x500_mono_cam_down`、目标机 `4001 x500`、`MicroXRCEAgent udp4 -p 8888`。
追踪机 spawn 在目标场景起点正上方，示例（circle 起点 `(47, 0)`）；本机 SITL 已编译（新环境首次运行前需
`make px4_sitl gz_x500_mono_cam_down`）：

```bash
cd /home/srcbit/anti-drone/PX4-Autopilot
PX4_SYS_AUTOSTART=4014 PX4_GZ_MODEL_POSE="47,0,0,0,0,0" PX4_UXRCE_DDS_NS=px4_1 \
  ./build/px4_sitl_default/bin/px4 -i 0
# 另一终端：目标机 4001/gz_x500，-i 1，/px4_2，spawn 在 (47, 0)
```

launch：

```bash
cd 7_2Dsimulation
colcon build --packages-select gazebosimulation2d && source install/setup.bash
ros2 launch gazebosimulation2d guidance.launch.py \
  algorithm:=pn scenario:=circle \
  enable_camera:=true vision_source:=yolo target_source:=vision use_sim_time:=true \
  yolo_python:=/home/srcbit/miniconda3/envs/ultralytics/bin/python \
  yolo_model_path:=/home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_caa_p3_dysample_detfly/weights/best.engine \
  record_output_dir:=outputs/gazebo2d_vision_runs
```

**验收**（一次 `circle + pn` 闭环记录）：

| 指标 | 目标 |
| --- | --- |
| `/camera/detections` 帧率 | ≈ `process_hz`，worker 重启 0 次 |
| 闭环内检测率（相机视野内） | ≥ 0.9 |
| 水平偏移 p95（追踪机 vs 目标 XY） | ≤ 2.0 m |
| 跟踪段丢失（`lost`） | 首次进入跟踪后无 > 1 s 的连续丢失 |
| 记录完整性 | `vision_samples.csv`、`yolo_detections.csv`、`gazebo_samples.csv`、图与指标齐全 |
| 运行时长 | 完成 `sim_time`，无异常退出 |

已知调参项：目标在正下方时视线方向接近奇异（`r_norm→0`），`pn_guidance()` 在零偏移附近可能抖动；
先用 `pn_mppi`/`pn_nmpc` 的平滑项复测，必要时引入“最小保持间距”作为调参记录（不新增算法）。
其余场景与算法（`linear/stationary`、`basic/pn_mppi/pn_nmpc`）只做冒烟检查，不做对比实验与结论表。

### P7 域适配（仅 P3 不达标时）

1. 用 P2 的录制器采 2～5k 帧（不同相位/倾角/背景，含背景负样本），另混合约 30% Det-Fly 真实图防止遗忘；
2. 在 ultralytics 仓库新增微调脚本与 `data_gazebo_uav.yaml`，从 `best.pt` 微调 yolo26n（50～100 epoch，48 GB 显存）；
3. 评测：Gazebo 留出帧 Recall@IoU0.5 ≥ 0.9，且 Det-Fly val mAP 掉点 ≤ 1～2；
4. 用微调权重复跑 P6，更新文档与指标。

### P8 文档与提交

- `README.md` 新增“YOLO 视觉闭环仿真”章节：依赖、外部终端、launch 命令、参数表、输出文件、验收命令；
- 新增实测结果表（检测率、像素残差、量测误差、水平偏移、丢失事件）与 `docs/assets/` 图片；
- `docs/camera_vision_integration_plan.md` 顶部状态更新为“视觉闭环见新计划”；
- 提交信息中文、分主题：`新增视觉检测节点与 worker`、`新增视觉闭环导引`、`更新文档…`，不混格式化。

## 5. 文件改动清单

| 文件 | 改动 |
| --- | --- |
| `src/gazebosimulation2d/gazebosimulation2d/vision_detector.py` | 新增：图像订阅、worker 管理、`/camera/detections`、统计 CSV |
| `src/gazebosimulation2d/scripts/yolo_worker.py` | 新增：conda 环境常驻推理进程（协议、可选 `.engine` 回退 `.pt`、`--self-test`） |
| `src/gazebosimulation2d/gazebosimulation2d/vision_adapter.py` | yolo 模式、位姿缓存、CSV 扩展、数据集录制 |
| `src/gazebosimulation2d/gazebosimulation2d/guidance_node.py` | `target_source`、视觉订阅、估计器接线、hold、记录列 |
| `src/gazebosimulation2d/gazebosimulation2d/px4_utils.py` | PX4 时间戳改宿主墙钟 |
| `src/pythonsimulation2d/target_filter.py` | 新增：α-β + coast/hold 状态机（纯算法） |
| `src/gazebosimulation2d/config/default.yaml` | `vision_detector` 段、adapter yolo 参数、guidance 视觉参数 |
| `src/gazebosimulation2d/config/camera_bridge.yaml` | 新增 `/clock` 桥接 |
| `src/gazebosimulation2d/launch/guidance.launch.py` | 新参数与条件节点、`use_sim_time` 传递 |
| `src/gazebosimulation2d/setup.py` | 新入口与 worker 数据文件 |
| `src/gazebosimulation2d/package.xml` | 不新增（现有依赖已覆盖） |
| `tests/test_target_filter.py`、`tests/test_camera_geometry.py` | 新增估计器测试；几何测试不改 |
| `src/gazebosimulation2d/test/test_vision_detector.py`、`test_vision_adapter.py` | 新增/扩展 |
| `plot_gazebo_csv.py`、`plot_vision_csv.py` | 视觉面板与视觉 CSV 绘图 |
| `tools/vision_offline_eval.py` | 零样本/微调离线评估（conda python 运行） |
| `README.md`、`docs/*` | 文档与结果表 |
| （P7 条件）`ultralytics-main/train_gazebo_ft.py` + `data_gazebo_uav.yaml` | 微调脚本与数据配置（跨仓库，独立提交） |

## 6. 参数清单

`vision_detector`（节点名同前缀，launch 参数加 `yolo_` 前缀避免与导引节点重名）：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `yolo_python` | `""` | conda 环境 python，必须由使用者提供 |
| `yolo_worker_script` | share 内 `scripts/yolo_worker.py` | 可覆盖为源码路径 |
| `model_path` | `""` | `.engine` 优先，回退 `.pt` |
| `model_fallback_path` | `""` | 引擎加载失败时的 `.pt` |
| `imgsz` / `conf` / `iou` | 640 / 0.25 / 0.7 | ultralytics 推理参数 |
| `device` / `half` | `0` / true | GPU 与 FP16 |
| `max_det` / `class_name` | 5 / `uav` | 单目标，取最高分 |
| `process_hz` / `max_frame_age_s` | 10.0 / 0.2 | 节流与过期丢帧 |
| `frame_format` | `raw` | `raw|jpeg`（网络/负载异常时可选 jpeg） |
| `inference_timeout_s` / `worker_restart_limit` | 1.0 / 3 | 超时与重启上限 |
| `save_frame_hz` / `dataset_output_dir` | 0.0 / `outputs/gazebo2d_vision/dataset` | 数据集录制（0 关闭） |
| `stats_csv` | `outputs/gazebo2d_vision/yolo_detections.csv` | 检测统计 |
| `debug_log` / `debug_log_period_s` | false / 1.0 | 周期日志 |

`vision_adapter` 新增：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `vision_source` | `off` | `off|truth|yolo` |
| `min_score` | 0.25 | 检测最低分 |
| `pose_match_tolerance_s` | 0.05 | 图像 stamp 与位姿缓存的时间容差 |
| `pose_cache_max_age_s` | 0.5 | 位姿缓存最大保留时长 |
| `pose_cache_interpolate` | true | 是否插值（false 时用最近样本） |
| `extrapolate_pose` | false | 是否允许在最新样本之后外推（默认拒绝） |
| `record_dataset` | false | 数据集录制开关 |
| `dataset_output_dir` | `outputs/gazebo2d_vision/dataset` | 图像/标签目录 |

`guidance_node_2d` 新增：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `target_source` | `odometry` | 导引输入：`odometry|vision` |
| `vision_topic` | `/vision/target_pose` | 视觉量测话题 |
| `vision_fallback` | `none` | 捕获前回退：`none|odometry`（本轮默认 none） |
| `vision_max_age_s` | 0.5 | 超过视为无效量测 |
| `vision_alpha` / `vision_beta` | 0.85 / 0.25 | α-β 系数 |
| `vision_accel_tau_s` | 0.5 | 加速度低通时间常数（0 关闭前馈） |
| `vision_gate_sigma` | 0.0 | 马氏门控（0 关闭） |
| `vision_coast_s` / `vision_loss_s` | 0.3 / 1.0 | 丢失状态机阈值 |
| `vision_hold_on_loss` | true | 丢失后是否悬停 |
| `min_dt_s` / `max_dt_s` | 0.01 / 0.5 | 滤波 dt 夹取范围 |

launch 通用参数：`use_sim_time`（默认 false，视觉闭环置 true）。

## 7. 输出与绘图

| 文件 | 内容 |
| --- | --- |
| `outputs/gazebo2d_vision/vision_samples.csv` | 逐检测量测/拒绝记录（truth/yolo 共用，含诊断字段） |
| `outputs/gazebo2d_vision/yolo_detections.csv` | 逐处理帧检测与延迟 |
| `outputs/gazebo2d_vision_runs/<scenario>/<algorithm>/gazebo_samples.csv` | 导引记录（视觉运行建议用独立 `record_output_dir`，避免覆盖 odometry 基线） |
| `outputs/gazebo2d_vision/dataset/{frames,labels}/` | P3 门槛评估与 P7 微调数据（gitignore） |

```bash
uv run plot_gazebo_csv.py outputs/gazebo2d_vision_runs/circle --output-dir outputs/circle_vision
uv run plot_vision_csv.py outputs/gazebo2d_vision --output-dir outputs/vision_report
```

## 8. 风险与对策

| 风险 | 对策 |
| --- | --- |
| **零样本检不到 Gazebo 目标（最大风险，俯视渲染差异大）** | P3 门槛 + P7 自动标注微调；worker 支持 `.pt` 回退 |
| 时间基准不一致导致速度估计成倍偏差 | `/clock` + `use_sim_time` + 按 stamp 插值；P2 单测覆盖 |
| 正下方视线奇异导致控制抖动 | 先用 MPPI/EMPC 平滑复测；记录调参过程，不新增算法 |
| 漏检/大倾角越界造成短时无检测 | coast/hold + 重捕获；指标量化可见率与丢失时长 |
| worker 崩溃或引擎不兼容 | 超时重启 + `.pt` 回退 + 周期日志；`--self-test` 先离线验证 |
| 运行机器与本机环境不一致 | `yolo_python` 显式传参；文档写明环境与引擎/GPU 绑定 |
| RTF 下降 | `gz stats` 记录；`process_hz` 节流；不改 PX4 安装目录与仓库模型 |

## 9. 明确不做 / 后续工作

- 不做 odometry/truth/多算法对比实验与结论表；不做远距离捕获、视觉移交、FOV 约束算法；
- 不新增导引算法，不改 `compute_guidance()` 的算法集合与注册表；
- 不改 ultralytics 仓库（P7 微调脚本除外，独立提交）；不修改 PX4 安装目录与模型 SDF；
- 未实现 TF、多目标跟踪、标注图发布、独立图像标定、视觉伺服（图像空间）导引；
  以上如需开展，另立计划并更新根 `README.md`。
