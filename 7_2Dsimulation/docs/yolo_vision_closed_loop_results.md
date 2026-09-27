# YOLO 视觉闭环实施与验证记录

> 本文件保留历史实现状态、离线验证结果与待实测表格；阶段编号仅用于对应历史记录。
> 这些结果并非本次清理重新验证所得。**未实测的项目不填写数字。**
> 运行步骤与参数见 [模块 README](../README.md)，接口约定见 [视觉设计参考](vision_design.md)。

## 1. 实现状态

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| P1 | `vision_detector` 检测节点 + `scripts/yolo_worker.py` 常驻推理 + 协议/重启/录制 | 已实现，离线测试与真实引擎自检通过 |
| P2 | `vision_adapter` yolo 模式：检测订阅、按图像 stamp 的位姿缓存插值、拒绝原因、CSV 扩展、数据集标注 | 已实现，离线测试通过 |
| P3 | `tools/vision_offline_eval.py` 零样本评估（Recall@IoU、像素误差、conf 扫描） | 工具已实现并冒烟验证；**Gazebo 数据集采集与门槛结论待闭环实测** |
| P4 | `pythonsimulation2d/target_filter.py` α-β + coast/lost 状态机 | 已实现，17 项离线测试通过 |
| P5 | `guidance_node_2d` 视觉接线、hold、记录列、`px4_utils` 宿主墙钟、绘图与 launch | 已实现，18 项 ROS 测试通过 |
| P6 | Gazebo 闭环验收 | **待实测** |
| P7 | 域适配微调（仅 P3 不达标时） | 未触发 |
| P8 | 文档与结果表 | 本文件 + README 已更新；结果表待 P6 数据 |

## 2. 数据流与关键决策

```text
/camera/image_raw ──► vision_detector ──stdio 协议──► conda yolo_worker（.engine/.pt）
        │                     │
        │                     └──► /camera/detections（Detection2DArray，原图坐标，单目标最高分）
        ▼
vision_adapter（use_sim_time=true）
  追踪机位姿缓存（接收时的仿真时间）── 按图像 stamp 插值 ──► camera_pose_from_odometry
        │
        └──► pixel_to_ground(z=target_base_altitude) ──► /vision/target_pose（ENU + XY 协方差）
                                                              │
guidance_node_2d（target_source=vision）── α-β update(stamp) → predict(now) ──► compute_guidance
        │                                        │
        │                                        └── lost → hold：零速零加速度、保持 yaw
        └──► gazebo_samples.csv（target_x/y 仍是 odometry 真值；估计值单独成列）
```

时间基准：视觉链路统一 `use_sim_time=true`；位姿缓存键是**收到 odometry 时的仿真时间**（不假设 PX4
`timestamp_sample` 与 ROS 时钟同源），量测按图像 `header.stamp` 插值；三节点均有墙钟防呆，2 s 内收不到
`/clock` 会打印 FATAL 并退出。残余同步误差（接收时刻 ≠ 采样时刻）在 `pose_match_dt_ms` 中量化。

拒绝原因集合：`no_camera_info / invalid_pursuer_odometry / no_pursuer_odometry / pose_cache_miss /
pose_cache_stale / pose_cache_future / no_drone_detection / low_score / backprojection_failed`。

## 3. 已完成验证

### 3.1 ROS 单元/集成测试（`colcon test`，假 worker，不需要 torch/GPU）

```bash
cd 7_2Dsimulation
colcon build --packages-select gazebosimulation2d
source install/setup.bash
colcon test --packages-select gazebosimulation2d && colcon test-result --verbose
```

结果：**104 tests, 0 errors, 0 failures**（检测节点 20、适配节点 45、导引视觉接线 24、相机记录与图像转换 15；2026-09-27 增加初始捕获、worker stderr 转发与 camera_recorder 用例后复测）。

覆盖要点：协议收发/握手、原图坐标不缩放、空检测、header 复制、节流与过期丢帧、超时/崩溃重启与上限、
jpeg 帧格式、统计 CSV 与数据集帧、握手失败时转发 worker stderr；yolo 量测、缓存插值/未来/过期/空洞拒绝、
`min_score`、空检测仍记录、truth 模式全量回归；视觉参数校验、α-β 接线、hold 零速、odometry 回归、
视觉模式初始捕获（起飞保持点取场景起点）、CSV 视觉列、PX4 时间戳；camera_recorder 参数校验、按图像
stamp 的 1 Hz 节流、重启跳过已存在帧、max_frames 上限与共享图像编码转换。

### 3.2 纯 Python 估计器测试

```bash
uv run python tests/test_target_filter.py
```

结果：**17 tests OK**。匀速收敛误差 < 2 cm、圆周（半径 12 m、0.25 rad/s）稳态误差 < 5 cm、
门控拒绝离群并恢复、coast/lost 边界、重复/回跳 stamp、长空窗外推夹取。

### 3.3 真实 worker 自检（conda + RTX PRO 5000）

```bash
/home/srcbit/miniconda3/envs/ultralytics/bin/python src/gazebosimulation2d/scripts/yolo_worker.py \
  --model /home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_caa_p3_dysample_detfly/weights/best.engine \
  --self-test /home/srcbit/Det-Fly-YOLO-1third/images/val/0207134.jpg
```

结果：`best.engine` 加载成功，FP16 推理约 2.34 ms/帧；框 `[1698.0, 938.25, 87.0, 46.5]`、score
`0.8408`，与 `predict_one_image.py` 使用同一引擎的输出**逐位一致**。

### 3.4 离线评估工具冒烟（非 Gazebo 门槛）

用 6 张 Det-Fly val 图（真实天空背景侧视图）构造 `frames/ + labels/` 冒烟数据集，`vision_offline_eval.py`
在 conf ∈ {0.10, 0.25, 0.30, 0.50} 下 Recall@IoU0.3/0.5 均为 1.000，匹配框中心像素误差
p50=2.21 px / p95=5.23 px。**该结果只验证工具链路，不代表 Gazebo 俯视渲染的零样本能力**，不能作为 P3 门槛结论。

### 3.5 离线 ROS 全链路联调（真实 worker，无 Gazebo）

用脚本发布 `/clock`（仿真时间从 100 s 推进）、`/camera/image_raw`（Det-Fly 4K 图，`rgb8`）、
`/camera/camera_info`（3840×2160，`fx=fy=1619.81`）与两机 `vehicle_odometry`（50 Hz），目标机位置按
检测框中心解析反投影点设置，实测结果：

| 项 | 结果 |
| --- | --- |
| 检测节点 | 真实 `best.engine` 通过 ROS 图像链路输出 `[1698.0, 938.25, 87.0, 46.5]`、score `0.8408`，与 worker 自检一致 |
| 检测率 / 帧率 | 31/31 帧检出（该图片为 Det-Fly 侧视图，非 Gazebo 渲染） |
| 推理 / 端到端耗时 | p50 = 7.97 ms / 72.91 ms（4K 原图 24.9 MB 走 raw 管道；Gazebo 1280×960 会明显更小） |
| 适配节点 | `valid=1`、`pose_interpolated=1`、`detection_age_ms=139.9`、`position_roundtrip_error_m=2.5e-8 m` |
| 反投影一致性 | `target_est=(-0.9730778, 0.6213233)` 与解析真值一致；`pixel_error_vs_truth_px=5.6e-6`、`position_error_vs_odom_m=2.5e-8`（同源自洽，非独立标定） |

该联调覆盖了“真实引擎 → ROS 检测 → 按 stamp 插值位姿 → 反投影量测 → CSV”的完整软件链路；
Gazebo 渲染域的零样本能力仍需 P3 数据采集判定。联调还发现并修复了 `detection_age_ms` 的 ns→ms 单位错误。

### 3.6 launch 接线与 `/clock` 防呆

`ros2 launch ... vision_source:=yolo target_source:=vision use_sim_time:=true` 启动三个节点；在无
`/clock` 的环境下，三个节点分别在 2 s 后打印明确 FATAL 并干净退出（验证墙钟防呆可用，而不是静默悬停）。

## 4. 待闭环实测（P3 门槛与 P6 验收）

环境启动、数据采集、离线评估和绘图命令统一维护在 [模块 README](../README.md#下视相机与视觉闭环)。Gazebo、PX4 SITL、XRCE Agent 与 QGC 由使用者手动启动。

### 4.1 P3 门槛表（待填）

| 指标 | 目标 | 实测 |
| --- | --- | --- |
| 正样本帧数 / 背景帧数 | ≥ 300 / ≥ 100 | 待填 |
| Recall@IoU0.3（conf=0.25） | ≥ 0.8 | 待填 |
| Recall@IoU0.5（conf=0.25） | 记录 | 待填 |
| 匹配框中心像素误差 p50 / p95 | 记录 | 待填 |
| 背景帧误检率 | 记录 | 待填 |
| 结论（直接闭环 / 降 conf / P7 微调） | — | 待填 |

### 4.2 P6 验收表（待填）

| 指标 | 目标 | 实测 |
| --- | --- | --- |
| `/camera/detections` 帧率 | ≈ `process_hz`，worker 重启 0 次 | 待填 |
| 闭环内检测率（相机视野内） | ≥ 0.9 | 待填 |
| 水平偏移 p95（追踪机 vs 目标 XY） | ≤ 2.0 m | 待填 |
| 跟踪段丢失（`lost`） | 首次进入跟踪后无 > 1 s 连续丢失 | 待填 |
| 记录完整性 | 4 类 CSV + 图 + 指标齐全 | 待填 |
| 运行时长 | 完成 `sim_time`，无异常退出 | 待填 |

## 5. 已知边界与风险

- **零样本域差异是最大风险**：Det-Fly 是真实天空背景侧视图，Gazebo 是俯视渲染；P3 门槛不达标时再评估是否需要域适配微调。
- `yolo_model_path` 必须是完整文件路径；worker 启动失败（如路径不存在）会先以 `[worker]` 前缀转发 worker stderr，再按 `worker_restart_limit` 重试并停止检测，不要复制文档中的路径占位符。
- 量测精度预期：中心区域 0.1～0.2 m、足印边缘 0.3～0.5 m（像素噪声 + 0.15 m 目标高度平面假设 + 毫秒级同步残差）；
  `pixel_error_vs_truth_px` / `position_error_vs_odom_m` 与量测同源，**不是独立标定**。
- 正下方视线接近奇异（`r_norm→0`），`pn_guidance()` 在零偏移附近可能抖动；先用 `pn_mppi`/`pn_nmpc` 的平滑项复测，
  必要时记录“最小保持间距”调参，不新增算法。
- PX4 SITL 的 `hrt` 由 Gazebo 时钟驱动（`GZBridge::clockCallback`），但本链路不依赖该事实：位姿缓存使用接收时的
  仿真时间，`pose_match_dt_ms` 暴露残差。
