# YOLO 视觉闭环实施与验证记录

> 本文件记录实现状态、离线验证结果与 2026-09-30 的 Gazebo 闭环实测数据；阶段编号仅用于对应历史记录。
> **未实测的项目不填写数字。**
> 运行步骤与参数见 [模块 README](../README.md)，接口约定见 [视觉设计参考](vision_design.md)。

## 1. 实现状态

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| P1 | `vision_detector` 检测节点 + `scripts/yolo_worker.py` 常驻推理 + 协议/重启/录制 | 已实现，离线测试与真实引擎自检通过 |
| P2 | `vision_adapter` yolo 模式：检测订阅、按图像 stamp 的位姿缓存插值、拒绝原因、CSV 扩展、数据集标注 | 已实现，离线测试通过 |
| P3 | `tools/vision_offline_eval.py` 零样本评估（Recall@IoU、像素误差、conf 扫描） | 工具已实现并冒烟验证；2026-09-30 闭环未采数据集，未做门槛评估 |
| P4 | `src/pythonsimulation2d/target_filter.py` α-β + coast/lost 状态机 | 已实现，17 项离线测试通过 |
| P5 | `guidance_node_2d` 视觉接线、hold、记录列、`px4_utils` 宿主墙钟、绘图与 launch | 已实现，18 项 ROS 测试通过 |
| P6 | Gazebo 闭环验收 | 已实测（2026-09-30）：跟踪段丢失达标，检测率与水平偏移未达门槛，见第 4 节 |
| P7 | 域适配微调（仅 P3 不达标时） | 未触发 |
| P8 | 文档与结果表 | 本文件 + README 已更新，P6 结果已记录 |

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

拒绝原因集合（yolo 模式）：`no_camera_info / no_pursuer_odometry / pose_cache_miss / pose_cache_stale /
pose_cache_future / no_drone_detection / low_score / backprojection_failed`；truth 模式另有
`no_target_odometry / unsupported_pursuer_frame / unsupported_target_frame / invalid_pursuer_odometry /
invalid_target_odometry / projection_failed`。

初始捕获：`target_source=vision` 时 `guidance_node_2d` 把追踪机起飞保持点设为场景起点 XY（高度
`pursuer_fixed_altitude`），目标机在准备阶段停在该起点，保证开始跟踪时目标已在相机视野内，不依赖两机
spawn 位置。下视相机在 8 m 高度、目标平面 1 m 时的足印约 17 x 12 m；circle 目标按默认 spawn 悬停时
最近距离 23 m，永远无法进入视野，必须先完成初始捕获。

环境：视觉实验使用仓库内置无阴影世界 `worlds/default.sdf`（相对 PX4 v1.16 的 `default.sdf` 仅关闭
`<scene><shadows>` 与太阳 `cast_shadows`，世界名保持 `default`）。下视相机 8 m 高度、目标平面 1 m 时
太阳仰角约 51°，两架无人机的影子会偏移约 5.7 m 落在画面内，YOLO 容易把影子误检成目标；Gazebo 先于
PX4 手动启动，启动步骤见 [模块 README](../README.md#下视相机与视觉闭环)。

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
  --model /home/srcbit/anti-drone/ultralytics-main/runs/detect/yolo26_baseline_detfly/weights/best.engine \
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

## 4. Gazebo 闭环实测（2026-09-30）

环境：PX4 v1.16 SITL + Gazebo 无阴影世界、`circle` 目标，`vision_source=yolo` + `target_source=vision`、
`use_sim_time=true`，模型 `yolo26_baseline_detfly/weights/best.engine`；四种算法依次各跑 40 s。
外部终端启动、参数与绘图命令见 [模块 README](../README.md#下视相机与视觉闭环)。

原始记录（均在仓库 `outputs/`）：

```text
gazebo2d_vision_runs/circle/{basic,pn,pn_mppi,pn_nmpc}/gazebo_samples.csv
gazebo2d_vision/vision_samples.csv        # 逐量测记录，来自最后一次 pn_nmpc 运行
gazebo2d_vision/yolo_detections.csv       # 逐处理帧检测，同上
circle_vision/                            # 绘图与 metrics.csv
```

时间口径：以 `gazebo_samples.csv` 的 0–40 s 为跟踪窗口，对应视觉记录的 elapsed 11.46–51.46 s
（与引导节点记录的量测计数 256 对齐）；窗口外的起飞准备与收尾数据不计入。

### 4.1 视觉量测统计（跟踪窗口）

| 指标 | 实测 |
| --- | ---: |
| 处理帧 / 有效量测 | 309 / 256 |
| 帧级检出率 | 0.829 |
| 拒绝原因 | 全部为 `no_drone_detection`（53 帧） |
| 最长连续丢失 | 0.90 s |
| 检测时延 p50 / p95 | 36 / 56 ms |
| 检测分数 p50 / p95 | 0.774 / 0.839 |
| 像素残差 p50 / p95（同源诊断） | 154 / 196 px |
| 位置残差 p50 / p95（同源诊断） | 1.99 / 2.16 m |
| 检测帧率（仿真时间） | ≈7.7 Hz |
| 推理 / 端到端耗时 p50 | 2.8 ms / 11.5 ms |

像素与位置残差由 `vision_adapter` 用目标 odometry 参考计算，与量测同源；本次记录中残差呈近似常值
`(-2.0, +0.1) m`（标准差 < 0.15 m），且不随时间与 yaw 变化，与像素噪声的量级不符。这来自两机本地系的
常值偏移：本次运行两机 spawn 为追踪机 `49,0`、目标机 `47,0`（x 相差 2 m），残差 x 分量与 spawn 差
（`47 - 49 = -2 m`）一致，符合 [视觉设计参考](vision_design.md) 1.1 节提示的本地原点问题；视觉量测在
追踪机本地系、目标 odometry 在目标机本地系，直接相减就会得到这个常值差。**若要用 P6 门槛判定，应先让
两机同点 spawn 或做原点转换后重跑；在此之前，所有“vs odom”口径的误差与距离都不能当作量测精度。**

### 4.2 P6 验收

| 指标 | 目标 | 实测 | 结论 |
| --- | --- | --- | --- |
| `/camera/detections` 帧率 | ≈ `process_hz`（10 Hz），worker 重启 0 次 | ≈7.7 Hz（仿真时间）；重启次数未记录进 CSV | 未达 |
| 闭环内检测率（相机视野内） | ≥ 0.9 | 0.829（256/309） | 未达 |
| 水平偏移 p95（追踪机 vs 目标 XY） | ≤ 2.0 m | 估计口径：`basic/pn` 3.00/3.46 m、`pn_mppi/pn_nmpc` 9.00/4.78 m；目标 odometry 口径整体再大约 2 m（见 4.1） | 未达 |
| 跟踪段丢失（`lost`） | 首次进入跟踪后无 > 1 s 连续丢失 | 最长 0.90 s | 达标 |
| 记录完整性 | 4 类 CSV + 图 + 指标齐全 | `vision_samples.csv`、`yolo_detections.csv`、4×`gazebo_samples.csv`、图与 `metrics.csv` 齐全；未采数据集/标注 | 齐全 |
| 运行时长 | 完成 `sim_time`，无异常退出 | 0–40 s 完整记录 | 达标 |

结论：本次 P6 **未通过**。

- 帧级检出率 0.829 低于 0.9，53 帧拒绝全部为 `no_drone_detection`；其中可能混有目标短暂离开
  相机足印的时段，建议用 `camera_recorder` 画面复核后再区分“模型漏检”与“目标不在视野”。
- 水平偏移按控制器实际使用的估计口径也在 3.0～9.0 m，四算法均未达到 2.0 m；`pn_mppi`/`pn_nmpc`
  在该次运行中没有回到 1.5 m 捕获半径。
- 若要按 P6 门槛给出判定，需先对齐两机本地原点（同点 spawn 或原点转换）后重跑；本次记录可作为视觉链路连通性验证。

### 4.3 EMPC 参数调整与 YOLO 闭环复核（2026-10-01）

2026-10-01 复核时发现：默认参数下 EMPC（`pn_nmpc`）存在"外侧大半径轨道"风险。两机从各自 spawn 正常起飞、就位后开始跟踪（追踪机就位时还带一点残余速度）的运行中，EMPC 稳定停在目标外侧 4～6 m 的轨道上，40 s 内水平误差峰值 **6.12 m**（28.5% 时间超过 5 m）；而追踪机先在起点悬停、再由同一套参数开始跟踪时，误差峰值只有 2.96 m。同一被控对象下 `pn` 与 `pn_mppi` 没有出现这种轨道，说明问题出在 EMPC 的内部预测模型：PX4 速度环对加速度指令有约 0.15～0.2 s 的响应滞后，2 s 预测窗口会让 EMPC 高估自身机动能力、预判激进候选"会飞过目标"，于是持续选择偏保守的小修正，闭环里无法把轨道收回来。

参数调整（只动 EMPC/MPPI 的预测窗口与权重，`pn` 与场景定义不变）：

| 参数 | 调整前 | 调整后 | 作用 |
| --- | ---: | ---: | --- |
| `horizon_steps` | 20（2.0 s） | 8（0.8 s） | 预测窗口缩短到与执行器动态同量级，减少模型高估 |
| `nmpc_w_path` | 0.1 | 0.5 | 提高预测窗口内累计距离的权重，鼓励持续接近 |
| `nmpc_w_pn` | 0.04 | 1.0 | 内部模型不可靠时跟随 `pn_trend` 候选 |

复核在无阴影世界 + YOLO 视觉闭环（`vision_source=yolo`、`target_source=vision`、`use_sim_time=true`）下进行，口径与第 4 节一致（`gazebo_samples.csv` 的 `distance_xy`，0–40 s 跟踪窗口）：

| 运行 | 起始方式 | 参数 | 最大水平误差/m | 超 5 m 时间占比 |
| --- | --- | --- | ---: | ---: |
| 1 | 两机正常起飞就位 | 默认 | 6.12 | 28.5% |
| 2 | 追踪机起点悬停后就位 | 默认 | 2.96 | 0% |
| 3 | 两机正常起飞就位 | 调整后 | 2.64 | 0% |
| 4 | 追踪机起点悬停后就位 | 调整后 | 2.44 | 0% |
| 5 | 两机正常起飞就位 | 调整后 | 2.81 | 0% |

结论：调整后的 EMPC 在两种起始方式和重复运行下最大水平误差 2.4～2.8 m，全程满足 ≤ 5 m；默认参数只在"悬停就位"这一种起始方式下达标。代价是控制更激进、更贴近 PN 趋势（`pn_nmpc` 的 yaw rate mean 不再是四算法中最低），离线捕获时间和控制能量反而下降，见 [算法说明](2d_simulation_guidance_overview.md) 第 10 节。原始记录在 `outputs/tuning_runs/`（生成物，不入库）。

## 5. 已知边界与风险

- **零样本域差异是最大风险**：Det-Fly 是真实天空背景侧视图，Gazebo 是俯视渲染；P3 门槛不达标时再评估是否需要域适配微调。
- `yolo_model_path` 必须是完整文件路径；worker 启动失败（如路径不存在）会先以 `[worker]` 前缀转发 worker stderr，再按 `worker_restart_limit` 重试并停止检测，不要复制文档中的路径占位符。
- 量测精度设计预期为厘米级（像素噪声 + 0.15 m 目标高度平面假设 + 毫秒级同步残差），但 2026-09-30 实测的
  同源残差呈约 2 m 的常值偏差（见 4.1），指向两机本地原点/坐标对齐问题而非像素噪声；对齐前不要引用任何
  误差数字作为精度指标。`pixel_error_vs_truth_px` / `position_error_vs_odom_m` 与量测同源，**不是独立标定**。
- **两机本地原点不一致会让跨机指标带常值偏差**：`vision_adapter` 的“vs odom”诊断列和
  `gazebo_samples.csv` 的距离列只对原点一致的两机成立；两机在不同点 spawn（本记录 `49,0` / `47,0`）
  会产生约 2 m 的常值 xy 偏差，需要两机同点 spawn 或在 ROS 边界显式转换，见 [视觉设计参考](vision_design.md) 1.1 节。
- 正下方视线接近奇异（`r_norm→0`），`pn_guidance()` 在零偏移附近可能抖动；先用 `pn_mppi`/`pn_nmpc` 的平滑项复测，
  必要时记录“最小保持间距”调参，不新增算法。
- PX4 SITL 的 `hrt` 由 Gazebo 时钟驱动（`GZBridge::clockCallback`），但本链路不依赖该事实：位姿缓存使用接收时的
  仿真时间，`pose_match_dt_ms` 暴露残差。
