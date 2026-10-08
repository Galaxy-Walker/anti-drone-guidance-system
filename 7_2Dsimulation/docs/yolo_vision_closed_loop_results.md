# YOLO 视觉闭环实施与验证记录

> 本文件记录实现状态、离线验证结果与 2026-10-04 的 Gazebo 四算法闭环复测数据；2026-09-30 首次闭环结果摘要见第 4 节开头，2026-10-01/10-04 的调参、FOV 验证与高度×速度矩阵见 4.3～4.5 节；2026-10-05 的桌下遮挡参数/速度扫描与同晚的四算法对照见第 5 节，配套基础设施问题定位见第 6 节。
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
| P6 | Gazebo 闭环验收 | 已实测（2026-09-30 首次、2026-10-04 复测）：检出率与水平偏移均未达门槛，复测中跟踪段丢失也未达标，见第 4 节 |
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

结果：**107 tests, 0 errors, 0 failures**（检测节点 20、适配节点 45、导引视觉接线 27、相机记录与图像转换 15；2026-09-27 增加初始捕获、worker stderr 转发与 camera_recorder 用例，2026-10-04 增加 `target_speed_scale` 用例后复测）。

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

## 4. Gazebo 闭环实测（2026-10-04 四算法复测）

> 本节为 2026-10-04 的复测记录，取代 2026-09-30 的首次闭环记录（首次记录为 309 帧 / 256 有效量测、
> 检出率 0.829，两机 spawn 差 2 m 使同源残差整体偏约 2 m）。两批均未通过 P6 验收，见 4.2。

环境：PX4 v1.16 SITL + Gazebo 无阴影世界、`circle` 目标，`vision_source=yolo` + `target_source=vision`、
`use_sim_time=true`，模型 `yolo26_baseline_detfly/weights/best.engine`；四种算法依次各跑 40 s，
两机 spawn 相差 1 m（x 方向原点差约 −1 m，见 4.1）。
外部终端启动、参数与绘图命令见 [模块 README](../README.md#下视相机与视觉闭环)。

原始记录（均在仓库 `outputs/`）：

```text
gazebo2d_vision_runs/circle/{basic,pn,pn_mppi,pn_nmpc}/gazebo_samples.csv
gazebo2d_vision/vision_samples.csv        # 逐量测记录，来自最后一次 pn_nmpc 运行
gazebo2d_vision/yolo_detections.csv       # 逐处理帧检测，同上
circle_vision/                            # 绘图与 metrics.csv
```

时间口径：以 `gazebo_samples.csv` 的 0–40 s 为跟踪窗口，对应视觉记录的 elapsed 14.38–54.38 s
（与引导节点记录的量测计数 251 对齐）；窗口外的起飞准备与收尾数据不计入。整份记录（含窗口外）由
`plot_vision_csv.py` 汇总为 437 处理帧、检出率 0.684，与下表差异来自窗口外目标不在相机视野内的时段。

### 4.1 视觉量测统计（跟踪窗口）

| 指标 | 实测 |
| --- | ---: |
| 处理帧 / 有效量测 | 303 / 251 |
| 帧级检出率 | 0.828 |
| 拒绝原因 | 全部为 `no_drone_detection`（52 帧） |
| 最长连续丢失 | 1.06 s |
| 检测时延 p50 / p95 | 16 / 20 ms |
| 检测分数 p50 / p95 | 0.749 / 0.834 |
| 像素残差 p50 / p95（同源诊断） | 83.6 / 106.4 px |
| 位置残差 p50 / p95（同源诊断） | 1.06 / 1.23 m |
| 检测帧率（仿真时间） | ≈7.6 Hz |
| 推理 / 端到端耗时 p50 | 2.03 ms / 7.67 ms |

像素与位置残差由 `vision_adapter` 用目标 odometry 参考计算，与量测同源；本次记录中残差近似常值
`(-1.04, -0.01) m`（标准差 0.12～0.14 m），与像素噪声的量级不符。这来自两机本地系的常值偏移：
本次运行两机 spawn 相差 1 m，残差 x 分量与 spawn 差一致，符合 [视觉设计参考](vision_design.md) 1.1 节
提示的本地原点问题；视觉量测在追踪机本地系、目标 odometry 在目标机本地系，直接相减就会得到这个常值差。
**若要用 P6 门槛判定，应先让两机同点 spawn 或做原点转换后重跑；在此之前，所有“vs odom”口径的误差与
距离都不能当作量测精度。**

### 4.2 P6 验收

| 指标 | 目标 | 实测 | 结论 |
| --- | --- | --- | --- |
| `/camera/detections` 帧率 | ≈ `process_hz`（10 Hz），worker 重启 0 次 | ≈7.6 Hz（仿真时间）；重启次数未记录进 CSV | 未达 |
| 闭环内检测率（相机视野内） | ≥ 0.9 | 0.828（251/303） | 未达 |
| 水平偏移 p95（追踪机 vs 目标 XY） | ≤ 2.0 m | 估计口径：`basic/pn` 5.43/2.53 m、`pn_mppi/pn_nmpc` 2.57/1.54 m；目标 odometry 口径 7.02/3.36/4.12/2.04 m（含约 1 m 原点差，见 4.1） | 未达 |
| 跟踪段丢失（`lost`） | 首次进入跟踪后无 > 1 s 连续丢失 | 最长 1.06 s（1 次） | 未达 |
| 记录完整性 | 4 类 CSV + 图 + 指标齐全 | `vision_samples.csv`、`yolo_detections.csv`、4×`gazebo_samples.csv`、图与 `metrics.csv` 齐全；未采数据集/标注 | 齐全 |
| 运行时长 | 完成 `sim_time`，无异常退出 | 0–40 s 完整记录 | 达标 |

结论：本次 P6 **未通过**（6 项中 4 项未达）。

- 帧级检出率 0.828 低于 0.9，52 帧拒绝全部为 `no_drone_detection`；其中可能混有目标短暂离开
  相机足印的时段，建议用 `camera_recorder` 画面复核后再区分“模型漏检”与“目标不在视野”。
- 水平偏移按控制器实际使用的估计口径，只有 `pn_nmpc` 的 p95（1.54 m）在 2.0 m 以内，`basic` 达
  5.43 m；odometry 口径还叠加约 1 m 的两机原点差（见 4.1），`basic` 达 7.02 m。
- 跟踪段出现 1 次 1.06 s 的连续丢失，超过 `vision_loss_s = 1.0 s`，估计器短暂进入 hold，“无 >1 s
  连续丢失”不再达标。
- 若要按 P6 门槛给出判定，需先对齐两机本地原点（同点 spawn 或原点转换）后重跑；本批记录可作为
  视觉链路与四算法闭环的验证。

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

同一天还跑了一轮**四算法各 40 s 的视觉闭环对照**（调参后参数，`target_source=vision`）：EMPC 最大水平距离 3.06 m、平均 1.30 m，均为四者最低；MPPI 控制能量与 yaw rate 最低但最大距离 13.59 m。该批数据已被 2026-10-04 四算法复测取代，当前口径见第 4 节与 [算法说明](2d_simulation_guidance_overview.md) 12.1～12.2 节。

### 4.4 EMPC 画面保持（FOV）惩罚受控对比（2026-10-04）

EMPC 原代价函数只约束水平距离，目标接近画幅边缘时没有主动回中梯度。2026-10-04 加入按标称下视相机投影的画面偏移惩罚（`nmpc_w_fov=120`、`fov_soft_margin=0.5`、`fov_violation_cap=3.0`，见 [算法说明](2d_simulation_guidance_overview.md) 7.4.1 节）后，在同一套 Gazebo + YOLO 闭环下做了受控前后对比：circle、40 s、`pn_nmpc`、无阴影世界、两机 spawn `48,0` / `47,0`，两轮使用同一启动命令与同一 PX4/Gazebo 实例，中间只切换 `nmpc_w_fov`；原始记录分别在 `outputs/gazebo2d_vision_runs_nofov/` 与 `outputs/gazebo2d_vision_runs_fov/`（已不在当前 `outputs/` 中）。

| 指标 | w=0（无 FOV） | w=120（有 FOV） |
| --- | ---: | ---: |
| 最小 / 最大 / 平均水平距离/m | 0.060 / 3.15 / 1.404 | 0.033 / 2.47 / 1.434 |
| 控制能量（中位 dt 口径） | 658.4 | 697.2 |
| yaw rate mean / variance | 1.528 / 3.076 | 1.322 / 2.537 |
| 真值画面偏移 max / mean / p95 | 0.287 / 0.106 / 0.224 | 0.360 / 0.115 / 0.251 |
| 偏移 > 0.85 占比 | 0 | 0 |
| 估计器可用（tracking/coast）占比 | 0.998 | 0.993 |
| 最长量测间隔（0–40 s） | 1.00 s | 0.90 s |

结论：两轮追踪精度基本一致（平均距离差 2%），画面偏移都远小于安全边界 0.85——在 EMPC 的跟踪误差范围内 circle 默认机动不会让目标压边，FOV 项是"压边保险"而不是主控制器；有 FOV 轮控制能量 +5.9%、yaw rate mean 低 13.5%，差异量与单次跑批的正常波动相当。惩罚的有效性由反事实回放确认：把权重置 0 重放同一批状态时，20/799 个周期选择了不同候选（最大加速度差 2.6 m/s²），且都出现在预测偏移超过软边界的周期，即它在"即将压边"时提前回中。作为对照，同批 2026-10-01 记录中未加 FOV 项的 `pn_mppi` 真值偏移 max = 1.83、42.5% 的时间超过 0.85；本轮只对 `pn_nmpc` 生效。完整离线对照与解读见 [算法说明](2d_simulation_guidance_overview.md) 12.5 节。

### 4.5 高度 × 速度矩阵（2026-10-04）

在 `circle` / `pn_nmpc` / YOLO 闭环下做了追踪机高度 × 目标速度全矩阵复测：高度 8/6/5/4 m × 目标速度
3.0/2.5/2.0/1.5/1.0 m/s 共 20 格，每格 40 s 跟踪窗口，全部用新增的 `target_speed_scale` 重跑
（scale = 速度/3.0，只缩 `circle` 角速度，半径与起点不变）。跨机坐标常值偏移按每轮
`vision_samples.csv` 的 `mean(target_est - target_ref)` 逐轮校正（约 `(-1.0, 0) m`），下表距离为校正后的
真实世界水平距离；原始记录在 `outputs/alt_speed_matrix/`（生成物，不入库）。矩阵跨两个 Gazebo 进程
完成（中途服务器重启），此前同条件对照显示批次对检出率的影响约 1.5 个百分点，远小于本矩阵的高度趋势。

**校正后水平距离 mean / m**

| 高度 \ 速度 | 1.0 m/s | 1.5 m/s | 2.0 m/s | 2.5 m/s | 3.0 m/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 8.0 m | 0.99 | 0.94 | 0.95 | 0.89 | 0.88 |
| 6.0 m | 1.38 | 1.02 | 1.13 | 1.15 | 1.35 |
| 5.0 m | 0.96 | 1.34 | 1.18 | **10.71** | 1.71 |
| 4.0 m | 0.98 | 1.69 | **14.35** | **6.47** | **9.99** |

**校正后水平距离 max / m**

| 高度 \ 速度 | 1.0 m/s | 1.5 m/s | 2.0 m/s | 2.5 m/s | 3.0 m/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 8.0 m | 1.80 | 1.98 | 1.85 | 2.52 | 2.87 |
| 6.0 m | 5.20 | 2.76 | 3.34 | 4.84 | 3.72 |
| 5.0 m | 2.71 | 4.71 | 3.77 | **21.26** | 4.86 |
| 4.0 m | 2.55 | 4.84 | **23.54** | **21.16** | **20.08** |

**YOLO 帧级检出率（跟踪窗口）**

| 高度 \ 速度 | 1.0 m/s | 1.5 m/s | 2.0 m/s | 2.5 m/s | 3.0 m/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 8.0 m | 0.61 | 0.59 | 0.72 | 0.82 | 0.83 |
| 6.0 m | 0.29 | 0.34 | 0.42 | 0.62 | 0.48 |
| 5.0 m | 0.19 | 0.22 | 0.26 | 0.06 | 0.29 |
| 4.0 m | 0.17 | 0.13 | 0.01 | 0.10 | 0.06 |

加粗格为估计器失锁后进入 hold 的发散结果（max 20 m 量级不是控制器稳态误差）。min 距离、量测率、
估计器可用占比与最长丢失见同目录 `summary_matrix.md`。

**关键格复跑一致性**（低空边界处单轮结果会翻转，对 4 个关键格各补跑 1 次）：

| 格 | 原始 mean/max / m | 原始可用占比 | 复跑 mean/max / m | 复跑可用占比 | 结论 |
| --- | --- | ---: | --- | ---: | --- |
| 4.0 m / 1.0 m/s | 0.98 / 2.55 | 0.752 | 1.08 / 3.22 | 0.672 | 稳定成功（2/2） |
| 4.0 m / 2.0 m/s | 14.35 / 23.54 | 0.074 | 14.54 / 23.32 | 0.079 | 稳定失败（2/2） |
| 4.0 m / 1.5 m/s | 1.69 / 4.84 | 0.628 | 6.40 / 22.26 | 0.377 | 翻转（脆弱） |
| 5.0 m / 2.5 m/s | 10.71 / 21.26 | 0.256 | 1.95 / 4.99 | 0.757 | 翻转（脆弱） |

结论：

- **目标速度对平均追踪误差几乎没有影响**：8 m 行 1.0–3.0 m/s 平均误差 0.88–0.99 m，速度只影响瞬态
  最大误差（低速时目标机动温和，max 从 2.87 m 降到 1.80 m）。原因是追踪机控制余量远大于 circle 目标
  的机动能力（目标向心加速度仅 0.08–0.75 m/s²），误差由控制律的站位取舍决定，不由速度决定。
- **高度是主导因素**：8 m 全档稳定（可用占比 ≥0.99）；6 m 仍可用但裕度下降（平均 1.02–1.38 m）；
  5 m 进入脆弱区；4 m 为失效边界——相机足印随高度线性缩小（4 m 时仅约 7.3 × 5.5 m），跟踪时的机体
  倾斜（平均指令加速度 4–5 m/s²，对应 22–29°）在画面偏移里占比过大，目标容易出画。
- **失锁是单向锁死**：丢失超过 `vision_loss_s=1.0 s` 即 hold 悬停，没有主动搜索/再捕获；低空一旦出画，
  误差直接发散到 20 m 量级，所以低空失效呈“悬崖”而不是平滑退化。4 m 下 1.0 m/s 能稳定，是因为目标
  在小窗口内停留久、闭环有时间修正。
- **检出率随高度塌陷**（8 m 0.59–0.83 → 6 m 0.29–0.62 → 5 m 0.06–0.29 → 4 m 0.01–0.17），量测率随之
  下降、估计器 coast 时间变长；另外 8/6 m 行低速档检出率反而更低（8 m：0.83@3.0 m/s → 0.61@1.0 m/s），
  两个独立 Gazebo 进程方向一致，机制未定（低速时追踪机更贴近目标、指令加速度均值反而更高，
  5.42 vs 4.02 m/s²，可能与近距视角/姿态有关）。

## 5. 桌下遮挡参数与目标速度扫描（2026-10-05）

在 `table_occlusion` / `pn_nmpc` / YOLO 视觉闭环下，对三个 launch 可覆盖参数（`pn_k_close`、
`pn_v_des_along_los`、`nmpc_w_pn`）与目标机速度做了系统扫描。全部 trial 为 Gazebo + PX4 SITL 双机实飞
闭环，追踪机只消费 `/vision/target_pose`；参数通过 launch 覆盖，不改 `config.py` 默认值。共 67 个有效
trial（含 4 个 PID 对照；另有 8 次因 6.2 的 gz 物理崩溃失败的 trial 未计入），每配置 3 次取中位数，
个别明确退化的配置只跑 1 次。

**指标口径**：与 `plot_gazebo_csv.compute_gazebo_metrics` 一致——桌下样本排除，`mean/max` 为可见段误差；
另记录出桌后 5 s 重获段（`post_*`）、终段（最后 7 s）饱和与 `x>10` 后的绕圈路径长度。汇总表
`outputs/sweep2/sweep2_summary.csv`（生成物，不入库）。

### 5.1 参数扫描（目标速度 0.5 m/s）

中位数（n=3；`pn_k_close` / `pn_v_des_along_los` / `nmpc_w_pn`）：

| 配置 | mean / m | max / m | 能量 | 备注 |
| --- | ---: | ---: | ---: | --- |
| 0.15 / 8.0 / 0.75 | 0.294 | 2.105 | 36 | |
| 0.25 / 8.0 / 0.5 | 0.299 | 1.877 | 55 | |
| 0.10 / 8.0 / 0.5 | 0.301 | 2.252 | 20 | |
| **0.10 / 6.0 / 1.0** | 0.305 | 1.947 | **16** | 精度-能耗综合最优 |
| 0.25 / 3.0 / 1.0 | 0.321 | 2.003 | 19 | |
| 0.10 / 8.0 / 1.0 | 0.339 | 2.205 | 24 | |
| 0.25 / 6.0 / 1.0 | 0.361 | 2.205 | 48 | |
| 0.25 / 8.0 / 1.0（当前默认） | 0.369 | 1.760 | 92 | 3 次中 1 次视觉离群坏 run，见 5.3 |
| 0.15 / 8.0 / 1.0 | 0.374 | 2.388 | 41 | |
| 0.25 / 2.0 / 1.0 | 0.393 | 2.216 | 15 | 最省能 |
| 0.25 / 10.0 / 1.0 | 0.408 | 1.990 | 110 | |

明确退化（单次）：`pn_k_close` 0.35 / 0.5（mean 0.445 / 0.465，能量 134 / 213）、
`pn_v_des_along_los` 12（0.466 / 137）、`nmpc_w_pn` 1.5 / 2.0（0.575 / 0.851，能量 170 / 244，
并出现终段饱和与绕圈路径增大）。

结论：

- **安全区**：`pn_k_close ≤ 0.25`、`pn_v_des_along_los` 2～10、`nmpc_w_pn ≤ 1.0`；
  **不要用 `pn_k_close ≥ 0.35` 或 `nmpc_w_pn ≥ 1.5`**。
- 精度组间差异（≤0.08 m）与遮挡期视觉离群噪声同量级，不具决定性；差异主要体现为能耗
  （低参组合比默认省 60%～90%）。
- 组合扫描（0.10/3.0、0.10/6.0、0.10/8.0/0.5、0.15/3.0、0.15/8.0/0.75）没有超过单参数最好值，
  其中 0.10/6.0/1.0 综合最好。

### 5.2 目标速度与 PID 对照

`target_speed_scale` 2.0 / 3.0（巡航 1.0 / 1.5 m/s），中位数（mean / m、max / m、能量；PID 为 2 次）：

| 速度 | base 0.25/8.0/1.0 | low 0.10/6.0/1.0 | PID |
| --- | --- | --- | --- |
| 1.0 m/s | 0.387 / 2.261 / 89（1 次坏 coast） | 0.419 / 2.524 / 20 | **0.293** / 2.609 / 30 |
| 1.5 m/s | **0.426 / 2.379 / 88** | 0.536 / 3.527 / 44 | 0.433 / 4.016 / 140 |

- base 跨速度鲁棒性最好：1.5 m/s 时均值与 PID 持平、最大误差更小（2.38 vs 4.02 m）、能耗约 2/3；
  low 在 1.5 m/s 明显退化（终端接近增益不足以跟上更高闭合速度）。
- 建议全局默认保持 0.25/8.0/1.0；仅桌下低速场景可覆盖 0.10/6.0/1.0（默认速度 mean 0.305 vs 0.369、
  能量 16 vs 92）。

### 5.3 遮挡期视觉离群与 `yolo_conf` A/B

坏 run 机制（`s1_base` 第 1 次）：目标在桌下时 YOLO 偶发低置信度误检（score 0.31～0.33，正常检测的
5% 分位约 0.60～0.77；bbox 仅 39 px、`pixel_error_vs_truth_px` 139 px、位置误差 1.55 m），α-β 估计器
接受后估计速度从 +0.21 翻到 −0.38 m/s，coast 反向、追踪机停在后方，出桌后最大误差 3.67 m。

`yolo_conf` 0.25（默认）与 0.5 各 3 次（base 配置）：

| `yolo_conf` | 被接受量测最低分 | 坏 coast | mean / max / 能量（中位） |
| --- | ---: | ---: | --- |
| 0.25 | 0.253～0.374 | 1/3 | 0.369 / 1.760 / 92 |
| 0.5 | 0.531～0.570 | 0/3 | 0.369 / 1.893 / 96 |

0.5 阈值只滤除离群误检、不伤主分布。**建议闭环运行加 `yolo_conf:=0.5`**；本轮未改 launch 默认值
（仍 0.25），样本量 3+3，方向明确但建议后续补测确认。

### 5.4 四算法对照（2026-10-05 晚）

在第 5 节参数扫描之外补了一组四算法视觉闭环对照：`basic`、`pn`、`pn_mppi`、`pn_nmpc` 各 1 次，
场景与第 5 节一致（`table_occlusion`、`yolo26_baseline_detfly_table_occlusion_ft100`、
`vision_source=yolo` + `target_source=vision`、`use_sim_time=true`、目标速度 0.5 m/s、追踪机/目标机
出生点 `-2,0` / `0,0`）。PN/EMPC 三者统一使用遮挡场景 low 参数组（`pn_k_close=0.10`、
`pn_v_des_along_los=6.0`、`nmpc_w_pn=1.0`）；`yolo_conf` 保持 launch 默认 0.25；无 QGC，用
`tools/px4_gcs_heartbeat.py` 满足解锁前置。指标口径与第 5 节相同（桌下样本排除，post_* 为出桌后 5 s）。

| 算法 | mean / m | max / m | min / m | post mean / m | post max / m | 控制能量 | yaw mean | yaw var | 量测可用占比 | 量测率 / Hz | 检出率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| basic | 1.791 | 3.955 | 0.012 | 2.194 | 3.243 | 1082 | 1.437 | 1.356 | 0.757 | 3.7 | 0.537 |
| pn | 1.256 | 3.790 | 0.006 | 1.717 | 3.768 | 40.8 | 0.518 | 0.657 | 0.753 | 5.8 | 0.724 |
| pn_mppi | 0.747 | 2.866 | 0.014 | 2.094 | 2.866 | 52.0 | 0.883 | 1.004 | 0.743 | 5.6 | 0.711 |
| pn_nmpc | **0.393** | **2.584** | 0.008 | **1.666** | **2.584** | **16.4** | 1.178 | 1.371 | 0.742 | 5.7 | 0.713 |

检出率为整段记录的帧级检测率（含准备段与遮挡段）；量测可用占比为追踪窗口内估计器非 `lost` 的比例，
量测率为被 α-β 接受的量测数 / 追踪时长。视觉链路四者一致：检测端到端时延 p50 8.3～8.8 ms /
p95 12.4～12.5 ms，推理 p50 2.05 ms；被接受量测的位置误差 p50 0.058～0.225 m / p95 0.230～0.516 m
（`basic` 因机动剧烈最大）；最长连续丢失 10.2～11.2 s，全部落在桌下遮挡段（含目标在桌心停稳 3 s），
拒绝原因全部为 `no_drone_detection`（另有各 1 次 `pose_cache_stale`）；出桌后均重新接上量测并恢复导引。
受追踪机站位影响，`basic` 的检出率只有 0.537，其余三者约 0.71。

结论：

- `pn_nmpc` 精度与能耗同时最优（平均 0.393 m、最大 2.584 m、能量 16.4），出桌后 5 s 平均恢复到 1.7 m；
- `pn_mppi` 次之（0.747 m），控制能量约为 EMPC 的 3 倍；纯 `pn` 站位较松散（1.256 m），但能量低（40.8）；
- `basic` 持续满推力绕飞（轨迹图呈多个环），平均 1.791 m、能量 1082、检出率最低，符合“机动越剧烈、
  画面越难保持”的既有结论。

原始记录（生成物，不入库）：

```text
outputs/gazebo2d_vision_runs/table_occlusion/{basic,pn,pn_mppi,pn_nmpc}/gazebo_samples.csv
outputs/table_occlusion_vision/{basic,pn,pn_mppi,pn_nmpc}/
outputs/table_occlusion_vision/plots/trajectories_2x2.png
```

异常说明：`pn` 的第一次运行（在 `basic` 之后，追踪机刚从自动降落状态重新起飞）出现约 −0.5 m/s² 的
横向漂移、车辆实际响应与指令方向相反，闭环发散到 10 m，已作废重跑；重跑同参数正常，期间用
`/px4_1/fmu/out/vehicle_control_mode` 确认 Offboard + 速度 + 加速度标志均开启。异常记录保留在
`outputs/table_occlusion_vision/excluded_pn_run1/`。4 次对照各只跑 1 次，供横向比较；需要更稳的结论时
按第 5 节口径补重复。

## 6. 基础设施问题定位（2026-10-05）

### 6.1 gz sim OOM：GstCameraSystem 无界帧队列

- 现象：gz server（进程名 `ruby`）RSS 以约 118 MB/s 线性增长，约 15～20 min 达 ~60 GB 后被 OOM 杀掉；
  62 GB RAM + 31 GB swap 全部耗尽，表现为服务器反复重启。三次 OOM 日志的 anon-rss 均约 60 GB。
- 根因：PX4 通过 `GZ_SIM_SERVER_CONFIG_PATH`（`build/px4_sitl_default/rootfs/gz_env.sh` 注入）加载的
  world 插件 `libGstCameraSystem.so` 把 1280×960@30 的相机帧推入无上界的 GStreamer `appsrc` 队列。
  本机 NVIDIA 驱动（RTX PRO 5000 / 595.91）拒绝插件设置的旧版 NVENC 属性，编码管线静默卡死、
  不再消费帧，帧以约 110 MB/s 持续囤积。上游修复见未合并的 PR #27873。
- 证据：有插件时实测 15 s 增长约 1.7 GB（≈118 MB/s，与相机帧率吻合）；从 `server.config` 过滤掉该
  插件后，RSS 3 min 稳定在 854 MiB，相机话题与视觉链路正常。
- 处置（临时，未固化进仓库）：启动 gz 前生成过滤配置并覆盖环境变量；本链路不使用 PX4 RTP 推流，
  移除无副作用：

  ```bash
  grep -v "GstCameraSystem" "$PX4/src/modules/simulation/gz_bridge/server.config" > /tmp/server_no_gst.config
  export GZ_SIM_SERVER_CONFIG_PATH=/tmp/server_no_gst.config
  ```

  备选永久方案：注释 PX4 `server.config` 中该行，或回移 PR #27873 后重编插件。

### 6.2 DART/ODE 物理断言崩溃

- 现象：连续运行约 30～40 min 后 gz 中止，日志为
  `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact && aabbBound < dMaxIntExact" failed in collide()`
  （`dart::collision::OdeCollisionDetector::collide`），即物理状态出现 NaN/越界；与 6.1 的内存问题相互独立。
- 处置：扫参脚本每 20 个 trial 主动重启 gz/PX4；target 150 s 未 ready 时快速失败并强制下一轮重启。
  崩溃后不再出现“每个 trial 空等 10 分钟”的级联失败。

## 7. 已知边界与风险

- **零样本域差异是最大风险**：Det-Fly 是真实天空背景侧视图，Gazebo 是俯视渲染；P3 门槛不达标时再评估是否需要域适配微调。
- `yolo_model_path` 必须是完整文件路径；worker 启动失败（如路径不存在）会先以 `[worker]` 前缀转发 worker stderr，再按 `worker_restart_limit` 重试并停止检测，不要复制文档中的路径占位符。
- 量测精度设计预期为厘米级（像素噪声 + 0.15 m 目标高度平面假设 + 毫秒级同步残差），但 2026-10-04 复测的
  同源残差呈约 1 m 的常值偏差（见 4.1），指向两机本地原点/坐标对齐问题而非像素噪声；对齐前不要引用任何
  误差数字作为精度指标。`pixel_error_vs_truth_px` / `position_error_vs_odom_m` 与量测同源，**不是独立标定**。
- **两机本地原点不一致会让跨机指标带常值偏差**：`vision_adapter` 的“vs odom”诊断列和
  `gazebo_samples.csv` 的距离列只对原点一致的两机成立；两机在不同点 spawn（本批相差 1 m）
  会产生约 1 m 的常值 xy 偏差，需要两机同点 spawn 或在 ROS 边界显式转换，见 [视觉设计参考](vision_design.md) 1.1 节。
- 正下方视线接近奇异（`r_norm→0`），`pn_guidance()` 在零偏移附近可能抖动；先用 `pn_mppi`/`pn_nmpc` 的平滑项复测，
  必要时记录“最小保持间距”调参，不新增算法。
- PX4 SITL 的 `hrt` 由 Gazebo 时钟驱动（`GZBridge::clockCallback`），但本链路不依赖该事实：位姿缓存使用接收时的
  仿真时间，`pose_match_dt_ms` 暴露残差。
