# 视觉接口设计参考

本文件合并原相机接入与 YOLO 接入计划中仍需保留的接口约定。运行步骤和参数表统一见 [模块 README](../README.md#下视相机与视觉闭环)，历史验证记录见 [验证记录](yolo_vision_closed_loop_results.md)。

## 1. 相机几何与坐标约定

### 1.1 坐标与外参

定义 $W$ 为约定公共原点的 ENU，$B$ 为机体 FLU（前、左、上），$C$ 为光学系（右、下、前）。PX4 四元数通常表示 FRD 机体到 NED 世界的旋转，不能仅左乘 NED→ENU 矩阵就称为 FLU 姿态。

在确认 `VehicleOdometry.pose_frame` 为 NED 后：

$$R_{W\leftarrow B}=R_{ENU\leftarrow NED}\,R_{NED\leftarrow FRD}(q)\,\operatorname{diag}(1,-1,-1).$$

验证四元数有限、非零并归一化；不支持的 frame 或非法姿态直接拒绝。不要沿用 yaw 工具对非法输入返回零角的宽松行为。

安装参数统一定义为相机 link 相对于机体 FLU 的平移 $t_{B,L}$ 和旋转 $R_{B\leftarrow L}$。光学轴转换仅定义一次：

$$R_{L\leftarrow C}=\begin{bmatrix}0&0&1\\-1&0&0\\0&-1&0\end{bmatrix},\quad R_{B\leftarrow C}=R_{B\leftarrow L}R_{L\leftarrow C}.$$

这里假设相机 link 为 x 前、y 左、z 上；若 SDF 的 sensor pose 另有变换，必须先合并。位姿合成为：

$$p_{cam,W}=p_{body,W}+R_{W\leftarrow B}t_{B,L},\qquad R_{W\leftarrow C}=R_{W\leftarrow B}R_{B\leftarrow C}.$$

对于无额外旋转的标称 $R_{B\leftarrow L}=R_y(\pi/2)$，得到：

$$R_{B\leftarrow C}=\begin{bmatrix}0&-1&0\\-1&0&0\\0&0&-1\end{bmatrix}.$$

此矩阵是**完整安装链的结果**，不能再次作为 link→optical 旋转叠加。预期安装平移为 `[0, 0, 0.10]` m，而非零；核验记录见 README。

两机各自的 PX4 本地原点不保证与 Gazebo world 或彼此相同。实施前在多个已知位置检查对齐；若不一致，应在 ROS 边界显式转换到公共世界系，并记录各自原点偏移/旋转。只交换 NED/ENU 轴不会消除原点偏差。

### 1.2 内参与射线求交

使用 $K$ 中独立的 $f_x,f_y,c_x,c_y$。ROS 路径检查内参为有限数、焦距为正、尺寸有效，且畸变为零；不支持的畸变明确报错，不能静默忽略。离线理想方形像素模型可使用：

$$f_x=f_y=\frac{W/2}{\tan(\theta_{hfov}/2)}.$$

1280×960、1.74 rad 对应约 539.94 px。主点以 `CameraInfo` 为准，不强制要求正好等于 `(640, 480)`。

令 $z_t$ 为公共 ENU 下的目标水平面高度：

$$r_C=[(u-c_x)/f_x,\ (v-c_y)/f_y,\ 1]^\top,\qquad r_W=R_{W\leftarrow C}r_C,$$
$$s=\frac{z_t-p_{cam,W,z}}{r_{W,z}},\qquad P_W=p_{cam,W}+s r_W.$$

无需单位化射线。要求相机高于平面、像素在图像内、射线朝下且远离平行、交点位于前方，所有值有限；否则返回 `None`。正投影还需检查光学深度为正及像素是否在图像范围内。近乎平行的判定使用归一化方向的 z 分量，避免阈值依赖射线长度。

`target_base_altitude` 是目标控制高度，不保证等于实际高度或 bbox 中心所代表物理点的高度。它只作为固定平面假设，不能据此承诺真实图像位置误差小于 5 cm。truth 正投影输入使用参考 XY 和同一个 $z_t$，同时记录 odometry 实际 z 与平面的差值。

### 1.3 线性灵敏度与局部雅可比

相机光轴铅垂时，令 $h=p_{cam,W,z}-z_t$：

$$\Delta p_W=R_z(\psi)\begin{bmatrix}0&-h/f_y\\-h/f_x&0\end{bmatrix}\begin{bmatrix}u-c_x\\v-c_y\end{bmatrix}.$$

这是水平姿态下的精确仿射映射；倾斜时不用它解算位置或传播协方差。设 $a=\partial r_W/\partial u=R_{W\leftarrow C}[:,0]/f_x$，$b=\partial r_W/\partial v=R_{W\leftarrow C}[:,1]/f_y$：

$$J=\frac{z_t-p_{cam,W,z}}{r_{W,z}}\left[\ a_{xy}-\frac{r_{W,xy}}{r_{W,z}}a_z,\quad b_{xy}-\frac{r_{W,xy}}{r_{W,z}}b_z\ \right].$$

因此 $J$ 依赖**像素、姿态和目标平面**。像素噪声传播为 $\Sigma_{xy}=J\operatorname{diag}(\sigma_u^2,\sigma_v^2)J^\top$；此处只计像素噪声，不把它宣称为包含姿态、安装、时间和高度误差的总不确定度。

以相机高度 8 m、目标平面 1 m、焦距 540 px 为示例：1 px≈12.96 mm，5 px≈6.5 cm；水平覆盖约 16.6×12.4 m；0.35 m 宽物体约 27 px。这里的 8 m 是**相机高度**，不是加上安装偏移前的机体高度。5° 倾斜的光轴足印偏移约 0.61 m，不是整幅图像的统一平移。

## 2. 推理进程与通信协议

ROS 包保持系统 Python（3.12），**不 import torch**；模型推理在 conda `ultralytics` 环境的常驻子进程里完成，
两者用 stdin/stdout 二进制协议通信。理由：

- 直接复用已有环境与 FP16 TensorRT 引擎，不新增第二套 torch 环境；
- ROS 包依赖仍由 rosdep/apt 提供（`sensor_msgs`、`vision_msgs`、`geometry_msgs`）；
- ultralytics 仓库不改动，worker 脚本随 `gazebosimulation2d` 安装到 share。

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

## 3. 时间基准与位姿缓存

- `src/gazebosimulation2d/config/camera_bridge.yaml` 提供 `/clock` 桥接（`gz.msgs.Clock → rosgraph_msgs/msg/Clock`）。
- 视觉链路三节点（`vision_detector`、`vision_adapter`、`guidance_node_2d`）在视觉模式下统一 `use_sim_time:=true`；
  `vision_source=yolo` 时若 `use_sim_time=false`，`vision_adapter` 直接报错退出。
- `vision_adapter` 维护追踪机位姿缓存 `(t_sim_ns, position_ned, quaternion_wxyz)`：
  收到检测后用图像 `stamp` 二分查找，位置线性插值、四元数最短弧插值后归一化，再经
  `camera_pose_from_odometry()` 合成相机位姿；超出 `pose_match_tolerance_s`、缓存落后于 `pose_cache_max_age_s`
  或落在最新样本之后（future）时拒绝量测并记录原因。
- PX4 消息 `timestamp` 字段使用宿主墙钟（`time.time_ns() // 1000`），把 `use_sim_time` 从 PX4 时间里解耦；
  默认路径（`use_sim_time=false`）下与现有行为等价。
- 启动防呆：`use_sim_time=true` 时若 2 s 墙钟内节点时钟仍无推进，打印明确错误并退出（避免“静默不动作”）。
- 已知残差：odometry 接收时刻 ≠ 采样时刻，量级为传输抖动（毫秒级）；不宣称采样同步，误差在 CSV 中量化。

## 4. 检测消息约定

| 项 | 约定 |
| --- | --- |
| 输入 | `/camera/image_raw`（BEST_EFFORT/SENSOR_DATA），按 `encoding` 处理（rgb8/bgr8/rgba8/mono8） |
| 输出 | `/camera/detections`，`vision_msgs/Detection2DArray`，BEST_EFFORT + VOLATILE |
| Header | 数组与检测元素复制源图像 `stamp`/`frame_id` |
| 类别 | ultralytics 的 `uav` → `drone`；score ∈ [0,1]；单目标取最高分 |
| bbox | `bbox.center.position.x/y` 使用**原图坐标**（ultralytics 已做 letterbox 反变换，禁止二次缩放）；`size_x/size_y>0` |
| 未检出 | 发布空数组，下游据此区分“没有目标”与“检测节点挂了” |
| 统计 | `yolo_detections.csv`：每处理帧一行 `stamp_s,u,v,w,h,score,inference_ms,e2e_ms,detections` |

## 5. 估计器接口约定

`src/pythonsimulation2d/target_filter.py`（无 ROS，可离线单测），供 `guidance_node_2d` 使用：

$$\hat p_k^-=\hat p_{k-1}+\hat v_{k-1}\Delta t,\qquad
\hat p_k=\hat p_k^-+\alpha\,(z_k-\hat p_k^-),\qquad
\hat v_k=\hat v_{k-1}+\frac{\beta}{\Delta t}(z_k-\hat p_k^-)$$

- 滤波与预测均只作用于 XY（z 固定为目标平面高度）；$\Delta t$ 用**量测 stamp 差**（仿真时间）并夹在 `[min_dt, max_dt]`；
- 加速度由速度一阶低通差分给出（`vision_accel_tau_s`，供 `pn_mppi`/`pn_nmpc` 的预测使用）；
- 可选马氏门控（`vision_gate_sigma`，用 `/vision/target_pose` 的 XY 协方差块）拒绝离群量测；
- 状态机：`tracking → coast`（无新量测超过 `vision_coast_s=0.3 s`）`→ lost`（超过 `vision_loss_s=1.0 s`）`→ tracking`（重捕获）；
- `lost` 时 `guidance_node_2d` 进入 hold：`velocity=[0,0,0]`、`accel=0`、保持 yaw（现有 `_publish_pursuer_setpoint` 在
  `accel=0` 时会发当前速度，hold 必须单独发零速 setpoint）。

### 5.1 初始捕获（仿真约定）

`target_source=vision` 时 `guidance_node_2d` 把追踪机起飞保持点设为场景起点 XY（高度 `pursuer_fixed_altitude`），
目标机在准备阶段停在同一起点，因此开始跟踪时目标位于相机视野内。这是仿真中代替外部引导/视觉移交的初始线索；
`vision_fallback=odometry` 仍预留给未来的远距离捕获，本轮未实现。目标中途脱离视野仍走 coast → lost → hold，
不提供搜索重捕获。
