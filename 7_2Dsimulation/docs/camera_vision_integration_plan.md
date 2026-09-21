# 7_2Dsimulation 下视相机与视觉接口接入计划

> 状态：待实施。本文档是实施前的设计基线，代码尚未落地。
> 目标版本：ROS 2 Jazzy + PX4 v1.16 SITL + Gazebo Harmonic (gz-sim 8)。

## 1. 目标与范围

### 1.1 目标

1. 在 `7_2Dsimulation` 的 Gazebo 闭环仿真中给**追踪机**加装一台固定朝下、俯视地面的单目相机。
2. 通过 `ros_gz_bridge`（A 方案）把 gz 相机话题桥接到 ROS 2，可在 `/camera/` 前缀下直接订阅。
3. 建立**像素偏差 → 地面位置偏差**的显式映射关系：既给出闭式线性映射（水平姿态下精确，用于增益设计与噪声换算），也给出精确射线-平面反投影（含姿态补偿，用于把检测结果换算成目标 XY 量测）。
4. 为目标识别预留**独立 YOLO 模块的接口契约**（本仓库不实现检测），并提供 `truth` 伪检测源，使整条链路在 YOLO 就绪前就能端到端验证。
5. 不改动已验证的导引闭环行为：本轮 `guidance_node` 的导引逻辑不变，视觉量测只发布、不消费。

### 1.2 非目标（本轮不做）

- 不实现目标检测/跟踪网络（YOLO 由独立模块负责）。
- 不把视觉量测接入导引闭环（`target_source:=vision` 留到后续阶段）。
- 不做 FOV 约束、遮挡、漏检建模，也不改二维导引算法与指标定义。
- 不修改 PX4/Gazebo 的安装目录与模型文件（相机使用 PX4 自带模型）。
- 不在仓库 launch 中启动 Gazebo / PX4 SITL / Micro XRCE-DDS Agent / QGC。

## 2. 已定决策

| 编号 | 决策 | 说明 |
| --- | --- | --- |
| D1 | 相机只挂在追踪机 | 追踪机是 `-i 0` 实例，也是负责拉起 gz world 的实例；目标机保持 `gz_x500` 不变 |
| D2 | 桥接用 `ros_gz_bridge`（A 方案） | 依赖由使用者自行 apt 安装，launch 内以条件节点形式并入 `guidance.launch.py` |
| D3 | 相机 ROS 话题统一 `/camera/` 前缀 | `/camera/image_raw`、`/camera/camera_info`、`/camera/detections`、`/camera/image_annotated` |
| D4 | 像素→位置映射模块化 | 纯数学放在 `pythonsimulation2d/camera_geometry.py`，ROS 侧只调用，禁止复制公式 |
| D5 | YOLO 为独立模块 | 本仓库只定义契约：输入 `/camera/image_raw` + `/camera/camera_info`，输出 `vision_msgs/Detection2DArray` |
| D6 | 目标所在平面高度 `z_target = 1.0 m` | 复用已有参数 `target_base_altitude`；有效高度 $h_{eff} = h_{cam} - z_{target}$ |

## 3. 数据流

```text
PX4 追踪机 (-i 0, x500_mono_cam_down_0)            目标机 (-i 1, x500_1)
        │  gz camera_link/sensor/imager                    │
        ▼                                                  │
  gz.msgs.Image / CameraInfo            VehicleOdometry ◄──┘ fmu/out/vehicle_odometry
        │                                       ▲
        │ ros_gz_bridge (GZ_TO_ROS, lazy)       │
        ▼                                       │
  /camera/image_raw ──────────► [ YOLO 独立模块 ] ──► /camera/detections
  /camera/camera_info ────────► (本仓库不实现)              │
        │                                                  │
        │                                    vision_adapter│ (truth 模式另发 /camera/detections_truth)
        │                                                  ▼
        │                            camera_geometry.pixel_to_ground()  (pythonsimulation2d)
        │                                                  │
        │                                                  ▼
        └────► /camera/image_annotated              /vision/target_pose
               (仅调试，truth 模式由适配节点发布)     (geometry_msgs/PoseWithCovarianceStamped)
                                                          │
                                                          ▼
                                            后续阶段：guidance_node（target_source:=vision）
```

## 4. Gazebo / PX4 侧

### 4.1 相机模型（PX4 自带，不改动 SDF）

路径：`~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf`

| 项 | 值 |
| --- | --- |
| 机架 | `4014_gz_x500_mono_cam_down` |
| gz 模型名 | `x500_mono_cam_down`，按 `<model>_<实例号>` 规则生成 → `x500_mono_cam_down_0` |
| 相机 link / sensor | `camera_link` / `imager` |
| 安装 | `<pose>0 0 .10 0 1.5707 0</pose>` + 绕 `base_link` 固定关节；标称光轴朝下（机体 $-z$），安装偏移/旋转**以 P2 标定实测为准** |
| 分辨率 | 1280 × 960 |
| 水平视场 | `1.74 rad` ≈ 99.7° |
| 帧率 | 30 Hz |
| 裁剪 | near 0.1 m，far 3000 m |
| 其它 | `<always_on>1</always_on>`、`<visualize>true</visualize>`（gz GUI 会画出视锥） |

### 4.2 追踪机启动命令（外部终端，写入 README）

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4014 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_UXRCE_DDS_NS=px4_1 \
./build/px4_sitl_default/bin/px4 -i 0
```

- `4014` 已把 `PX4_SIM_MODEL` 置为 `x500_mono_cam_down`（`${PX4_SIM_MODEL#*gz_}` 前缀剥离后仍是该名），无需额外覆盖。
- 目标机启动命令不变：`PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,5,0,0,0,0" ... -i 1`。
- **带相机的追踪机必须先启动**（它负责拉起 world）；目标机为 standalone 实例，只负责 spawn。
- 注意 `PX4_GZ_MODELS` 由 `gz_env.sh` 提供，只在"拉起 world"的分支里被 source。启动前用 `echo $PX4_GZ_MODELS` 确认，否则模型 spawn 路径解析会落到根目录。
- 老工作流（无相机）继续可用：追踪机用 `PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500`，同时 launch 传 `enable_camera:=false`。

### 4.3 gz 话题（P1 实测核对）

预期（模型/链接/传感器未显式声明 `<topic>`，使用 scoped name）：

```text
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info
```

核对命令：

```bash
gz topic -l | grep -i imager
gz topic -e -t /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image -n 1
```

若实际话题名不同，只改 §5.2 的桥接配置（唯一改动点）。

## 5. ROS 2 侧桥接

### 5.1 依赖（使用者自行安装）

```bash
sudo apt install ros-jazzy-ros-gz-bridge      # 桥接：gz.msgs.Image ↔ sensor_msgs/msg/Image
sudo apt install ros-jazzy-vision-msgs        # YOLO 检测接口消息（yolo 模式需要）
# 已有：cv_bridge、image_transport、rqt_image_view
```

`package.xml` 增加 `exec_depend`：`ros_gz_bridge`、`sensor_msgs`、`geometry_msgs`、`vision_msgs`。

### 5.2 桥接配置 `config/camera_bridge.yaml`（需打进 `setup.py` 的 `data_files`）

`ros_gz_bridge` 的 `parameter_bridge` 支持 `-p config_file:=<yaml>`，配置文件是**顶层列表**，每项一个话题：

```yaml
# ros_gz_bridge 桥接配置：gz 相机 → ROS 2。
# 注意：本文件不是 ROS 参数文件，结构由 ros_gz_bridge 定义（顶层为列表）。
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
  frame_id: "camera_link_optical"
  lazy: true

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
  frame_id: "camera_link_optical"
  lazy: true
```

- 支持过的参数：`config_file`、`override_frame_id`、`override_timestamps_with_wall_time`、`expand_gz_topic_names`、`subscription_heartbeat`；配置项支持 `direction`、`qos_profile`、`frame_id`、`lazy`、`subscriber_queue`、`publisher_queue`。
- `lazy: true` 在无 ROS 订阅者时省掉 gz→ROS 拷贝；**不影响** gz 侧渲染（模型里 `<always_on>1</always_on>` 仍会渲染），渲染开销见 §11。
- 话题名中的 world/模型名在实现时由 launch 参数替换（生成临时参数或直接在模板中替换），默认 `default` / `x500_mono_cam_down_0`。

### 5.3 话题 / QoS / frame 约定

| 话题 | 类型 | 方向 | QoS | frame_id | 说明 |
| --- | --- | --- | --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | gz → ROS | SENSOR_DATA（best effort） | `camera_link_optical` | 预期编码 `rgb8`（gz 默认 RGB_INT8），P1 用 `ros2 topic echo --once --field encoding` 核对 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | gz → ROS | SENSOR_DATA | `camera_link_optical` | 内参权威来源，畸变系数为 0 |
| `/camera/detections` | `vision_msgs/msg/Detection2DArray` | YOLO → 我们 | SENSOR_DATA（或 reliable） | `camera_link_optical` | YOLO 输出，见 §7 |
| `/camera/detections_truth` | `vision_msgs/msg/Detection2DArray` | 我们（truth 模式） | reliable | `camera_link_optical` | 真值伪检测，与 YOLO 同类型、同字段，便于并行对比 |
| `/camera/image_annotated` | `sensor_msgs/msg/Image` | 我们 / YOLO | SENSOR_DATA | `camera_link_optical` | 可选调试图；truth 模式由适配节点发布，yolo 模式由 YOLO 模块发布（避免双发布者） |
| `/vision/target_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 我们 → 导引 | reliable | `enu` | 视觉量测，见 §7.5 |

- `camera_link_optical`：`camera_link` 按 REP-103 旋转到光学系（$z$ 光轴向前、$x$ 向右、$y$ 向下）。
- 订阅端统一用 `SensorDataQoS` 风格（best effort + KEEP_LAST），对 best effort / reliable 两种发布者都兼容。

### 5.4 静态 TF（可选，仅可视化）

launch 内用 `tf2_ros static_transform_publisher` 发布两条静态变换：

- `base_link → camera_link`：安装外参（标称 `xyz=(0,0,0)`、`rpy=(0,+90°,0)`）。
- `camera_link → camera_link_optical`：REP-103 光学系变换，标称四元数 $(x,y,z,w)=(0.707,-0.707,0,0)$。

两条均由标称安装推导，P2 标定核对。导引节点与视觉适配节点**不依赖 TF**：动态外参（世界系下的相机姿态）由适配节点用 PX4 odometry 自行合成，避免引入完整 TF 树。

## 6. 像素偏差 → 位置偏差映射

### 6.1 坐标系与符号约定

| 记号 | 含义 |
| --- | --- |
| $W$ | ENU 世界系：$x$ 东、$y$ 北、$z$ 上（与 `pythonsimulation2d` 一致，原点 = PX4 本地原点） |
| $B$ | 机体系：$x$ 前、$y$ 左、$z$ 上（由 PX4 NED 姿态经 `coordinates.py` 转换得到） |
| $C$ | 相机光学系（ROS 侧 `camera_link_optical`）：$z$ 为光轴、$x$ 向右、$y$ 向下 |
| $u,v$ | 像素坐标（整数或浮点），$u$ 向右增大、$v$ 向下增大 |
| $h_{cam}$ | 相机在 ENU 下的高度（由 odometry 实时给出，`pursuer_fixed_altitude` 仅作标称/后备） |
| $z_{target}$ | 目标所在水平面高度，默认 `target_base_altitude = 1.0` m |
| $h_{eff}$ | 有效高度 $h_{eff} = h_{cam} - z_{target}$，默认 8.0 − 1.0 = 7.0 m |

本安装下相机光轴沿机体 $-z$，因此 $C \to B$ 的固定旋转为

$$R_{B \leftarrow C} = \begin{bmatrix} 0 & -1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix},$$

即 $x_C = -y_B$、$y_C = -x_B$、$z_C = -z_B$。这是"光轴朝下 + REP-103 光学系"的直接结论，**必须由 P2 的标定实验确认符号**，代码中作为标称常量并在文档记录实测结论。

### 6.2 内参

$$f_x = f_y = f = \frac{W/2}{\tan(\theta_{\mathrm{hfov}}/2)}, \qquad c_x = \frac{W}{2}, \qquad c_y = \frac{H}{2}.$$

代入 $W=1280$、$H=960$、$\theta_{\mathrm{hfov}}=1.74$ rad：$f \approx 539.9$ px，$(c_x, c_y) = (640, 480)$。

- 内参**优先取 `/camera/camera_info`**（gz 发布值即权威值），上式为后备（`use_camera_info:=false` 或未收到 CameraInfo 时）。
- 畸变系数为 0，不做去畸变。

### 6.3 精确反投影（像素 → ENU，射线-平面求交）

设相机在 ENU 下的位置 $p_{cam,W} \in \mathbb{R}^3$、旋转 $R_{W \leftarrow C}$（由 odometry 与安装外参合成）。

1. 归一化视线（光学系）：

$$\tilde{d}_C = \big[\,(u-c_x)/f,\ (v-c_y)/f,\ 1\,\big]^\top .$$

2. 转到世界系并单位化：$d_W = \dfrac{R_{W \leftarrow C}\,\tilde{d}_C}{\lVert R_{W \leftarrow C}\,\tilde{d}_C \rVert}$。

3. 与水平面 $z = z_{target}$ 求交：

$$P_W = p_{cam,W} + \lambda d_W, \qquad \lambda = \frac{z_{target} - p_{cam,W,z}}{d_{W,z}} .$$

有效性判据：$d_{W,z} < -\varepsilon$（视线朝下，$\varepsilon$ 取小正数）且 $\lambda > 0$；否则返回 `None`（视野外/朝天/近乎平行）。

### 6.4 线性映射与雅可比

光轴铅垂（机体水平）且地面为水平面时，目标到相机的深度恒为 $h_{eff}$，§6.3 退化为**精确仿射映射**（不是近似）：

$$\begin{bmatrix} \Delta x_B \\ \Delta y_B \end{bmatrix} = \frac{h_{eff}}{f} \begin{bmatrix} -\Delta v \\ -\Delta u \end{bmatrix}, \qquad \Delta p_W = R_z(\psi) \begin{bmatrix} \Delta x_B \\ \Delta y_B \end{bmatrix},$$

$\psi$ 为机体航向（ENU yaw）。等价写法：

$$\Delta p_W = \frac{h_{eff}}{f}\, R_z(\psi) \begin{bmatrix} 0 & -1 \\ -1 & 0 \end{bmatrix} \begin{bmatrix} \Delta u \\ \Delta v \end{bmatrix}, \qquad J = \frac{\partial p_W}{\partial (u,v)} \in \mathbb{R}^{2\times 2}.$$

物理含义：

- **标量灵敏度** $m/p_x = h_{eff}/f$：1 像素对应多少米地面位移；与有效高度成正比、与焦距成反比。
- **方向**（由 $R_{B\leftarrow C}$ 直接推出，P2 标定确认）：图像上方对应机体前方，因此目标偏下（$+\Delta v$）→ 目标在机体**后方**（$\Delta x_B < 0$）；目标偏右（$+\Delta u$）→ 目标在机体**右侧**（$\Delta y_B < 0$，因 $y_B$ 指向左）。
- **噪声传播**：$\Sigma_p = J \,\mathrm{diag}(\sigma_u^2, \sigma_v^2)\, J^\top$，写入 §7.5 量测协方差；$J$ 在水平姿态下与像素位置无关。
- **有效性**：上述线性式只在水平姿态下精确。机体一旦倾斜 $\theta$，视线与地面的交点使深度随像素位置变化，映射变为单应（projective），线性式降级为一阶近似：视场边缘（半角 50°）、$\theta=5°$ 时局部尺度偏差可达 ~10%。因此**位置解算一律用 §6.3 的精确式**，线性式只用于灵敏度换算与协方差传播。

### 6.5 数值表（$h_{cam}=8$ m、$z_{target}=1$ m、$h_{eff}=7$ m、$f \approx 540$ px）

| 量 | 值 |
| --- | --- |
| 标量灵敏度 $h_{eff}/f$ | **12.96 mm/px ≈ 1.3 cm/px** |
| 10 px 偏差 | 0.130 m |
| 30 px 偏差（典型检测中心抖动上限） | 0.39 m |
| 2 px 检测噪声 | 0.026 m |
| 5 px 检测噪声 | 0.065 m |
| 全幅覆盖（宽 × 高） | 16.6 m × 12.4 m |
| 目标机 0.35 m 成像尺寸 | ≈ 27 px |

结论：图像中心 1 px ≈ 1.3 cm 地面位移；相对于 8 m 高度，检测框中心的像素精度直接决定量测精度，而**姿态（倾斜）带来的偏差远大于像素噪声**（见 §6.6）。

### 6.6 姿态与高度敏感度

| 误差源 | 量级 | 处理方式 |
| --- | --- | --- |
| 机体倾斜 $\theta$：足印中心水平偏移 $h_{eff}\tan\theta$ | 2° → 0.24 m（19 px）；5° → 0.61 m（47 px）；10° → 1.23 m（95 px） | 必须用 odometry 姿态做 §6.3 精确补偿；线性式不够 |
| 高度误差 $\Delta h$：灵敏度比例变化 $(h_{eff}+\Delta h)/h_{eff}$ | +0.5 m → 1.071 倍；+1.0 m → 1.143 倍 | 相机高度取 odometry 实测，不用标称值 |
| 目标高度偏差 | 0 m 与 1 m 相差 14% 的比例因子 | 用 `target_base_altitude` 参数化 |
| 姿态延迟（图像与 odometry 不同步） | 与角速率成正比 | 用"最近一次 odometry"做软同步并记录延迟，超时视为无效 |

### 6.7 标定与验证实验（P2 验收）

1. 追踪机按 §4.2 启动并悬停到 8 m（`scenario:=stationary`，或先不启动导引保持位置）。
2. `ros2 topic echo --once /camera/camera_info` 记录 $f, c_x, c_y$，与 §6.2 公式核对。
3. 用 Gazebo `/world/default/set_pose` 把目标机瞬移到已知偏移集合（相对相机正下方），偏移量在**机体系**下给出以保证与 yaw 无关：$(+x_B,2)$、$(-x_B,2)$、$(+y_B,2)$、$(-y_B,2)$ m（同时记录当前机体航向 $\psi$，或先把航向对齐到 0）。
4. 对每个位置：
   - 用 `camera_geometry.ground_to_pixel()` 预测 $(u,v)$；
   - 在 `/camera/image_annotated` 上核对预测点是否落在目标机上（truth 模式自动打点）；
   - 记录实测中心像素，填写符号表：目标 $+x_B$ → $\Delta v$ 的符号（预期为负，目标出现在图像上方）；目标 $+y_B$ → $\Delta u$ 的符号（预期为负，目标出现在图像左侧）。
5. 记录姿态补偿前后残差：人为让机体倾斜 5–10°（或对比飞行动态段），确认补偿后误差回到 5 cm 以内。
6. 把结论写入本文档表格与 `camera_geometry.py` 的标称常量注释。

### 6.8 模块 API：`src/pythonsimulation2d/camera_geometry.py`

纯 NumPy、无 ROS 依赖，供离线仿真、适配节点和测试共用。

```python
@dataclass(slots=True)
class CameraIntrinsics:
    width: int = 1280
    height: int = 960
    hfov_deg: float = 99.7          # 1.74 rad
    fx: float | None = None         # 给定则优先，否则由 hfov_deg 推导
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None

@dataclass(slots=True)
class CameraPose:
    """世界系（ENU）下的相机位姿。"""
    position_enu: np.ndarray        # (3,)
    rotation_world_from_optical: np.ndarray   # (3,3)，R_{W<-C}

class DownwardCameraModel:
    def __init__(self, intrinsics: CameraIntrinsics, mount_rotation_body_from_optical: np.ndarray | None = None) -> None: ...
    def intrinsics_matrix(self) -> np.ndarray: ...                     # 3x3 K
    def meters_per_pixel(self, height_above_target: float) -> float: ...  # h_eff / f
    def pixel_jacobian(self, camera_pose: CameraPose) -> np.ndarray: ...  # 2x2, m/px
    def ground_to_pixel(self, point_enu: np.ndarray, camera_pose: CameraPose) -> np.ndarray | None: ...
    def pixel_to_ground(self, pixel: np.ndarray, camera_pose: CameraPose) -> np.ndarray | None: ...
```

`CameraPose` 的构造由调用方完成：ROS 侧在 `vision_adapter` 里用 odometry + 安装外参合成，离线侧直接用仿真真值。

离线自检脚本 `tests/test_camera_geometry.py`（纯断言，无 pytest 依赖）：

```bash
cd 7_2Dsimulation
uv run python tests/test_camera_geometry.py
```

覆盖用例：

- 往返一致性：`pixel_to_ground(ground_to_pixel(P)) ≈ P`（误差 < 1e-6 m）。
- 灵敏度：$h_{eff}=7$ m 时 `meters_per_pixel() == 7/540`≈12.96 mm（±0.05 mm）。
- 正下方目标投影到 $(c_x, c_y)$。
- 5° 倾斜下足印中心不在 $(c_x,c_y)$，偏移约 47 px（验证姿态确实进入模型）。
- 目标高于相机、视线朝天、像素越界 → 返回 `None`。
- 水平姿态下线性映射与精确反投影一致（误差 < 1e-6 m）；倾斜 5° 时视场边缘处线性式与精确解偏差 > 5%（验证必须用精确式）。

### 6.9 `coordinates.py` 新增姿态转换

```python
def rotation_enu_from_quaternion_ned(quaternion_wxyz) -> np.ndarray:
    """PX4 VehicleOdometry 四元数 → ENU 世界系旋转矩阵 R_{W<-B}。"""
```

- 保留现有 `yaw_from_quaternion_ned()`（导引节点仍在使用），新函数只做边界转换，数学进入 `camera_geometry`。
- 实现要点：先把 NED→ENU 的轴置换写成固定矩阵，再组合四元数旋转，用现有 `ned_to_enu_vector()` 的一致性做单测。

## 7. YOLO 接口契约（独立模块）

本仓库**不实现检测**。以下契约是"最常见做法"（`vision_msgs/Detection2DArray` + bbox 中心点），作为 YOLO 模块与本仿真的对接标准。

### 7.1 输入契约（YOLO 订阅）

| 项 | 要求 |
| --- | --- |
| 图像话题 | `/camera/image_raw` |
| 图像类型/编码 | `sensor_msgs/msg/Image`，`encoding = rgb8`（**注意不是 bgr8**，BGR 网络需自行转换） |
| 分辨率/帧率 | 1280×960 @ 30 Hz（允许丢帧） |
| 内参 | `/camera/camera_info`，`sensor_msgs/msg/CameraInfo`，畸变系数 0 |
| QoS | best effort, KEEP_LAST(depth ≥ 5)，durability volatile |
| `header.stamp` | 图像采集时刻（gz sim time），必须原样回填到检测结果 |
| `header.frame_id` | `camera_link_optical`（原样回填） |
| 坐标系 | 光学系：$x$ 右、$y$ 下、$z$ 光轴向前 |

### 7.2 输出契约（YOLO 发布）

| 项 | 要求 |
| --- | --- |
| 话题 | `/camera/detections` |
| 类型 | `vision_msgs/msg/Detection2DArray` |
| QoS | best effort 或 reliable 均可（订阅端 best effort，兼容两者） |
| 频率 | 与图像同频（≤30 Hz） |
| `header` | 与对应图像一致（stamp 与 frame_id 原样复制） |
| `detections[i].bbox.center.x/y` | 目标中心像素坐标（px，float） |
| `detections[i].bbox.size_x/size_y` | 检测框宽/高（px，>0） |
| `detections[i].results[0].hypothesis.class_id` | 固定为 `"drone"` |
| `detections[i].results[0].hypothesis.score` | 置信度 ∈ [0, 1] |
| `detections[i].results[0].pose` | 可留默认值（位置估计由本仓库反投影计算） |
| `detections[i].id` | 可留空字符串 |
| 多目标 | 按 score 降序排列；适配层取 score 最高的 `"drone"` |
| 未检出 | 允许发空数组（推荐，便于消费端看到该帧时间戳），也允许不发消息 |

字段结构（`vision_msgs` 4.1，Jazzy）：

```text
Detection2DArray
└── std_msgs/Header header
└── Detection2D[] detections
    ├── std_msgs/Header header
    ├── ObjectHypothesisWithPose[] results
    │   ├── ObjectHypothesis hypothesis { string class_id; float64 score; }
    │   └── geometry_msgs/PoseWithCovariance pose
    ├── BoundingBox2D bbox { Pose2D center {float64 x; float64 y; float64 theta;}; float64 size_x; float64 size_y; }
    └── string id
```

### 7.3 时序语义

- `header.stamp` 是唯一权威时间源；适配层用**到达时刻 + 最近一次 odometry** 合成相机位姿（图像时间戳属于 gz 时钟，与 PX4 odometry 时钟不可直接比较）。
- 允许 YOLO 处理延迟（单帧算法延迟不设上限，但由适配层按 `measurement_timeout_s` 判陈旧）。
- 丢失检测的处理由消费端负责：不发布 ≠ 目标不存在，适配层仅停止发布量测（不发布"丢失"事件）。

### 7.4 适配节点 `gazebosimulation2d/vision_adapter.py`

入口：`vision_adapter`（`setup.py` 增加 console_script），节点名 `vision_adapter`。

| 职责 | 说明 |
| --- | --- |
| 订阅 | `pursuer_namespace/fmu/out/vehicle_odometry`（PX4 QoS：BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST(1)）；`/camera/camera_info`；`source:=yolo` 时 `/camera/detections`，`source:=truth` 时订阅自身生成的伪检测（内部直通） |
| 相机位姿合成 | odometry 位置/姿态 → ENU + 安装外参 → `CameraPose`（姿态转换走 `coordinates.rotation_enu_from_quaternion_ned`） |
| 反投影 | 调 `camera_geometry.pixel_to_ground()` 得目标 XY；`z` 取 `target_base_altitude` |
| 协方差 | $\Sigma_p = J\,\mathrm{diag}(\sigma_{px}^2,\sigma_{px}^2)\,J^\top$，$\sigma_{px}$ 由参数给出 |
| 发布 | `/vision/target_pose`（见 §7.5）；`truth` 模式额外发布 `/camera/detections_truth` 与 `/camera/image_annotated` |
| 记录 | 退出时写 `outputs/gazebo2d_vision/<scenario>/<algorithm>/vision_samples.csv` |
| 日志 | `debug_log` / `debug_log_period_s`，沿用仓库现有风格 |

`truth` 模式的意义：用 odometry 真值经 `ground_to_pixel()` 造**同类型同字段**的伪检测，使"桥接 → 检测 → 反投影 → 量测"整条链路在 YOLO 就绪前即可验收；YOLO 上线后只需把 `source` 改为 `yolo`，并把两个模块并行运行对比。

### 7.5 量测契约（适配层 → 导引）

| 项 | 值 |
| --- | --- |
| 话题 | `/vision/target_pose` |
| 类型 | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| `header.stamp` | 对应检测/图像的时间戳（gz 时钟） |
| `header.frame_id` | `enu` |
| `pose.pose.position` | 目标 ENU 位置，`z = target_base_altitude` |
| `pose.pose.orientation` | 不可用，保持单位四元数 |
| `pose.covariance` | 仅填 XY 块（索引 0、1、7）：$\Sigma_p$（m²），其余为 0 |
| 发布条件 | 检测有效（score ≥ `detection_score_min`、反投影有效、odometry 未超时） |
| 无效语义 | 不发布；消费端以 `stamp` + `measurement_timeout_s` 判定陈旧 |

### 7.6 YOLO 模块对接清单

1. 订阅 `/camera/image_raw`（`rgb8`）与 `/camera/camera_info`，使用 best-effort QoS。
2. 输出 `/camera/detections`（`vision_msgs/Detection2DArray`），`class_id="drone"`、bbox 中心为像素坐标。
3. 原样回填 `header.stamp` 与 `header.frame_id`。
4. 未检出时发空数组（或保持静默），不要用异常退出。
5. （可选）发布 `/camera/image_annotated` 便于可视化；不要与本仓库的 truth 模式同时运行在同一话题上。
6. 按 §7.1 的编码约定处理 `rgb8 → bgr8` 转换。

## 8. 参数与 launch 设计

### 8.1 launch 参数（`launch/guidance.launch.py`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `enable_camera` | `true` | 是否启动相机桥接节点；老工作流（`gz_x500`）传 `false` |
| `camera_topic_prefix` | `/camera` | 相机话题前缀 |
| `pursuer_gazebo_world` | `default` | 拼 gz 话题用 |
| `pursuer_gazebo_model` | `x500_mono_cam_down_0` | 拼 gz 话题用 |
| `vision_source` | `off` | `off`（不启动适配节点）/ `truth` / `yolo` |
| `vision_record_data` | `true` | 是否保存视觉 CSV |
| `vision_record_output_dir` | `outputs/gazebo2d_vision` | CSV 根目录 |

现有参数（`algorithm`、`scenario`、`pursuer_namespace`、`target_base_altitude`、`pursuer_fixed_altitude` 等）保持不变，并与新参数一起透传给 `vision_adapter`。

### 8.2 `config/default.yaml` 新增块

```yaml
vision_adapter:
  ros__parameters:
    source: truth                    # off | truth | yolo
    pursuer_namespace: /px4_1
    camera_topic_prefix: /camera
    camera_frame_id: camera_link_optical
    target_base_altitude: 1.0
    pursuer_fixed_altitude: 8.0
    image_width: 1280
    image_height: 960
    camera_hfov_deg: 99.7
    use_camera_info: true
    camera_mount_xyz: [0.0, 0.0, 0.0]
    camera_mount_rpy_deg: [0.0, 90.0, 0.0]
    detection_score_min: 0.5
    pixel_noise_px: 3.0
    measurement_timeout_s: 0.3
    pose_timeout_s: 0.2
    publish_annotated_image: true
    record_data: true
    record_output_dir: outputs/gazebo2d_vision
    debug_log: false
    debug_log_period_s: 0.5
```

### 8.3 其它文件改动

| 文件 | 改动 |
| --- | --- |
| `src/gazebosimulation2d/setup.py` | `data_files` 增加 `config/camera_bridge.yaml`；`console_scripts` 增加 `vision_adapter` |
| `src/gazebosimulation2d/package.xml` | 增加 `ros_gz_bridge`、`sensor_msgs`、`geometry_msgs`、`vision_msgs` 的 `exec_depend` |
| `7_2Dsimulation/README.md` | 新增"相机与视觉链路"章节：PX4 启动命令、依赖安装、话题表、参数表、验证命令、YOLO 对接说明 |
| `docs/2d_simulation_guidance_overview.md` | 修正"不引入相机"的表述：相机用于视觉量测链路，导引默认仍用真值（P4 起可选视觉闭环）；新增本计划与几何公式的引用 |
| `docs/camera_vision_integration_plan.md` | 本文件 |

### 8.4 视觉 CSV 字段（`vision_samples.csv`）

```text
time, valid, u_det, v_det, u_truth, v_truth, pixel_error_px,
target_x_est, target_y_est, target_x_truth, target_y_truth, position_error_m,
pursuer_x, pursuer_y, pursuer_z, roll_deg, pitch_deg, yaw_deg,
detection_latency_ms
```

- `time` 采用与 `guidance_node` 相同的"节点启动后经过时间"口径。
- `truth` 模式下 `target_*_truth` 直接来自 odometry，用于验证反投影误差；`yolo` 模式下作为评估基准（YOLO 模块的精度指标由此得出）。

## 9. 实施阶段

### P1 相机 + 桥接 + 可订阅（不改控制回路）

| 任务 | 文件 |
| --- | --- |
| 更新追踪机启动命令与说明，补 `PX4_GZ_MODELS` 检查 | `README.md` |
| 新增桥接配置文件与条件节点，新增 launch 参数 | `config/camera_bridge.yaml`、`launch/guidance.launch.py`、`setup.py`、`package.xml` |
| 可选：静态 TF | `launch/guidance.launch.py` |

验收：

```bash
gz topic -l | grep -i imager                      # 两条 gz 话题存在
ros2 topic list | grep /camera                    # /camera/image_raw、/camera/camera_info
ros2 topic hz /camera/image_raw                   # ≈30 Hz
ros2 topic echo --once /camera/camera_info        # f≈540、cx=640、cy=480、width=1280
ros2 topic echo --once --field encoding /camera/image_raw   # rgb8
ros2 run rqt_image_view rqt_image_view /camera/image_raw    # 看到俯视地面画面
ros2 launch gazebosimulation2d guidance.launch.py enable_camera:=false   # 老工作流仍正常
```

### P2 几何模块 + 标定

| 任务 | 文件 |
| --- | --- |
| 新增下视相机几何模块与自检脚本 | `src/pythonsimulation2d/camera_geometry.py`、`tests/test_camera_geometry.py` |
| 新增 ENU 姿态转换 | `src/gazebosimulation2d/gazebosimulation2d/coordinates.py` |
| 执行 §6.7 标定实验，把符号表与结论写入文档 | 本文件、`docs/2d_simulation_guidance_overview.md` |

验收：`uv run python tests/test_camera_geometry.py` 全通过；标定实验中预测像素与实测像素一致（偏差 < 5 px），姿态补偿后位置残差 < 5 cm。

### P3 视觉适配层 + 接口契约落地

| 任务 | 文件 |
| --- | --- |
| 新增适配节点（truth/yolo 双源、协方差、CSV、标注图） | `src/gazebosimulation2d/gazebosimulation2d/vision_adapter.py` |
| 参数块、launch 条件节点、参数表 | `config/default.yaml`、`launch/guidance.launch.py`、`README.md` |
| 契约文档（§7 内容进 README/docs） | `README.md`、`docs/2d_simulation_guidance_overview.md` |

验收：

```bash
ros2 launch gazebosimulation2d guidance.launch.py enable_camera:=true vision_source:=truth
ros2 topic hz /vision/target_pose                 # 与图像同频
ros2 topic echo --once /vision/target_pose        # frame_id=enu，位置与真值一致，协方差非零
# 退出后：
uv run plot_gazebo_csv.py ...                     # 现有闭环链路不受影响
# 检查 outputs/gazebo2d_vision/<scenario>/<algorithm>/vision_samples.csv：
# truth 模式下 pixel_error_px ≈ 0（<5 px），position_error_m < 0.1 m
```

### P4（后续，本轮仅预留）

- `guidance_node` 增加 `target_source:=odometry|vision`，视觉量测接入导引；丢帧/超时降级策略；视觉导引算法族与指标（含"像素误差—追踪性能"敏感性分析）。
- 可选性能优化：自定义相机模型（640×480@15 Hz、`<always_on>0</always_on>`、显式 `<topic>`），减少 WSL2 渲染负载。

## 10. 风险与缓解

| 风险 | 影响 | 缓解 |
| --- | --- | --- |
| WSL2 无 GPU 加速时相机渲染拖慢 RTF | 闭环时序失真 | 先测 RTF；必要时降分辨率/帧率（自定义模型），或仅在做视觉实验时启用相机 |
| gz 相机话题名与预期不符 | 桥接收不到数据 | `gz topic -l` 实测；话题名只在 `camera_bridge.yaml` 一处维护 |
| `PX4_GZ_MODELS` 未生效（standalone 实例不 source `gz_env.sh`） | 模型 spawn 失败 | 启动前 `echo $PX4_GZ_MODELS`；追踪机先启动 |
| 图像 `frame_id` 缺失或语义错误 | 反投影符号错 | 桥接显式设 `frame_id: camera_link_optical`；P2 标定确认符号并记录 |
| gz 与 PX4 时钟不同源 | 姿态与图像错配 | 只用"到达时刻 + 最近 odometry"软同步；`pose_timeout_s` 超时判无效 |
| 追踪机 8 m 高、100° 视场下目标机仅 ~27 px | 检测精度受限 | 契约中给出像素噪声→位置噪声换算；如需更高精度，后续缩小 FOV 或提高分辨率 |
| `vision_msgs` / `ros_gz_bridge` 未安装 | yolo 模式或桥接不可用 | `truth` 模式与桥接解耦；`vision_msgs` 在 yolo 模式延迟导入，缺失时给出明确报错 |

## 11. 后续扩展预留

- **视觉闭环**：量测话题（§7.5）、超时语义、CSV 评估字段均已定义，导引侧只需增加一个订阅与降级分支。
- **FOV/遮挡**：`camera_geometry.ground_to_pixel()` 已返回视野外 `None`，天然支持"目标离开视野"判定；后续可在此基础上补 FOV 约束指标。
- **多机/多相机**：`camera_topic_prefix`、`pursuer_gazebo_model` 已参数化，扩展第二台相机只需增加一份桥接配置项与一个适配实例。
