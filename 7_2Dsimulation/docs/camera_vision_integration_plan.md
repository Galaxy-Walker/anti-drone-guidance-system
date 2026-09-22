# 7_2Dsimulation 下视相机与视觉接口接入计划

> 状态：待实施。本文件是修订后的实施基线，不代表代码或 Gazebo 验证已完成。
> 目标环境：ROS 2 Jazzy + PX4 v1.16 SITL + Gazebo Harmonic（gz-sim 8）。模型、消息和桥接能力以本机安装版本核验结果为准。

## 1. 范围与决策

本轮只完成三件事：追踪机下视相机可订阅、纯 NumPy 投影几何可测试、ROS 伪检测可转换成独立位置量测。**不修改现有导引闭环，不实现 YOLO，不让视觉量测控制飞机。**

| 决策 | 本轮约定 |
| --- | --- |
| 相机模型 | 优先使用 PX4 自带 `x500_mono_cam_down`；只给追踪机安装，不修改 PX4 安装目录或 SDF |
| 桥接 | `ros_gz_bridge`；先用固定 YAML，不同时维护模板、临时文件和动态桥接参数三套方案 |
| 话题 | 固定 `/camera`、`/vision` 前缀；多相机和任意前缀参数化后移 |
| 几何 | 集中在 `pythonsimulation2d/camera_geometry.py`，ROS 节点不复制公式 |
| 位姿 | 必须合成安装平移、安装旋转和完整机体姿态，不能只使用 yaw |
| 时间 | 本轮 truth 使用单调接收时间进行近邻匹配；明确是近似，不宣称采样同步 |
| 内参 | ROS 运行必须等到有效 `CameraInfo`；FOV 推导值只用于离线测试和数量级核对 |
| 默认行为 | `enable_camera:=false`、`vision_source:=off`，保持现有启动方式不变 |
| 后移内容 | 静态 TF、标注图、YOLO 运行模式、多目标跟踪、遮挡、视觉闭环和降级策略 |

QGC、PX4 SITL、Gazebo 和 Micro XRCE-DDS Agent 仍由使用者在外部终端启动，仓库 launch 不拉起这些进程。改变机型或增加渲染负载可能影响闭环表现，因此“不改导引代码”不等于“实验结果必然不变”。

## 2. 最小数据流

```text
追踪机相机 → ros_gz_bridge → /camera/image_raw（人工看图）
                         → /camera/camera_info ─────────────┐
追踪机 VehicleOdometry → 坐标转换、安装外参 → CameraPose ──┤
目标机 VehicleOdometry → 坐标对齐 → 参考目标位置 ───────────┤
                                                          ▼
                                            vision_adapter（truth）
                                            ground_to_pixel()
                                                 │
                                       /camera/detections_truth
                                                 │ 同一处理函数直通，不自订阅
                                            pixel_to_ground()
                                                 ▼
                                       /vision/target_pose + CSV
```

truth 不订阅图像、不解码图像、不生成标注图；等到 `CameraInfo` 后以独立定时器生成伪检测。它验证几何往返和消息封装，**不验证渲染、目标识别、图像同步或 YOLO 精度**。图像桥接另行验收。

`VehicleOdometry` 是 PX4 估计输出，并非 Gazebo 世界真值。`truth` 是模式名称；日志和误差表中称其为“odometry 参考”。真实投影验证需要独立的图像观测或 Gazebo 世界位姿。

## 3. P1：相机桥接与环境核验

### 3.1 模型与启动

候选模型和启动方式如下，**先核对本机文件，再运行命令**：

```bash
rg -n 'x500_mono_cam_down|PX4_SIM_MODEL' ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4014_gz_x500_mono_cam_down
rg -n 'pose|camera_link|imager|horizontal_fov|width|height|update_rate|topic' ~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf
```

预期模型为 `x500_mono_cam_down`，airframe 为 `4014`，相机 link/sensor 为 `camera_link/imager`，1280×960、30 Hz、水平 FOV 1.74 rad。安装 pose 预期包含 `0 0 .10 0 1.5707 0`；还需检查 include、父 link 和 sensor pose，不能把单个 pose 当成最终外参。

核对 airframe 确实选中该模型后，在无旧模型环境变量覆盖的终端中启动追踪机：

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4014 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_UXRCE_DDS_NS=px4_1 \
./build/px4_sitl_default/bin/px4 -i 0
```

沿用现有双机启动流程：追踪机先启动 world，目标机以 standalone 模式启动，继续使用 `4001/gz_x500`、`-i 1` 和 `/px4_2`。记录最终模型实例名，不假设一定是 `x500_mono_cam_down_0`。模型路径错误时检查 PX4 启动脚本如何加载 `gz_env.sh`；父终端的 `echo $PX4_GZ_MODELS` 不能证明子进程内是否正确加载。

### 3.2 依赖与桥接

由使用者自行安装：

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-vision-msgs ros-jazzy-rqt-image-view
ros2 pkg xml ros_gz_bridge
ros2 interface show vision_msgs/msg/BoundingBox2D
ros2 interface show vision_msgs/msg/Pose2D
gz topic -l | grep -E 'imager|camera_info'
```

在 `config/camera_bridge.yaml` 保存两条实测 gz 话题。以下只表示预期结构，实施时核对 `qos_profile`、`frame_id` 的本机版本支持；不把最新分支的功能直接视为 Jazzy 已安装功能。

```yaml
- ros_topic_name: /camera/image_raw
  gz_topic_name: /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
  lazy: true
- ros_topic_name: /camera/camera_info
  gz_topic_name: /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
  qos_profile: SENSOR_DATA
  lazy: true
```

该文件是桥接专用顶层列表，不是 ROS 节点参数文件；通过 `config_file` 参数加载。节点级 `override_frame_id` 或逐桥接 `frame_id` 只选择本机验证支持的一种，目标名称为 `camera_link_optical`。若不能覆盖，则记录桥接实际 frame，并让适配节点验证这个已知 frame；不能默默把未知 frame 当成光学系。修改名称不会旋转数据或修复错误的外参。

本轮固定 world/模型路径，实际不同只修改 YAML。未来确实需要动态名称时，再选本机支持的 Python launch 桥接参数接口；不引入 YAML 字符串模板。`lazy` 只减少无订阅者时的桥接工作，不停止 Gazebo 相机渲染。

### 3.3 验收

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 topic echo --once --field encoding /camera/image_raw
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

记录桥接版本、实际话题、frame、编码、K/D、分辨率和 Gazebo RTF。30 Hz 是仿真时间频率，RTF 不足 1 时壁钟观测频率可能不足 30 Hz。验证画面确实朝下；本轮不发布 TF。

## 4. P2：纯几何模块

### 4.1 坐标与外参

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

此矩阵是**完整安装链的结果**，不能再次作为 link→optical 旋转叠加。预期安装平移为 `[0, 0, 0.10]` m，而非零；最终值由 P1 的 SDF 变换链决定。

两机各自的 PX4 本地原点不保证与 Gazebo world 或彼此相同。实施前在多个已知位置检查对齐；若不一致，应在 ROS 边界显式转换到公共世界系，并记录各自原点偏移/旋转。只交换 NED/ENU 轴不会消除原点偏差。

### 4.2 内参与射线求交

使用 $K$ 中独立的 $f_x,f_y,c_x,c_y$。ROS 路径检查内参为有限数、焦距为正、尺寸有效，且畸变为零；不支持的畸变明确报错，不能静默忽略。离线理想方形像素模型可使用：

$$f_x=f_y=\frac{W/2}{\tan(\theta_{hfov}/2)}.$$

1280×960、1.74 rad 对应约 539.94 px。主点以 `CameraInfo` 为准，不强制要求正好等于 `(640, 480)`。

令 $z_t$ 为公共 ENU 下的目标水平面高度：

$$r_C=[(u-c_x)/f_x,\ (v-c_y)/f_y,\ 1]^\top,\qquad r_W=R_{W\leftarrow C}r_C,$$
$$s=\frac{z_t-p_{cam,W,z}}{r_{W,z}},\qquad P_W=p_{cam,W}+s r_W.$$

无需单位化射线。要求相机高于平面、像素在图像内、射线朝下且远离平行、交点位于前方，所有值有限；否则返回 `None`。正投影还需检查光学深度为正及像素是否在图像范围内。近乎平行的判定使用归一化方向的 z 分量，避免阈值依赖射线长度。

`target_base_altitude` 是目标控制高度，不保证等于实际高度或 bbox 中心所代表物理点的高度。它只作为固定平面假设，不能据此承诺真实图像位置误差小于 5 cm。truth 正投影输入使用参考 XY 和同一个 $z_t$，同时记录 odometry 实际 z 与平面的差值。

### 4.3 线性灵敏度与局部雅可比

相机光轴铅垂时，令 $h=p_{cam,W,z}-z_t$：

$$\Delta p_W=R_z(\psi)\begin{bmatrix}0&-h/f_y\\-h/f_x&0\end{bmatrix}\begin{bmatrix}u-c_x\\v-c_y\end{bmatrix}.$$

这是水平姿态下的精确仿射映射；倾斜时不用它解算位置或传播协方差。设 $a=\partial r_W/\partial u=R_{W\leftarrow C}[:,0]/f_x$，$b=\partial r_W/\partial v=R_{W\leftarrow C}[:,1]/f_y$：

$$J=\frac{z_t-p_{cam,W,z}}{r_{W,z}}\left[\ a_{xy}-\frac{r_{W,xy}}{r_{W,z}}a_z,\quad b_{xy}-\frac{r_{W,xy}}{r_{W,z}}b_z\ \right].$$

因此 $J$ 依赖**像素、姿态和目标平面**。像素噪声传播为 $\Sigma_{xy}=J\operatorname{diag}(\sigma_u^2,\sigma_v^2)J^\top$；本轮只计像素噪声，不把它宣称为包含姿态、安装、时间和高度误差的总不确定度。

以相机高度 8 m、目标平面 1 m、焦距 540 px 为示例：1 px≈12.96 mm，5 px≈6.5 cm；水平覆盖约 16.6×12.4 m；0.35 m 宽物体约 27 px。这里的 8 m 是**相机高度**，不是加上安装偏移前的机体高度。5° 倾斜的光轴足印偏移约 0.61 m，不是整幅图像的统一平移。

### 4.4 API 与测试

`camera_geometry.py` 只接受已构造的相机位姿，不重复持有安装旋转；使用 `dataclasses(slots=True)` 和类型注解。

```python
@dataclass(slots=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

@dataclass(slots=True)
class CameraPose:
    position_enu: np.ndarray
    rotation_world_from_optical: np.ndarray

def ground_to_pixel(point_enu: np.ndarray, pose: CameraPose,
                    intrinsics: CameraIntrinsics) -> np.ndarray | None: ...

def pixel_to_ground(pixel: np.ndarray, pose: CameraPose,
                    intrinsics: CameraIntrinsics,
                    target_plane_z: float) -> tuple[np.ndarray, np.ndarray] | None:
    """返回三维交点和对像素的 2×2 XY 雅可比。"""
    ...
```

ROS 坐标边界在 `coordinates.py` 新增完整姿态转换函数，保留现有 yaw API。在 `tests/test_camera_geometry.py` 使用标准库 `unittest`，不引入 pytest 依赖；保留现有相对路径导入机制。

```bash
cd 7_2Dsimulation
uv run python tests/test_camera_geometry.py
```

覆盖：水平往返误差 < 1e-6 m、非零平移、yaw/roll/pitch、不同 `fx/fy` 和偏置主点、非零目标平面高度、射线朝天/近平行、越界与非有限输入、非法四元数和 frame。雅可比与中心有限差分比较；完整姿态转换检查正交性、行列式及 FLU 三个基向量，不能只比 yaw。

## 5. P3：ROS truth 适配器

### 5.1 订阅、发布与依赖

新增 `gazebosimulation2d/vision_adapter.py`，节点名和入口均为 `vision_adapter`。

| 接口 | 约定 |
| --- | --- |
| 两机 odometry | `pursuer_namespace`、`target_namespace` 下的 `fmu/out/vehicle_odometry`；PX4 QoS：BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST(1) |
| `/camera/camera_info` | BEST_EFFORT + VOLATILE；缓存并验证内参和 frame |
| `/camera/detections_truth` | `vision_msgs/Detection2DArray`，BEST_EFFORT + VOLATILE；`class_id="drone"`、score=1；小型正尺寸 bbox 仅是占位，不冒充真实目标框 |
| `/vision/target_pose` | `geometry_msgs/PoseWithCovarianceStamped`，RELIABLE + VOLATILE；frame=`enu`，原点按 §4.1 约定 |

truth 同样依赖 `vision_msgs`，不能仅在 yolo 模式导入。包清单增加 `ros_gz_bridge`、`sensor_msgs`、`geometry_msgs`、`vision_msgs`；不新增 `cv_bridge`、`image_transport`、`tf2_ros`。

### 5.2 本轮时间语义

本轮 `vision_adapter` 使用 `use_sim_time=false`，不修改现有导引节点的时钟。保存两机消息的单调接收时间，在有界缓存内近邻配对：两条消息均未超过 `pose_timeout_s`，且接收时间差不超过 `pose_pair_tolerance_s` 才有效。

伪检测及量测共用**本次生成时刻的 ROS 系统时间戳**，不是图像采集时间，也不是 CameraInfo 时间戳。PX4 的 `timestamp`/`timestamp_sample` 可记录诊断，但未完成时钟映射前不与 ROS/Gazebo 时间相减。超时和 CSV 经过时间使用单调时钟，不受系统时间跳变影响。

接收时间匹配不能消除传输和估计延迟，也不能证明两机采样同步。重复使用的 odometry 样本须在 CSV 中可识别；不把定时器频率视为新量测频率。缺内参、姿态无效、样本过旧、配对超差或投影无效时不发布位置，记录失败原因。

### 5.3 协方差与记录

位置为反投影交点，姿态填单位四元数但明确“不提供姿态观测”。6×6 行优先协方差中 XY 块索引必须填 **0、1、6、7**，保持对称；z 方差用配置的平面高度不确定度，三个姿态对角线填明确的大有限方差，不能以全零表示未知。

XY 方差本轮只是像素条件协方差；消费者不得当成完整融合协方差。未实现姿态、高度、位姿及时间误差传播前，不用于滤波置信度验收。

CSV 固定写入 `vision_record_output_dir/vision_samples.csv`，不按算法/场景分目录，实验使用不同输出目录。字段最小集合：

```text
elapsed_s, stamp_s, valid, invalid_reason,
pursuer_timestamp_sample_us, target_timestamp_sample_us,
pursuer_age_ms, target_age_ms, pose_pair_delta_ms,
u_ref, v_ref, target_x_est, target_y_est, target_x_ref, target_y_ref,
target_z_odom, target_plane_z, position_roundtrip_error_m
```

`elapsed_s` 是适配节点单调经过时间；现有导引 CSV 从追踪开始计时，不能直接按同名 time 列对齐。truth 没有检测延迟或独立像素误差，不输出具有误导性的 `detection_latency_ms`、`pixel_error_px`。现有 `plot_gazebo_csv.py` 不用于视觉 CSV。

### 5.4 参数与文件变更

launch 与节点使用相同参数名；YAML 默认值同步维护。不向适配器透传导引算法、控制频率、解锁或 setpoint 参数。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `enable_camera` | false | 仅 launch 使用，控制桥接，不控制相机渲染 |
| `vision_source` | off | 本轮只支持 off/truth，其他值报错 |
| `pursuer_namespace` / `target_namespace` | /px4_1 / /px4_2 | 两机输入 |
| `camera_frame_id` | camera_link_optical | P1 核验后填写 |
| `target_base_altitude` | 1.0 | 映射到公共 ENU 的目标平面；原点不同时不能直接复用数值 |
| `camera_mount_xyz` | [0.0, 0.0, 0.10] | 标称值，P1 核验完整平移 |
| `camera_mount_rpy_deg` | [0.0, 90.0, 0.0] | 标称 link 安装旋转，不含光学轴转换 |
| `truth_rate_hz` | 10.0 | 独立定时器，不要求与图像同频 |
| `pose_timeout_s` / `pose_pair_tolerance_s` | 0.2 / 0.05 | 接收时间新鲜度和两机配对容差 |
| `pixel_noise_px` | 3.0 | 协方差假设，不代表实际添加随机噪声 |
| `target_plane_sigma_m` | 0.1 | 输出 z 不确定度假设 |
| `vision_record_data` | true | 保存 CSV |
| `vision_record_output_dir` | outputs/gazebo2d_vision | 实验可覆盖 |
| `debug_log` / `debug_log_period_s` | false / 0.5 | 日志开关与周期 |

`vision_source=truth` 不强制启用仓库桥接，允许外部桥接提供 CameraInfo；缺数据时等待并提示。参数正值、数组长度、有限数、旋转有效性及支持的 source 都需检查。

| 文件 | 实施时改动 |
| --- | --- |
| `src/pythonsimulation2d/camera_geometry.py` | 纯几何与雅可比 |
| `src/gazebosimulation2d/gazebosimulation2d/coordinates.py` | 完整姿态转换 |
| `tests/test_camera_geometry.py` | 离线几何和坐标测试 |
| `src/gazebosimulation2d/gazebosimulation2d/vision_adapter.py` | truth、共享检测处理、CSV |
| `src/gazebosimulation2d/test/test_vision_adapter.py` | ROS 环境下的消息、超时和参数测试 |
| `src/gazebosimulation2d/config/camera_bridge.yaml` | 实测固定桥接配置 |
| `src/gazebosimulation2d/config/default.yaml` | 新增 vision_adapter 块 |
| `src/gazebosimulation2d/launch/guidance.launch.py` | 条件节点与上述参数，不改变原导引参数 |
| `src/gazebosimulation2d/setup.py`、`package.xml` | 安装配置、入口与依赖 |
| `README.md`、`docs/2d_simulation_guidance_overview.md` | 实施后同步命令和范围，不提前宣称功能可用 |

### 5.5 验收边界

1. 无相机依赖环境下，默认 launch 仍能启动原导引节点；条件节点关闭时不解析可选包资源。
2. `enable_camera:=true vision_source:=truth` 时，收到有效内参及两机数据后输出伪检测和位置；缺数据、NaN、非法 frame、超时、越界时拒绝量测并记录原因。
3. 覆盖实际消息字段、协方差对称性、不同 source 校验、时间跳变和陈旧样本测试。
4. truth 同一几何模型往返误差 < 1e-6 m；这只是内部一致性，不作为真实视觉误差指标。
5. 在 P1～P3 后单独做真实图像外参验证：使用已知高度的静态标记和独立测得的像素，在中心、四个方向及倾斜姿态下比较预测与观测。通过受控悬停完成，不在运行中的 Offboard 目标机上直接瞬移制造估计器跳变。
6. 真实图像残差分别报告像素误差、XY 误差及同步/高度条件；不能用同一模型生成的像素作为独立实测。没有独立观测时，该项明确标记“未验证”。

## 6. 后续 YOLO 接口与进入条件

只保留最小契约，不在本轮实现 `vision_source=yolo`：

| 项目 | 契约 |
| --- | --- |
| 输入 | `/camera/image_raw` + `/camera/camera_info`，按实际 encoding 处理，不盲目假设 RGB/BGR |
| 输出 | `/camera/detections`，`vision_msgs/Detection2DArray`，BEST_EFFORT 或 RELIABLE 均可；订阅者 BEST_EFFORT |
| Header | 数组和检测元素复制源图像 stamp/frame；未检出推荐空数组 |
| 中心 | `bbox.center.position.x/y`，坐标对应原始图像；网络 resize/letterbox 必须反变换 |
| 类别 | `results[].hypothesis.class_id="drone"`，score 在 [0,1]；单目标阶段按有效 drone 的最大分数选择 |
| bbox | `size_x/size_y` 为正；不使用默认 results.pose 作为三维位置 |

字段依据 [vision_msgs 4.1 Pose2D](https://github.com/ros-perception/vision_msgs/blob/4.1.0/vision_msgs/msg/Pose2D.msg)，本机仍用 `ros2 interface show` 核验。独立模块的检测契约和 truth 相同，时间语义不同：truth 是生成时刻，YOLO 是原图采集时刻，不能直接切换后沿用 truth 的接收时间近似。

接入 YOLO 前必须完成：

- 确定 Gazebo、ROS、PX4 的采样时间映射，建立按原图采集时刻索引的相机位姿缓存；必要时插值，拒绝超出缓存或时钟映射不可信的量测。
- 可桥接 `/clock` 并仅让相关视觉节点使用 `use_sim_time=true`；**这不会自动同步 PX4 采样时间**，也不应顺手改变导引节点时钟。另用单调时钟监督数据流停滞和仿真暂停。
- 在同一时间基准计算图像年龄、推理延迟和姿态匹配误差，支持乱序、重复帧、时间回跳和超时拒绝。不能用最新姿态处理任意延迟的旧图像。
- 确定 bbox 中心对应的物理点及目标高度误差，完成独立图像标定和动态误差评估。

视觉闭环另立计划：状态估计、速度获得方式、目标选择稳定性、FOV/丢帧/超时策略及回归验收均未在本轮解决，不能描述为“仅增加一个订阅即可”。标注图、自定义低分辨率模型、多相机、TF 和多目标跟踪在确有需求时再增加。

## 7. 风险与实施顺序

| 风险 | 处理 |
| --- | --- |
| 相机渲染降低 RTF | P1 记录 RTF；关闭桥接不等于停止渲染，低负载实验使用原无相机模型 |
| 桥接版本与模型不匹配 | 先核验版本、真实话题和 frame，未验证能力不写成已支持 |
| FRD/FLU 或公共原点错误 | P2 基向量测试及多位置对齐检查；不能仅靠往返一致性 |
| 固定目标平面偏离实际 | 记录实际高度差，真实图像误差单独评估 |
| truth 被误当端到端视觉验收 | 将几何自检、桥接验收和独立图像验证分开报告 |
| 接收时间近似掩盖不同采样时刻 | 仅用于本轮旁路 truth；YOLO/闭环前完成真实时间映射 |

执行顺序固定为 **P1 桥接核验 → P2 离线几何测试 → P3 truth 消息验证 → 独立图像验证**。本轮只完成旁路基础设施，导引不消费 `/vision/target_pose`。
