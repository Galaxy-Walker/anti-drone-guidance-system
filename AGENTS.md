# AGENTS.md

本文件面向在本仓库工作的 AI 编码助手与协作者。仓库文档统一使用中文，新增的注释、文档和提交信息也请使用中文。

## 1. 项目概览

基于 ROS 2 + PX4 Offboard + Gazebo 的无人机追踪/拦截系统，核心是三维比例导引（PN），并在仿真平台中对比 FOV 约束、CBF、MPPI 与 MPC 类控制器。仓库按研发阶段编号：`5`–`8` 是活跃代码，`1`–`4` 是历史阶段，只作参考，不要主动扩展。

| 目录 | 状态 | 内容 |
| --- | --- | --- |
| `1_初期/` | 历史 | 纯 Python 2D/3D PN 原型；`pn_nmpc_guidance_deprecated/` 已废弃 |
| `2_中期/` | 历史 | MAVSDK + PX4 SITL 联调探索 |
| `3_ROS2/` | 历史 | 最早的 ROS 2 工作空间，已被 `5_AntiDrone` 取代 |
| `4_fsm/` | 历史 | C++（`fsmpx4`）与 Python（`pixhawk_py`）两套 Offboard 状态机参考 |
| `5_AntiDrone/` | **活跃** | PX4 Offboard + 3D PN 闭环拦截主实现，含安全状态机与 Gazebo 评估器 |
| `6_Simulation/` | **活跃** | 3D 质点仿真（6 种算法横向对比）+ 双机 PX4 Gazebo 接入 + 算法文档 |
| `7_2Dsimulation/` | **活跃** | 2D 定高俯瞰追踪仿真 + 2D Gazebo 接入 |
| `8_MoCap/` | **活跃** | 动捕悬停/追踪/轨迹记录与回放（真机，部署到 Jetson 运行） |

每个编号目录都有独立 `README.md`；`6_Simulation/docs/`、`7_2Dsimulation/docs/` 存放算法原理、公式推导和实测结果表。改动行为后同步更新对应文档。

## 2. 环境与依赖规则

- **Python 包管理只用 uv**。纯 Python 仿真依赖由根目录 `pyproject.toml` / `uv.lock` 管理（Python ≥ 3.14，numpy / matplotlib / foxglove-sdk），首次使用执行 `uv sync`。禁止 `pip install` 全局安装。
- **ROS 2 包使用系统 Python（3.12）**，依赖由 ROS 提供。不要把 `rclpy`、`px4_msgs` 等 ROS 依赖加进根 `pyproject.toml`。
- 开发机：Ubuntu 24.04（WSL2）+ ROS 2 **Jazzy** + PX4 v1.16 SITL/Gazebo + Micro XRCE-DDS Agent + QGroundControl。
- `8_MoCap` 真机运行在另一台机器 `/home/nvidia/ws_ros2`（Jetson，ROS 2 **Humble** + PX4 1.15.4）。不要假设开发机的 Jazzy 与其完全一致，改动该模块时保持 1.15 话题与 API 兼容。
- 各工作空间的 `src/px4_msgs` 被 `.gitignore` 忽略、不在版本库中，当前固定在 px4_msgs `release/1.15`（`a1045ec`）。克隆后需自行放入或从其他已构建工作空间 source。**注意：`5_AntiDrone/src` 目前没有 `px4_msgs`**，构建/运行前需自行补充。
- Gazebo 闭环仿真所需的 QGC、PX4 SITL、`MicroXRCEAgent udp4 -p 8888` 一律由使用者在外部终端手动启动；代码和 launch **不得**尝试拉起这些进程。
- 无 CI、无 Makefile。验证靠下面的本地命令。

## 3. 常用命令

所有命令除非注明，均在仓库根目录下执行。ROS 工作空间各自独立，`colcon` 必须在对应编号目录内运行。

### 纯 Python 仿真（无需 ROS / PX4）

```bash
# 3D 多算法对比：stationary | linear | circle | all
cd 6_Simulation
uv run python main.py --scenario all
uv run python main.py --scenario circle --show --export-mcap

# 2D 定高俯瞰仿真
cd 7_2Dsimulation
uv run main.py --scenario all
```

### Gazebo 记录后处理（普通 uv 环境运行，不需要 source ROS）

```bash
cd 6_Simulation
uv run python plot_gazebo_csv.py outputs/gazebo/circle/pn_fov_nmpc/gazebo_samples.csv

cd 7_2Dsimulation
uv run plot_gazebo_csv.py outputs/gazebo2d/circle --output-dir outputs/circle
```

### ROS 2 构建与运行

```bash
# 6_Simulation：双机 Gazebo 接入（pursuer=/px4_1, target=/px4_2）
cd 6_Simulation
# 首次构建需用 --packages-up-to 把 px4_msgs 一并编译，之后可改用 --packages-select
colcon build --base-paths src --packages-select gazebosimulation
source install/setup.bash
ros2 launch gazebosimulation guidance.launch.py algorithm:=pn_fov_nmpc scenario:=circle

# 7_2Dsimulation：2D 接入
cd 7_2Dsimulation
colcon build --packages-up-to gazebosimulation2d
source install/setup.bash
ros2 launch gazebosimulation2d guidance.launch.py algorithm:=pn_mppi scenario:=circle

# 5_AntiDrone：单机拦截与闭环评估
cd 5_AntiDrone
colcon build --packages-select anti_drone_guidance
source install/setup.bash
ros2 launch anti_drone_guidance pn_guidance_launch.py
ros2 run anti_drone_guidance gazebo_evaluator --motions static line --speed-max 10 20

# 8_MoCap（真机部署，路径为 Jetson 上的 /home/nvidia/ws_ros2）
cd 8_MoCap
colcon build --packages-select px4_mocap_hover
colcon test --packages-select px4_mocap_hover && colcon test-result --verbose
```

- 反复调试 launch/config 时可用 `--symlink-install` 免去重复构建；只改 YAML 参数不需要重新 `colcon build`。
- `src/px4_msgs` 被 `.gitignore` 忽略、不在版本库中，所以工作空间首次构建必须用 `--packages-up-to <包名>` 把 `px4_msgs` 一并编译（`px4_msgs` 约需 3~4 分钟）；只有 `install/px4_msgs` 已存在时，才能用 `--packages-select` 只编译目标包。
- 从零构建 `gazebosimulation`、`gazebosimulation2d` 时按各自 README 补上 `--cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`，避免 CMake 选到错误的 Python。

### 测试

- `8_MoCap/src/px4_mocap_hover/test/` 有实质 pytest 单元测试，使用 `colcon test` 运行（见上）。
- `5_AntiDrone`、`3_ROS2`、`4_fsm` 的 `test/` 只有 ament copyright/flake8/pep257 模板检查，且仓库现状并非全部通过。**不要顺手做全仓格式化**，改动保持局部风格一致即可。

## 4. 代码约定与架构不变量

### 目录与模块职责

- 纯 Python 仿真核心在 `6_Simulation/src/pythonsimulation/`（3D）与 `7_2Dsimulation/src/pythonsimulation2d/`（2D）；`main.py` 通过 `sys.path.insert` 导入同目录 `src/`，ROS 包通过 `_ensure_*_on_path()` 向上查找导入。**保留这些机制**，不要把路径改成硬编码绝对路径，也不要为此把仿真模块包装成 ROS 包。
- Gazebo 接入包（`gazebosimulation`、`gazebosimulation2d`）只负责 Offboard 通信与坐标转换，导引算法一律调用 `compute_guidance()`。**禁止在 ROS 节点里复制第二套算法实现。**
- 坐标系转换集中在 `gazebosimulation/coordinates.py`（ENU↔NED）与 `8_MoCap/.../transforms.py`；PX4 的 NED 与算法内部的 ENU 只在边界处转换。
- `5_AntiDrone/` 的 `pn_guidance_core.py` 与 ROS/PX4 解耦；`target_provider.py` 负责目标源；`gazebo_evaluator.py` 只做批量评估，不启动仿真器。

### 算法与场景注册表（单一事实来源）

- 算法/场景名单只在对应 `config.py` 的 `ALGORITHMS`、`SCENARIOS`、`ALGORITHM_LABELS` 中定义：
  - 6_Simulation：`basic`、`basic_fov`、`pn_fov`、`pn_fov_cbf`、`pn_fov_mppi`、`pn_fov_nmpc`
  - 7_2Dsimulation：`basic`、`pn`、`pn_mppi`、`pn_nmpc`
- 新增算法必须同时更新：`config.py` → `guidance.compute_guidance()` 分支 → 标签/绘图 → launch 默认值与 README/文档。
- **已知文档漂移**：`6_Simulation/README.md` 的算法表仍写作 `pn`（实际代码是 `basic_fov`），且部分描述停留在 last-seen 预测（现为 α-β 滤波）。改算法时以 `config.py` 与 `docs/` 为准，并顺手修正 README。
- `pn_nmpc` / `nmpc_acceleration()` / `nmpc_w_*` 是历史代码标识，文档中把该候选枚举式控制器称为 **EMPC**（Enumerative MPC）。**不要重命名代码标识**（会破坏 CLI、输出目录与已生成图表），只在文档里注明二者等价。

### ROS 2 / PX4 约定

- 状态机进入导引前必须先连续发布 `offboard_control_mode` + setpoint 预热，再请求 Offboard/解锁，并确认 PX4 反馈（`VehicleControlMode`、`VehicleStatus`、`vehicle_command_ack`）后才进入控制阶段。
- `5_AntiDrone` 同时兼容带 `_v1` 与不带 `_v1` 的 PX4 话题命名，以适配不同 PX4/px4_msgs 组合；改话题名时保持这一兼容。
- PX4 QoS 固定使用 BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST(1)（见各节点 `px4_qos`），新增话题沿用。
- 参数分组命名（`guidance` / `flight` / `target` / `evaluation` / `debug`）与 launch 参数覆盖方式保持一致；新参数要同时写入 `config/*.yaml`、launch 文件与 README 参数表。

### 代码风格

- Python：`from __future__ import annotations`、`dataclasses(slots=True)`、类型注解、numpy 向量化。3D 模块用双引号，`8_MoCap` 沿用其现有单引号风格。
- `6_Simulation` 与 `7_2Dsimulation` 的注释为中文、解释"为什么"；`8_MoCap` 为英文。跟随所在文件的语言。
- `8_MoCap` 会控制真机：保留"拆桨测试"、预解锁稳定性检查、动捕超时即停发 Offboard 等安全语义，**不要放松解锁/Offboard 前置检查**。

## 5. 生成物、文档与提交

- `build/`、`install/`、`log/`、`outputs/`、`outputs_verify/`、`*.mcap`、`*.bag`、`results.csv`、`src/px4_msgs/` 均被忽略，**不要提交生成物**，也不要手工编辑它们。
- `7_2Dsimulation/svg/architecture_pn_nmpc.svg` 由 `svg/generate_architecture_svg.py` 生成：改图改脚本，不要手改 SVG。
- 文档为中文；`docs/` 中公式使用 LaTeX（`$...$` / `$$...$$`），结果表引用 `docs/assets/` 下的图片。根 `README.md` 汇总各模块，模块 README 描述细节。
- 提交信息使用中文，沿用现有风格：`新增…`、`更新…`、`重构…`、`修复…`、`移除…` + 简短说明。
- 单提交尽量聚焦一个主题（算法改动、文档同步、launch 调整分开），不要把格式化或批量重命名混进功能提交。

## 6. 硬性禁止

- 禁止全局安装：Python 依赖用 uv，ROS 依赖走 rosdep/apt；禁止 `pip install` 到系统环境。
- 仅允许使用 bun，禁止 npm / yarn / pnpm（本仓库当前无 JS 依赖）。
- 禁止批量删除文件；只允许删除明确路径的单个/多个文件，批量删除需暂停并交由用户手动操作。
- 禁止修改 `1_初期`~`4_fsm` 的历史代码，除非用户明确要求；`4_fsm` 的 `fsmpx4` 与 `pixhawk_py` 也**不可同时运行在同一 PX4 上**（指令冲突）。
