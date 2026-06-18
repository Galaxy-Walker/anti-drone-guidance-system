# 4_fsm — 有限状态机探索

PX4 Offboard 有限状态机（FSM）的独立探索阶段。目标是形式化飞行状态切换逻辑，提升控制流程的安全性和可维护性。

## 内容

- [`src/fsmpx4/`](src/fsmpx4/) — C++ 版 FSM（`ament_cmake`），基于几何控制理论的位置-姿态控制器，实现了 `MANUAL → OFFBOARD_STABILIZED → AUTO_HOVER → CMD_CTRL` 的渐进式控制
- [`src/pixhawk_py/`](src/pixhawk_py/) — Python 版 FSM（`ament_python`），将 C++ 版的状态机迁移至 Python，行为等效但独立运行
- `src/pixhawk/` — 早期 C++ 基线的另一版本

## 与 5_AntiDrone 的关系

`5_AntiDrone` 的状态机设计借鉴了本阶段的经验（Offboard 切入确认、解锁超时、安全回退等），但实现独立，不直接依赖本目录的代码。

## 注意

请勿在同一套 PX4 主题上同时运行 `fsmpx4` 和 `pixhawk_py`，以免指令冲突。
