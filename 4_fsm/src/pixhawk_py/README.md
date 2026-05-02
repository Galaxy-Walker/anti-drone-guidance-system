# pixhawk_py

将 PX4 的机外（offboard）状态机从 C++ 版 `pixhawk` 包迁移至 Python，作为一个独立的 `ament_python` 包实现。

## 行为等效范围

本包在以下方面与 C++ 源码保持行为一致：

- 主题（Topics）：
  - 发布者（Publishers）：`/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`、`/fmu/in/vehicle_command`
  - 订阅者（Subscriptions）：`/fmu/out/vehicle_status_v1`、`/fmu/out/vehicle_control_mode`、`/fmu/out/battery_status`
- 订阅 QoS 设置：`KEEP_LAST(depth=10)`、`BEST_EFFORT`、`TRANSIENT_LOCAL`
- 定时器周期：`100 ms`
- 状态机流程及指令时序：
  - `INIT`
  - `SENDING_CONTROL`
  - `REQUESTING_OFFBOARD`
  - `ARMING`
  - `WAITING_ARM_CONFIRM`
  - `OFFBOARD_ACTIVE`
  - `RETURN_REQUESTED`
  - `ERROR`
- OFFBOARD/解锁（ARM）序列中的指令发布行为及超时策略
- 返航（RTL）触发机制及返航流程中的一次性日志门控（log gate）

## 非严格等效项

- 日志文本无需与 C++ 日志逐字节完全一致。
- 错误恢复方法 `handle_error_state()` 已完成迁移，但在主 `process()` 循环中未启用，以匹配当前 C++ 运行时行为。

## 构建与运行

在工作区根目录 `5_AntiDrone` 中执行：

```bash
colcon build --packages-select pixhawk_py
source install/setup.zsh
ros2 run pixhawk_py px4_node
```

## 与 C++ 包共存说明

- 验证原有行为时，请运行 C++ 版 `pixhawk`；
- 验证 Python 迁移效果时，请运行 `pixhawk_py`；
- 请勿在同一套 PX4 主题上同时运行两个节点，以免指令冲突。
