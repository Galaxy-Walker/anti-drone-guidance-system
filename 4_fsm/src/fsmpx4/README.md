# fsmpx4 - PX4 有限状态机控制节点

## 项目简介

`fsmpx4` 是一个基于 ROS 2 的 PX4 外部控制有限状态机（FSM）节点，为四旋翼无人机提供从纯手动控制到自动悬停、指令跟踪的连续控制体验。项目封装了 PX4 的 offboard 通信接口，集成了位置-姿态控制器与参数管理模块，便于在研发环境中快速完成从实验验证到飞行测试的闭环控制。

## 1. ROS2 参数传递机制

### 1.1 参数配置文件

项目使用 YAML 格式的配置文件进行参数管理，默认配置文件位于 `config/fsm.yaml`：

```yaml
fsmpx4_fsm:
  ros__parameters:
    # 基础参数
    ctrl_freq_max: 200.0
    max_manual_vel: 1.0
    use_manual_gate: false
    manual_control_topic: false
    
    # 推力映射参数
    "thr_map.hover_percentage": 0.67
    
    # 控制器参数 - 位置控制增益
    "control.Kp_x": 1.0
    "control.Kp_y": 2.0
    "control.Kp_z": 2.0
    "control.Kv_x": 3.0
    "control.Kv_y": 3.0
    "control.Kv_z": 4.5
    "control.Kvi_x": 0.02
    "control.Kvi_y": 0.02
    "control.Kvi_z": 0.1
    
    # 控制器参数 - 物理参数
    "control.mass": 2.0
    "control.gravity": 9.81
    "control.Ixx": 0.03232
    "control.Iyy": 0.03232
    "control.Izz": 0.04238
    
    # 控制器参数 - 控制限制
    "control.max_thrust": -0.1
    "control.min_thrust": -0.9
    "control.max_torque": 0.5
    
    # 控制器参数 - 积分控制
    "control.use_integral": false
```

### 1.2 参数加载机制

参数加载通过 `src/param_loader.cpp` 实现，主要特点：

- **分层参数结构**：使用嵌套结构体组织参数，包括基础参数、增益参数、物理参数、限制参数等
- **默认值机制**：每个参数都有合理的默认值，确保系统在参数缺失时仍能正常运行
- **类型安全**：使用模板函数确保参数类型正确性
- **错误处理**：参数加载失败时提供详细的错误信息

```cpp
// 参数加载示例
bool load_params_from_node(rclcpp::Node& node, FSMParams& params)
{
    try {
        params.basic.ctrl_freq_max = node.declare_parameter<double>("ctrl_freq_max", params.basic.ctrl_freq_max);
        params.gains.Kp_x = node.declare_parameter<double>("control.Kp_x", params.gains.Kp_x);
        params.physical.mass = node.declare_parameter<double>("control.mass", params.physical.mass);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node.get_logger(), "参数加载失败: %s", e.what());
        return false;
    }
}
```

### 1.3 参数使用方式

- **启动时加载**：通过 `--ros-args --params-file config/fsm.yaml` 指定参数文件
- **动态修改**：使用 `ros2 param set` 命令动态修改参数
- **参数验证**：启动时自动验证参数合理性，不合理的参数会使用默认值

## 2. 输入处理模块 (input.cpp)

### 2.1 遥控器输入处理 (RC_Receiver)

`RC_Receiver` 类负责处理遥控器输入，主要功能：

- **通道数据处理**：处理滚转、俯仰、油门、偏航四个通道的输入
- **死区处理**：对输入信号进行死区处理，避免微小抖动
- **模式检测**：检测遥控器开关状态，判断当前飞行模式
- **姿态指令生成**：根据遥控器输入生成期望的姿态和推力指令

```cpp
// 遥控器输入处理核心逻辑
void RC_Receiver::feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr pMsg)
{
    // 通道数据处理
    ch[0] = msg.roll;    // 滚转
    ch[1] = msg.pitch;   // 俯仰  
    ch[2] = msg.throttle; // 油门
    ch[3] = msg.yaw;     // 偏航
    
    // 死区处理
    for (double &channel : ch) {
        if (channel > DEAD_ZONE) {
            channel = (channel - DEAD_ZONE) / (1.0 - DEAD_ZONE);
        } else if (channel < -DEAD_ZONE) {
            channel = (channel + DEAD_ZONE) / (1.0 - DEAD_ZONE);
        } else {
            channel = 0.0;
        }
    }
    
    // 模式检测
    mode = (msg.aux1 + 1.0) / 2.0;
    gear = (msg.aux2 + 1.0) / 2.0;
}
```

### 2.2 IMU 数据处理 (IMU_Reader)

`IMU_Reader` 类处理来自 PX4 的 IMU 数据：

- **姿态数据**：处理四元数姿态信息，转换为旋转矩阵
- **角速度数据**：处理角速度信息
- **数据有效性检查**：检查数据的时间戳和有效性

### 2.3 位置数据处理 (Position_Reader)

`Position_Reader` 类处理位置和速度数据：

- **位置信息**：处理本地位置坐标
- **速度信息**：处理本地速度信息
- **数据有效性**：检查位置和速度数据的有效性标志

### 2.4 悬停推力处理 (HoverThrust_Reader)

`HoverThrust_Reader` 类处理悬停推力估计：

- **推力估计**：从 PX4 获取悬停推力估计值
- **默认值处理**：当估计值无效时使用可配置的默认值
- **数据更新**：实时更新无人机状态中的悬停推力值

## 3. 数据结构定义 (types.h)

### 3.1 核心数据结构

`types.h` 定义了系统中使用的所有核心数据结构：

#### UAVState - 无人机状态
```cpp
struct UAVState {
    Vector3 position;           // 位置 [x, y, z]
    Vector3 velocity;           // 速度 [vx, vy, vz]
    Matrix3 rotation;           // 旋转矩阵 R
    Vector3 angular_velocity;   // 角速度 [wx, wy, wz]
    double hover_thrust;        // 悬停推力值
    double timestamp;           // 时间戳
};
```

#### UAVCommand - 无人机指令
```cpp
struct UAVCommand {
    Vector3 position;           // 期望位置
    Vector3 velocity;           // 期望速度
    Vector3 acceleration;       // 期望加速度
    Vector3 b1d;                // 期望方向向量
    double yaw_desired;         // 期望偏航角
    Matrix3 Rd;                 // 期望旋转矩阵
    Vector3 Wd;                 // 期望角速度
    double timestamp;           // 时间戳
};
```

#### ControlOutput - 控制输出
```cpp
struct ControlOutput {
    double thrust;              // 归一化推力指令
    Vector3 thrust_vector;      // 完整推力向量
    Vector3 moment;             // 力矩
    Vector3 A;                  // 期望加速度向量
    Vector3 eR;                 // 旋转误差
    Vector3 eW;                 // 角速度误差
    Matrix3 Rd;                 // 期望旋转矩阵
    Quaternion qd;              // 期望姿态四元数
    Vector3 wd;                 // 期望角速度
    bool valid;                 // 输出有效性
    double timestamp;           // 时间戳
};
```

### 3.2 控制器配置结构

#### ControllerGains - 控制器增益
```cpp
struct ControllerGains {
    Vector3 position_gains;            // 位置控制增益
    Vector3 velocity_gains;            // 速度控制增益
    Vector3 attitude_gains;            // 姿态控制增益
    Vector3 angular_velocity_gains;    // 角速度控制增益
    Vector3 integral_position_gains;   // 位置积分增益
    Vector3 integral_attitude_gains;   // 姿态积分增益
    bool use_integral;                 // 是否使用积分控制
};
```

#### UAVParameters - 物理参数
```cpp
struct UAVParameters {
    double mass;                // 质量 [kg]
    double gravity;             // 重力加速度 [m/s^2]
    Matrix3 inertia;            // 惯性矩阵
    double max_thrust;          // 最大推力
    double min_thrust;          // 最小推力
    double max_torque;          // 最大力矩
};
```

## 4. 控制器实现 (control.cpp)

### 4.1 位置-姿态控制器

`PositionAttitudeController` 类实现了基于几何控制理论的位置-姿态控制器：

#### 控制算法
控制器采用分层控制结构：
1. **位置控制层**：根据位置和速度误差计算期望加速度
2. **姿态控制层**：根据期望加速度计算期望姿态和推力

#### 核心控制方程
```cpp
// 位置控制：A = -Kp*eX - Kv*eV - Kvi*∫eX - m*g*e3 + m*a_d
const Vector3 A = -cfg_.kp.cwiseProduct(eX)
                  - cfg_.kv.cwiseProduct(eV)
                  - cfg_.kvi.cwiseProduct(position_integral_)
                  - cfg_.mass * cfg_.gravity * Vector3::UnitZ()
                  + cfg_.mass * cmd.acceleration;
```

#### 姿态计算
```cpp
// 机体系 z 轴与合力方向相反
const Vector3 b3c = (-A / a_norm);

// 计算期望姿态矩阵
Matrix3 Rd;
Rd.col(0) = b1c;  // x 轴
Rd.col(1) = b2c;  // y 轴  
Rd.col(2) = b3c;  // z 轴
```

### 4.2 积分控制

控制器支持积分控制以消除稳态误差：

```cpp
void PositionAttitudeController::accumulateIntegral(const Vector3& eX, double dt) const
{
    if (!cfg_.use_integral) {
        position_integral_.setZero();
        return;
    }
    
    if (dt <= 0.0) return;
    
    position_integral_ += eX * dt;
    
    // 积分限幅
    const double limit = std::abs(cfg_.integral_limit);
    if (limit > 0.0) {
        for (int i = 0; i < 3; ++i) {
            position_integral_(i) = std::clamp(position_integral_(i), -limit, limit);
        }
    }
}
```

### 4.3 推力归一化

控制器将计算得到的推力归一化到 PX4 期望的范围：

```cpp
// 推力归一化
const double hover_thrust = (std::isfinite(state.hover_thrust) && state.hover_thrust > 0.0)
                            ? state.hover_thrust
                            : cfg_.hover_thrust_default;
const double weight = cfg_.mass * cfg_.gravity;
double normalized_thrust = 0.0;
if (weight > 0.0) {
    const double thrust_ratio = a_norm / weight;
    normalized_thrust = thrust_ratio * hover_thrust;
}
// PX4 约定：机体系 Z 轴正向朝下，期望向上的推力为负值
output.thrust = -normalized_thrust;
```

## 5. FSM 状态逻辑

### 5.1 状态定义

系统定义了四个主要状态：

```cpp
enum class State {
    MANUAL_CTRL,           // 手动控制
    OFFBOARD_STABILIZED,   // 离板姿态稳定
    AUTO_HOVER,           // 自动悬停
    CMD_CTRL              // 指令控制
};
```

### 5.2 状态转换逻辑

#### MANUAL_CTRL (手动控制)
- **进入条件**：系统启动或失去必要传感器
- **行为**：完全交给遥控器，关闭 offboard 控制
- **退出条件**：遥控器进入 offboard 姿态模式且 IMU 可用

```cpp
void FSMPX4::handleManual(const rclcpp::Time& now)
{
    if (rc_input_.enter_offboard_stabilized_mode) {
        if (!imuReady(now)) {
            RCLCPP_WARN(this->get_logger(), "Rejected OFFBOARD_STABILIZED entry: IMU not ready");
            return;
        }
        target_initialized_ = false;
        enterState(State::OFFBOARD_STABILIZED);
        return;
    }
    toggleOffboardMode(false);
    publishOffboardMode(false);
}
```

#### OFFBOARD_STABILIZED (离板姿态稳定)
- **进入条件**：遥控器进入 offboard 姿态模式且 IMU 可用
- **行为**：发布姿态+油门指令，实现姿态助飞
- **退出条件**：失去 offboard 模式或 IMU 数据

```cpp
void FSMPX4::handleOffboardStabilized(const rclcpp::Time& now)
{
    if (!(rc_input_.is_offboard_stabilized_mode && imuReady(now))) {
        enterState(State::MANUAL_CTRL);
        publishOffboardMode(false);
        return;
    }
    
    if (rc_input_.enter_hover_mode) {
        if (!positionReady(now)) {
            RCLCPP_WARN(this->get_logger(), "Rejected AUTO_HOVER entry: Position not ready");
            return;
        }
        target_initialized_ = false;
        enterState(State::AUTO_HOVER);
        return;
    }
    
    // 发布姿态控制指令
    toggleOffboardMode(true);
    publishOffboardMode(true);
    output_.Rd = rc_input_.getDesiredRotationMatrix();
    output_.thrust = rc_input_.getDesiredThrust();
    output_.valid = true;
    if (output_.valid) {
        publishAttitudeCommand(output_.Rd, output_.thrust, now);
    }
}
```

#### AUTO_HOVER (自动悬停)
- **进入条件**：在姿态模式基础上触发悬停开关且位置可用
- **行为**：锁定当前位置，通过控制器保持定点
- **退出条件**：失去 offboard 模式、IMU 或位置数据

```cpp
void FSMPX4::handleAutoHover(const rclcpp::Time& now)
{
    if (!(rc_input_.is_offboard_stabilized_mode && imuReady(now))) {
        enterState(State::MANUAL_CTRL);
        publishOffboardMode(false);
        return;
    }
    
    if (rc_input_.enter_command_mode) {
        target_initialized_ = false;
        enterState(State::CMD_CTRL);
        return;
    }
    
    // 初始化悬停目标
    if (!target_initialized_) {
        auto_hover_position_target_ = uav_state_->position;
        auto_hover_yaw_target_ = std::atan2(uav_state_->rotation(1, 0), uav_state_->rotation(0, 0));
        target_initialized_ = true;
    }
    
    // 计算控制指令
    cmd_.position = auto_hover_position_target_;
    cmd_.yaw_desired = auto_hover_yaw_target_;
    cmd_.velocity.setZero();
    cmd_.acceleration.setZero();
    cmd_.b1d = types::Vector3(std::cos(cmd_.yaw_desired), std::sin(cmd_.yaw_desired), 0.0);
    
    output_ = controller_.computeControl(*uav_state_, cmd_);
    output_.valid = true;
    if (output_.valid) {
        publishOffboardMode(true);
        publishAttitudeCommand(output_.Rd, output_.thrust, now);
    }
}
```

#### CMD_CTRL (指令控制)
- **进入条件**：在悬停模式下进一步触发自动指令
- **行为**：跟踪预设的相对位置偏移，执行自动飞行任务
- **退出条件**：失去 offboard 模式、IMU 或退出指令模式

```cpp
void FSMPX4::handleCommandControl(const rclcpp::Time& now)
{
    if (!(rc_input_.is_offboard_stabilized_mode && imuReady(now))) {
        target_initialized_ = false;
        enterState(State::MANUAL_CTRL);
        publishOffboardMode(false);
        return;
    }
    
    if (!rc_input_.is_command_mode) {
        target_initialized_ = false;
        enterState(State::AUTO_HOVER);
        return;
    }
    
    // 计算控制指令
    cmd_.position = auto_cmd_position_target_;
    cmd_.yaw_desired = auto_cmd_yaw_target_;
    cmd_.b1d = types::Vector3(std::cos(cmd_.yaw_desired), std::sin(cmd_.yaw_desired), 0.0);
    
    output_ = controller_.computeControl(*uav_state_, cmd_);
    output_.valid = true;
    if (output_.valid) {
        publishOffboardMode(true);
        publishAttitudeCommand(output_.Rd, output_.thrust, now);
    }
}
```

### 5.3 状态转换图

```
MANUAL_CTRL
    ↓ (遥控器进入offboard模式 + IMU可用)
OFFBOARD_STABILIZED
    ↓ (触发悬停开关 + 位置可用)
AUTO_HOVER
    ↓ (触发指令模式)
CMD_CTRL
    ↑ (失去offboard/IMU/位置数据)
    ↑ (退出指令模式)
    ↑ (失去offboard/IMU数据)
```

### 5.4 安全机制

- **传感器检查**：每个状态都会检查必要的传感器数据可用性
- **超时保护**：传感器数据超时自动回退到安全状态
- **状态日志**：状态转换时打印详细的日志信息便于调试
- **渐进式控制**：从手动到自动的渐进式控制，确保飞行安全

## 构建与运行

### 构建
```bash
cd /home/jjxl/FSM_ws
colcon build --packages-select fsmpx4
source install/setup.bash
```

### 运行
```bash
# 启动主状态机节点
ros2 launch fsmpx4 fsmpx4_fsm.launch.py

# 启动点控制测试节点
ros2 launch fsmpx4 point_control.launch.py
```

### 话题接口

#### 订阅话题
- `/rc/manual_control_setpoint` 或 `/fmu/out/manual_control_setpoint`：遥控器输入
- `/fmu/out/vehicle_attitude`：姿态信息
- `/fmu/out/vehicle_angular_velocity`：角速度信息
- `/fmu/out/vehicle_local_position`：位置信息
- `/fmu/out/hover_thrust_estimate`：悬停推力估计

#### 发布话题
- `/fmu/in/offboard_control_mode`：离板控制模式
- `/fmu/in/vehicle_attitude_setpoint`：姿态设定点
- `/fmu/in/vehicle_command`：通用指令
- `~/debug`：调试信息

## 调试工具

- **FSMDebug 消息**：发布控制输入输出与无人机状态
- **rqt_plot**：可视化控制数据
- **point_control_node**：独立测试位置-姿态控制器

## 许可证

项目采用 GPLv3 授权。