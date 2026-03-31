#include "../include/fsmpx4.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string_view>
#include <utility>

namespace fsmpx4
{
namespace
{
// 便于打印状态名称的小工具函数
constexpr std::string_view toString(FSMPX4::State state)
{
    switch (state)
    {
        case FSMPX4::State::MANUAL_CTRL: return "MANUAL_CTRL";
        case FSMPX4::State::OFFBOARD_STABILIZED: return "OFFBOARD_STABILIZED";
        case FSMPX4::State::AUTO_HOVER: return "AUTO_HOVER";
        case FSMPX4::State::CMD_CTRL: return "CMD_CTRL";
    }
    return "UNKNOWN";
}

geometry_msgs::msg::Vector3 toVector3Msg(const types::Vector3& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

geometry_msgs::msg::Quaternion toQuaternionMsg(const Eigen::Quaterniond& q)
{
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

geometry_msgs::msg::Quaternion rotationToQuaternionMsg(const types::Matrix3& R)
{
    Eigen::Quaterniond q(R);
    q.normalize();
    return toQuaternionMsg(q);
}
}

FSMPX4::FSMPX4(const rclcpp::NodeOptions& options)
    : rclcpp::Node("fsmpx4_fsm", options),
      params_(),
      state_(State::MANUAL_CTRL),
      previous_state_(State::MANUAL_CTRL),
      target_initialized_(false),
      auto_hover_position_target_(types::Vector3::Zero()),
      auto_hover_yaw_target_(0.0),
      auto_cmd_position_target_(types::Vector3::Zero()),
      auto_cmd_yaw_target_(0.0),
      current_state_(std::make_shared<types::UAVState>()),
      uav_state_(current_state_.get()),
      wd_(types::Vector3::Zero()),
      thrust_(types::Vector3::Zero()),
      torque_(types::Vector3::Zero())
{
    if (!params_.load_from_node(*this))
    {
        RCLCPP_FATAL(this->get_logger(), "参数加载失败，FSMPX4 将使用默认配置");
    }
    else
    {
        logLoadedParams();
    }

    // 将输入模块直接绑定到底层共享 UAVState，保持与 PX4CtrlFSM 一致
    imu_input_.setUAVStatePtr(current_state_.get());
    rc_input_.setUAVStatePtr(current_state_.get());
    position_input_.setUAVStatePtr(current_state_.get());
    hover_input_.setUAVStatePtr(current_state_.get());
    hover_input_.setDefaultHoverThrust(params_.thr_map.hover_percentage);
    current_state_->hover_thrust = params_.thr_map.hover_percentage;

    control::PositionAttitudeController::Config controller_cfg;
    controller_cfg.load_from_params(params_);
    if (!controller_.initialize(controller_cfg))
    {
        RCLCPP_WARN(this->get_logger(), "Controller initialization failed with provided params");
    }

    initializePublishers();
    initializeSubscribers();
    ControlLoop(params_.basic.ctrl_freq_max);

    RCLCPP_INFO(this->get_logger(), "FSM initialized at %.1f Hz", params_.basic.ctrl_freq_max);
}

void FSMPX4::initializePublishers()   //发布话题
{
    // 保持与 PX4 默认 QoS 对齐：只保留最新指令，降低时延
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);

    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", best_effort_qos);
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", best_effort_qos);
    attitude_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/fmu/in/vehicle_attitude_setpoint", best_effort_qos);

    auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);

    debug_pub_ = this->create_publisher<fsmpx4::msg::FSMDebug>(
        "~/debug", debug_qos);
}

void FSMPX4::initializeSubscribers()   //订阅话题
{
    // 订阅端使用 BEST_EFFORT，兼容 PX4 发布端的 QoS 设置
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);

    const char* manual_topic = params_.basic.use_fmu_manual_topic
        ? "/fmu/out/manual_control_setpoint"
        : "/rc/manual_control_setpoint";

    rc_sub_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        manual_topic, px4_qos,
        [this](const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
        {
            rc_input_.feed(msg);
        });

    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", px4_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
        {
            imu_input_.feedAttitude(msg);
        });

    angular_vel_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        "/fmu/out/vehicle_angular_velocity", px4_qos,
        [this](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
        {
            imu_input_.feedAngularVelocity(msg);
        });

    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
        {
            position_input_.feed(msg);
        });

    hover_thrust_sub_ = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
        "/fmu/out/hover_thrust_estimate", px4_qos,
        [this](const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg)
        {
            hover_input_.feed(msg);
        });

    rates_setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
        "/fmu/out/vehicle_rates_setpoint", px4_qos,
        [this](const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg)
        {
            onVehicleRatesSetpoint(msg);
        });

    thrust_setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
        "/fmu/out/vehicle_thrust_setpoint", px4_qos,
        [this](const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg)
        {
            onVehicleThrustSetpoint(msg);
        });

    torque_setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>(
        "/fmu/out/vehicle_torque_setpoint", px4_qos,
        [this](const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr msg)
        {
            onVehicleTorqueSetpoint(msg);
        });
}

void FSMPX4::ControlLoop(double frequency_hz)
{
    const double freq = std::max(1.0, frequency_hz);
    const auto period = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 / freq));

    control_timer_ = this->create_wall_timer(period, [this]() { process(); });
}

void FSMPX4::process()
{
    const auto now = this->get_clock()->now();
    
    // 初始化控制输出
    output_ = types::ControlOutput{};
    output_.timestamp = now.seconds();
    
    // 初始化 UAV 指令
    cmd_ = types::UAVCommand{};
    cmd_.timestamp = now.seconds();
    
    switch (state_)
    {
        case State::MANUAL_CTRL:
            handleManual(now);
            break;
        case State::OFFBOARD_STABILIZED:
            handleOffboardStabilized(now);
            break;
        case State::AUTO_HOVER:
            handleAutoHover(now);
            break;
        case State::CMD_CTRL:
            handleCommandControl(now);
            break;
    }
    previous_state_ = state_;
    // static auto last_control_print_time = this->get_clock()->now();
    // if ((now - last_control_print_time).seconds() >= 2.0)  // 每1秒打印一次
    // {
    //     printControlData();
    //     last_control_print_time = now;
    // }
    output_.wd = wd_;
    output_.thrust_vector = thrust_;
    output_.moment = torque_;  
    publishDebugMessage(now);
}

// 手动状态处理函数
void FSMPX4::handleManual(const rclcpp::Time& now)
{
    // 检测遥控开关进入姿态 offboard，并确保已有 IMU 数据
    if (rc_input_.enter_offboard_stabilized_mode)
    {
        if (!imuReady(now))
        {
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

// 姿态 offboard 稳定状态处理函数
void FSMPX4::handleOffboardStabilized(const rclcpp::Time& now)
{

    if (!(rc_input_.is_offboard_stabilized_mode && imuReady(now)))
    {
        enterState(State::MANUAL_CTRL);
        publishOffboardMode(false);
        return;
    }

    if (rc_input_.enter_hover_mode)
    {
        if (!positionReady(now))
        {
            RCLCPP_WARN(this->get_logger(), "Rejected AUTO_HOVER entry: position invalid");
            return;
        }

        target_initialized_ = false;
        enterState(State::AUTO_HOVER);
        return;
    }
    toggleOffboardMode(true);
    publishOffboardMode(true);
    output_.Rd = rc_input_.getDesiredRotationMatrix();
    output_.thrust = rc_input_.getDesiredThrust();
    output_.valid = true;
    if (output_.valid)
    {
        publishAttitudeCommand(output_.Rd, output_.thrust, now);
    }
    return;
}
// 自动悬停状态处理函数
void FSMPX4::handleAutoHover(const rclcpp::Time& now)
{
    // 悬停态继续依赖 offboard+IMU，缺失即退回
    if (!(rc_input_.is_offboard_stabilized_mode && imuReady(now)))
    {
        enterState(State::MANUAL_CTRL);
        publishOffboardMode(false);
        return;
    }

    if (rc_input_.enter_command_mode)
    {
        target_initialized_ = false;
        enterState(State::CMD_CTRL);
        return;
    }

    if (!rc_input_.is_hover_mode)
    {
        target_initialized_ = false;
        enterState(State::OFFBOARD_STABILIZED);
        return;
    }
    toggleOffboardMode(true);

    if (!target_initialized_)
    {
        auto_hover_position_target_ = uav_state_->position;
        auto_hover_yaw_target_ = std::atan2(uav_state_->rotation(1, 0), uav_state_->rotation(0, 0));
        target_initialized_ = true;
    }
    cmd_.timestamp = now.seconds();
    cmd_.position = auto_hover_position_target_;
    cmd_.yaw_desired = auto_hover_yaw_target_;
    cmd_.velocity.setZero();
    cmd_.acceleration.setZero();
    cmd_.b1d = types::Vector3(std::cos(cmd_.yaw_desired), std::sin(cmd_.yaw_desired), 0.0);
    output_ = controller_.computeControl(*uav_state_, cmd_);
    output_.valid=true;
    if (output_.valid)
    {
        publishOffboardMode(true);
        publishAttitudeCommand(output_.Rd, output_.thrust, now);
    }
}

// 指令控制状态处理函数
void FSMPX4::handleCommandControl(const rclcpp::Time& now)
{
    // 指令态失去 offboard 或 IMU 即脱离自动控制
    if (!(rc_input_.is_offboard_stabilized_mode && imuReady(now)))
    {
        target_initialized_ = false;
        enterState(State::MANUAL_CTRL);
        publishOffboardMode(false);
        return;
    }
    if (!rc_input_.is_command_mode)
    {
        target_initialized_ = false;
        enterState(State::AUTO_HOVER);
        return;
    }
    toggleOffboardMode(true);
    publishOffboardMode(true);

    // if (!target_initialized_)
    // {

    //     auto_cmd_position_target_ = uav_state_->position;
    //     auto_cmd_yaw_target_ = std::atan2(uav_state_->rotation(1, 0), uav_state_->rotation(0, 0));
    //     target_initialized_ = true;
    // }

    cmd_ = types::UAVCommand{};
    cmd_.timestamp = now.seconds();
    cmd_.position = auto_cmd_position_target_;
    cmd_.yaw_desired = auto_cmd_yaw_target_;
    cmd_.b1d = types::Vector3(std::cos(cmd_.yaw_desired), std::sin(cmd_.yaw_desired), 0.0);

    output_ = controller_.computeControl(*uav_state_, cmd_);
    output_.valid = true;
    if (output_.valid)
    {
        publishOffboardMode(true);
        publishAttitudeCommand(output_.Rd, output_.thrust, now);
    }
}

// 状态切换打印函数
void FSMPX4::enterState(State next_state)
{
    if (next_state == state_)
    {
        return;
    }

    // 打印状态变化便于现场调试
    RCLCPP_INFO(this->get_logger(), "\033[32mFSM transition: %.*s -> %.*s\033[0m",
                static_cast<int>(toString(state_).size()), toString(state_).data(),
                static_cast<int>(toString(next_state).size()), toString(next_state).data());

    previous_state_ = state_;
    state_ = next_state;
}

// imu数据检测函数
bool FSMPX4::imuReady(const rclcpp::Time& now)
{

    return imu_input_.is_received(now);
}
// 位置数据检测函数
bool FSMPX4::positionReady(const rclcpp::Time& now)
{
    return position_input_.is_received(now) && position_input_.isPositionValid();
}
// 发布offboard模式函数
void FSMPX4::publishOffboardMode(bool use_attitude)
{
    if (!offboard_mode_pub_)
    {
        return;
    }

    px4_msgs::msg::OffboardControlMode msg{};
    const auto stamp = this->get_clock()->now();
    msg.timestamp = stamp.nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = use_attitude;
    msg.body_rate = false;
    msg.thrust_and_torque = !use_attitude;
    msg.direct_actuator = false;

    offboard_mode_pub_->publish(msg);
}

// 发布飞行模式函数
void FSMPX4::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    if (!vehicle_command_pub_)
    {
        return;
    }

    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.command = command;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_pub_->publish(cmd);
}
// 发布offboard模式函数
bool FSMPX4::toggleOffboardMode(bool on_off)
{
    auto now = this->get_clock()->now();

    if (!offboard_has_last_command_ || on_off != offboard_last_command_state_)
    {
        offboard_has_last_command_ = true;
        offboard_last_command_state_ = on_off;
        offboard_toggle_active_ = true;
        offboard_toggle_start_time_ = now;
    }

    if (!offboard_toggle_active_)
    {
        return true;
    }

    rclcpp::Duration elapsed = now - offboard_toggle_start_time_;
    if (elapsed < offboard_toggle_duration_)
    {
        const auto mode_cmd = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        if (on_off)
        {
            publishVehicleCommand(mode_cmd, 1.0f, 6.0f);
        }
        else
        {
            publishVehicleCommand(mode_cmd, 1.0f, 7.0f);
        }
    }
    else
    {
        offboard_toggle_active_ = false;
    }

    return true;
}

// 发布姿态命令函数
void FSMPX4::publishAttitudeCommand(const types::Matrix3& attitude,
    double thrust,
    const rclcpp::Time& stamp)
{
if (!attitude_pub_)
{
return;
}

const types::Quaternion q(attitude);

px4_msgs::msg::VehicleAttitudeSetpoint msg{};
msg.timestamp = stamp.nanoseconds() / 1000;
msg.q_d[0] = static_cast<float>(q.w());
msg.q_d[1] = static_cast<float>(q.x());
msg.q_d[2] = static_cast<float>(q.y());
msg.q_d[3] = static_cast<float>(q.z());
msg.thrust_body[0] = 0.0f;
msg.thrust_body[1] = 0.0f;
msg.thrust_body[2] = static_cast<float>(thrust);

attitude_pub_->publish(msg);
}
// 参数打印
void FSMPX4::logLoadedParams() const
{
    const auto& gains = params_.gains;
    const auto& phys = params_.physical;
    const auto& limits = params_.limits;

    RCLCPP_INFO(this->get_logger(),
                "控制频率: %.1f Hz, max_manual_vel: %.2f, use_integral: %s",
                params_.basic.ctrl_freq_max,
                params_.basic.max_manual_vel,
                params_.basic.use_integral ? "true" : "false");

    const char* rc_topic = params_.basic.use_fmu_manual_topic
        ? "/fmu/out/manual_control_setpoint"
        : "/rc/manual_control_setpoint";
    RCLCPP_INFO(this->get_logger(),
                "遥控输入来源: %s",
                rc_topic);

    RCLCPP_INFO(this->get_logger(),
                "悬停推力比例: %.3f", params_.thr_map.hover_percentage);

    RCLCPP_INFO(this->get_logger(),
                "位置增益 Kp=(%.3f, %.3f, %.3f) 速度增益 Kv=(%.3f, %.3f, %.3f)",
                gains.Kp_x, gains.Kp_y, gains.Kp_z,
                gains.Kv_x, gains.Kv_y, gains.Kv_z);

    RCLCPP_INFO(this->get_logger(),
                "积分增益 Kvi=(%.3f, %.3f, %.3f)",
                gains.Kvi_x, gains.Kvi_y, gains.Kvi_z);

    RCLCPP_INFO(this->get_logger(),
                "物理量 mass=%.3fkg g=%.3f I=(%.5f, %.5f, %.5f)",
                phys.mass, phys.gravity, phys.Ixx, phys.Iyy, phys.Izz);

    RCLCPP_INFO(this->get_logger(),
                "推力/力矩限制 max_thr=%.3f min_thr=%.3f max_torque=%.3f",
                limits.max_thrust, limits.min_thrust, limits.max_torque);

}

void FSMPX4::publishDebugMessage(const rclcpp::Time& stamp)
{
    if (!debug_pub_)
    {
        return;
    }

    fsmpx4::msg::FSMDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = this->get_fully_qualified_name();

    if (current_state_)
    {
        const auto& state = *current_state_;
        msg.uav_position = toVector3Msg(state.position);
        msg.uav_velocity = toVector3Msg(state.velocity);
        msg.uav_angular_velocity = toVector3Msg(state.angular_velocity);
        msg.uav_hover_thrust = state.hover_thrust;
        msg.uav_attitude = rotationToQuaternionMsg(state.rotation);
    }

    msg.cmd_position = toVector3Msg(cmd_.position);
    msg.cmd_velocity = toVector3Msg(cmd_.velocity);
    msg.cmd_acceleration = toVector3Msg(cmd_.acceleration);
    msg.cmd_b1d = toVector3Msg(cmd_.b1d);
    msg.cmd_yaw = cmd_.yaw_desired;
    msg.cmd_attitude = rotationToQuaternionMsg(cmd_.Rd);

    msg.control_thrust_vector = toVector3Msg(output_.thrust_vector);
    msg.control_thrust = output_.thrust;
    msg.control_moment = toVector3Msg(output_.moment);
    msg.control_acceleration = toVector3Msg(output_.A);
    msg.control_attitude = rotationToQuaternionMsg(output_.Rd);
    msg.control_wd = toVector3Msg(output_.wd);
    msg.control_valid = output_.valid;

    debug_pub_->publish(std::move(msg));
}

// UAV姿态数据打印
void FSMPX4::printUAVAttitude()
{
    if (uav_state_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "🚁 UAV状态无效");
        return;
    }

    // 将旋转矩阵转换为欧拉角 (Roll, Pitch, Yaw)
    const auto& R = uav_state_->rotation;
    
    // 从旋转矩阵提取欧拉角 (ZYX顺序)
    double roll = std::atan2(R(2, 1), R(2, 2));
    double pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
    double yaw = std::atan2(R(1, 0), R(0, 0));
    
    // 转换为度数
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    RCLCPP_INFO(this->get_logger(), "🚁 UAV姿态数据:");
    RCLCPP_INFO(this->get_logger(),
                "  欧拉角: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°",
                roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "🚁 UAV姿态数据:");
    
    RCLCPP_INFO(this->get_logger(),
                "  位置: [%.3f, %.3f, %.3f] m",
                uav_state_->position.x(), uav_state_->position.y(), uav_state_->position.z());
    
    RCLCPP_INFO(this->get_logger(),
                "  速度: [%.3f, %.3f, %.3f] m/s",
                uav_state_->velocity.x(), uav_state_->velocity.y(), uav_state_->velocity.z());
    
    RCLCPP_INFO(this->get_logger(),
                "  角速度: [%.3f, %.3f, %.3f] rad/s",
                uav_state_->angular_velocity.x(), uav_state_->angular_velocity.y(), uav_state_->angular_velocity.z());
    
    if (std::isfinite(uav_state_->hover_thrust))
    {
        RCLCPP_INFO(this->get_logger(),
                    "  悬停推力: %.3f", uav_state_->hover_thrust);
    }
    
    RCLCPP_INFO(this->get_logger(),
                "  数据时间戳: %.3f s", uav_state_->timestamp);
}

// 控制数据打印（命令和输出）
void FSMPX4::printControlData()
{
    RCLCPP_INFO(this->get_logger(), "🎮 控制数据:");
    
    // 打印目标初始化状态
    RCLCPP_INFO(this->get_logger(), "  目标初始化: %s", 
                target_initialized_ ? "✓ 已初始化" : "✗ 未初始化");
    
    // 打印 UAV 命令
    RCLCPP_INFO(this->get_logger(), "  📋 UAV命令:");
    RCLCPP_INFO(this->get_logger(),
                "    位置: [%.3f, %.3f, %.3f] m",
                cmd_.position.x(), cmd_.position.y(), cmd_.position.z());
    RCLCPP_INFO(this->get_logger(),
                "    速度: [%.3f, %.3f, %.3f] m/s",
                cmd_.velocity.x(), cmd_.velocity.y(), cmd_.velocity.z());
    RCLCPP_INFO(this->get_logger(),
                "    加速度: [%.3f, %.3f, %.3f] m/s²",
                cmd_.acceleration.x(), cmd_.acceleration.y(), cmd_.acceleration.z());
    RCLCPP_INFO(this->get_logger(),
                "    偏航角: %.2f°",
                cmd_.yaw_desired * 180.0 / M_PI);
    
    // 打印控制输出
    RCLCPP_INFO(this->get_logger(), "  🚀 控制输出:");
    RCLCPP_INFO(this->get_logger(),
                "    四元数: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                output_.qd.w(), output_.qd.x(), output_.qd.y(), output_.qd.z());
    RCLCPP_INFO(this->get_logger(),
                "    推力: %.3f",
                output_.thrust);
    RCLCPP_INFO(this->get_logger(),
                "    推力向量: [%.3f, %.3f, %.3f] N",
                output_.thrust_vector.x(), output_.thrust_vector.y(), output_.thrust_vector.z());
    RCLCPP_INFO(this->get_logger(),
                "    力矩: [%.3f, %.3f, %.3f] N⋅m",
                output_.moment.x(), output_.moment.y(), output_.moment.z());
    RCLCPP_INFO(this->get_logger(),
                "    期望加速度: [%.3f, %.3f, %.3f] m/s²",
                output_.A.x(), output_.A.y(), output_.A.z());
    RCLCPP_INFO(this->get_logger(),
                "    旋转误差: [%.3f, %.3f, %.3f]",
                output_.eR.x(), output_.eR.y(), output_.eR.z());
    RCLCPP_INFO(this->get_logger(),
                "    角速度误差: [%.3f, %.3f, %.3f] rad/s",
                output_.eW.x(), output_.eW.y(), output_.eW.z());
    RCLCPP_INFO(this->get_logger(),
                "    期望角速度: [%.3f, %.3f, %.3f] rad/s",
                output_.wd.x(), output_.wd.y(), output_.wd.z());
    RCLCPP_INFO(this->get_logger(),
                "    有效性: %s",
                output_.valid ? "✓" : "✗");
    
    RCLCPP_INFO(this->get_logger(),
                "    时间戳: %.3f s", output_.timestamp);
}

// Vehicle Rates Setpoint 回调函数
void FSMPX4::onVehicleRatesSetpoint(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg)
{
    // 存储角速度设定值到 wd_
    wd_(0) = msg->roll;   // Roll rate
    wd_(1) = msg->pitch;  // Pitch rate
    wd_(2) = msg->yaw;    // Yaw rate
}

// Vehicle Thrust Setpoint 回调函数
void FSMPX4::onVehicleThrustSetpoint(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg)
{
    // 存储推力设定值到 thrust_
    thrust_(0) = msg->xyz[0];  // Thrust X
    thrust_(1) = msg->xyz[1];  // Thrust Y
    thrust_(2) = msg->xyz[2];  // Thrust Z
}

// Vehicle Torque Setpoint 回调函数
void FSMPX4::onVehicleTorqueSetpoint(const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr msg)
{
    // 存储力矩设定值到 torque_
    torque_(0) = msg->xyz[0];  // Torque X
    torque_(1) = msg->xyz[1];  // Torque Y
    torque_(2) = msg->xyz[2];  // Torque Z
}

} // namespace fsmpx4
