#include "../include/input.h"

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>

namespace fsmpx4 {
namespace input {

//==============================================================================
// RC_Receiver Implementation
//==============================================================================

RC_Receiver::RC_Receiver()
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    std::fill(std::begin(ch), std::end(ch), 0.0);

    mode = 0.0;
    gear = 0.0;
    last_mode = -1.0;
    last_gear = -1.0;
    filtered_mode = 0.0;

    is_command_mode = true;
    enter_command_mode = false;
    is_hover_mode = true;
    enter_hover_mode = false;
    is_offboard_stabilized_mode = false;
    enter_offboard_stabilized_mode = false;
    
    // 初始化新增的成员变量
    manual_yaw_sp_ = 0.0;
    yaw_sp_initialized_ = false;
    last_yaw_update_stamp_ = rclcpp::Clock(RCL_ROS_TIME).now();
    uav_state_ptr = nullptr;
}

void RC_Receiver::feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    ch[0] = msg.roll;
    ch[1] = msg.pitch;
    ch[2] = msg.throttle;
    ch[3] = msg.yaw;

    for (double &channel : ch)
    {
        if (channel > DEAD_ZONE)
        {
            channel = (channel - DEAD_ZONE) / (1.0 - DEAD_ZONE);
        }
        else if (channel < -DEAD_ZONE)
        {
            channel = (channel + DEAD_ZONE) / (1.0 - DEAD_ZONE);
        }
        else
        {
            channel = 0.0;
        }
    }

    mode = (msg.aux1 + 1.0) / 2.0;
    gear = (msg.aux2 + 1.0) / 2.0;
    filtered_mode = FILTER_ALPHA * filtered_mode + (1.0 - FILTER_ALPHA) * mode;

    check_validity();

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }

    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
    {
        enter_offboard_stabilized_mode = true;
    }
    else
    {
        enter_offboard_stabilized_mode = false;
    }

    is_offboard_stabilized_mode = mode > API_MODE_THRESHOLD_VALUE;

    if (mode > API_MODE_THRESHOLD_VALUE &&
        ((last_gear <= HOVER_THRESHOLD && gear > HOVER_THRESHOLD && gear < CMD_THRESHOLD) ||
         (last_gear >= CMD_THRESHOLD && gear > HOVER_THRESHOLD && gear < CMD_THRESHOLD)))
    {
        enter_hover_mode = true;
    }
    else
    {
        enter_hover_mode = false;
    }

    is_hover_mode = mode > API_MODE_THRESHOLD_VALUE &&
                    gear > HOVER_THRESHOLD &&
                    gear < CMD_THRESHOLD;

    if (mode > API_MODE_THRESHOLD_VALUE && last_gear < CMD_THRESHOLD && gear >= CMD_THRESHOLD)
    {
        enter_command_mode = true;
    }
    else
    {
        enter_command_mode = false;
    }

    is_command_mode = mode > API_MODE_THRESHOLD_VALUE && gear >= CMD_THRESHOLD;

    if (mode <= API_MODE_THRESHOLD_VALUE)
    {
        is_offboard_stabilized_mode = false;
        is_hover_mode = false;
        is_command_mode = false;
        enter_offboard_stabilized_mode = false;
        enter_hover_mode = false;
        enter_command_mode = false;
    }

    last_mode = mode;
    last_gear = gear;
}

void RC_Receiver::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1)
    {
        return;
    }

    // Invalid readings currently ignored (kept for future diagnostics)
}

bool RC_Receiver::check_centered()
{
    const double centered_threshold = 0.05;
    return std::abs(ch[0]) < centered_threshold &&
           std::abs(ch[1]) < centered_threshold &&
           std::abs(ch[2]) < centered_threshold &&
           std::abs(ch[3]) < centered_threshold;
}

bool RC_Receiver::is_received(const rclcpp::Time &now_time)
{
    constexpr double timeout = 0.5; // seconds
    return (now_time - rcv_stamp).seconds() <= timeout;
}

double RC_Receiver::getDesiredThrust()
{
    // 遥控器油门通道值范围：-1.0 到 1.0
    // 推力输出范围：-0.0 到 -0.9 (负值表示向上推力)
    const double THRUST_MIN = -0.9;  // 最大推力(负值表示向上)
    const double THRUST_MAX = -0.0;  // 最小推力(负值表示向上)
    const double RC_MIN = -1.0;      // 遥控器最小值
    const double RC_MAX = 1.0;       // 遥控器最大值
    
    // 获取油门通道值(通常是ch[2])
    double throttle_input = ch[2];
    
    // 限制输入范围在[-1, 1]
    throttle_input = std::max(RC_MIN, std::min(RC_MAX, throttle_input));
    
    // 线性映射：y = a*x + b
    // 当 throttle_input = -1.0 时，thrust = -0.0 (最小推力)
    // 当 throttle_input = 1.0 时，thrust = -0.9 (最大推力)
    double thrust_range = THRUST_MIN - THRUST_MAX;  // -0.9 - (-0.0) = -0.9
    double rc_range = RC_MAX - RC_MIN;              // 1.0 - (-1.0) = 2.0
    
    // 映射公式：thrust = THRUST_MAX + (throttle_input - RC_MIN) * thrust_range / rc_range
    double desired_thrust = THRUST_MAX + (throttle_input - RC_MIN) * thrust_range / rc_range;
    
    // 安全限制：确保推力在允许范围内
    desired_thrust = std::max(THRUST_MIN, std::min(THRUST_MAX, desired_thrust));
    
    return desired_thrust;
}

fsmpx4::types::Matrix3 RC_Receiver::getDesiredRotationMatrix()
{
    const double MAX_ATTITUDE_ANGLE = 0.5;  // 最大姿态角 (约30度)
    const double MAX_MANUAL_YAW_RATE = 1.0; // 最大偏航角速度 (rad/s)
    
    // 从遥控器输入计算期望姿态角（限制范围）
    double desired_roll = std::clamp(ch[0] * MAX_ATTITUDE_ANGLE,
                                     -MAX_ATTITUDE_ANGLE,
                                     MAX_ATTITUDE_ANGLE);
    double desired_pitch = std::clamp(-ch[1] * MAX_ATTITUDE_ANGLE,
                                      -MAX_ATTITUDE_ANGLE,
                                      MAX_ATTITUDE_ANGLE);
    
    auto wrap_pi = [](double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    };

    const double yaw_input = std::clamp(ch[3], -1.0, 1.0);
    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    double dt = (now - last_yaw_update_stamp_).seconds();
    last_yaw_update_stamp_ = now;

    if (!std::isfinite(dt) || dt < 0.0) {
        dt = 0.0;
    } else if (dt > 0.5) {
        // 防止长时间未更新时积分过大
        dt = 0.5;
    }

    double current_yaw = manual_yaw_sp_;
    bool attitude_ready = false;
    if (uav_state_ptr != nullptr) {
        current_yaw = uav_state_ptr->rotation.eulerAngles(2, 1, 0)(0);
        attitude_ready = uav_state_ptr->timestamp > 0.0;
    }

    const bool reset_yaw_sp = (ch[2] < -0.9);
    if ((reset_yaw_sp || !yaw_sp_initialized_) && attitude_ready) {
        manual_yaw_sp_ = current_yaw;
        yaw_sp_initialized_ = true;
    } else if (!attitude_ready) {
        yaw_sp_initialized_ = false;
    } else if (dt > 0.0) {
        const double yaw_rate = yaw_input * MAX_MANUAL_YAW_RATE;
        manual_yaw_sp_ = wrap_pi(manual_yaw_sp_ + yaw_rate * dt);
    }

    const double desired_yaw = manual_yaw_sp_;

    // 使用 Eigen 计算旋转矩阵
    fsmpx4::types::Matrix3 roll_matrix, pitch_matrix, yaw_matrix;
    
    roll_matrix = Eigen::AngleAxisd(desired_roll, fsmpx4::types::Vector3::UnitX()).toRotationMatrix();
    pitch_matrix = Eigen::AngleAxisd(desired_pitch, fsmpx4::types::Vector3::UnitY()).toRotationMatrix();
    yaw_matrix = Eigen::AngleAxisd(desired_yaw, fsmpx4::types::Vector3::UnitZ()).toRotationMatrix();

    return yaw_matrix * pitch_matrix * roll_matrix;
}

void RC_Receiver::setUAVStatePtr(fsmpx4::types::UAVState* ptr)
{
    uav_state_ptr = ptr;
}

double RC_Receiver::getManualYawSetpoint() const
{
    return manual_yaw_sp_;
}

//==============================================================================
// IMU_Reader Implementation
//==============================================================================

IMU_Reader::IMU_Reader()
    : uav_state_ptr(nullptr)
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
}

void IMU_Reader::feedAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr pMsg)
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    if (uav_state_ptr != nullptr)
    {
        const Eigen::Quaterniond q(pMsg->q[0], pMsg->q[1], pMsg->q[2], pMsg->q[3]);
        uav_state_ptr->rotation = q.toRotationMatrix();
        uav_state_ptr->timestamp = rcv_stamp.seconds();
    }
}

void IMU_Reader::feedAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr pMsg)
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    if (uav_state_ptr != nullptr)
    {
        uav_state_ptr->angular_velocity << pMsg->xyz[0], pMsg->xyz[1], pMsg->xyz[2];
        uav_state_ptr->timestamp = rcv_stamp.seconds();
    }
}

void IMU_Reader::setUAVStatePtr(fsmpx4::types::UAVState* ptr)
{
    uav_state_ptr = ptr;
}

bool IMU_Reader::is_received(const rclcpp::Time &now_time)
{
    constexpr double timeout = 0.2; // seconds
    return (now_time - rcv_stamp).seconds() <= timeout;
}

//==============================================================================
// Position_Reader Implementation
//==============================================================================

Position_Reader::Position_Reader()
    : uav_state_ptr(nullptr),
      xy_valid(false),
      z_valid(false),
      v_xy_valid(false),
      v_z_valid(false)
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
}

void Position_Reader::feed(const px4_msgs::msg::VehicleLocalPosition::SharedPtr pMsg)
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    xy_valid = pMsg->xy_valid;
    z_valid = pMsg->z_valid;
    v_xy_valid = pMsg->v_xy_valid;
    v_z_valid = pMsg->v_z_valid;

    if (uav_state_ptr == nullptr)
    {
        return;
    }

    if (xy_valid && z_valid)
    {
        uav_state_ptr->position << pMsg->x, pMsg->y, pMsg->z;
    }

    if (v_xy_valid && v_z_valid)
    {
        uav_state_ptr->velocity << pMsg->vx, pMsg->vy, pMsg->vz;
    }

    uav_state_ptr->timestamp = rcv_stamp.seconds();
}

void Position_Reader::setUAVStatePtr(fsmpx4::types::UAVState* ptr)
{
    uav_state_ptr = ptr;
}

bool Position_Reader::is_received(const rclcpp::Time &now_time)
{
    constexpr double timeout = 0.5; // seconds
    return (now_time - rcv_stamp).seconds() <= timeout;
}

//==============================================================================
// HoverThrust_Reader Implementation
//==============================================================================

HoverThrust_Reader::HoverThrust_Reader()
    : hover_thrust(0.0f),
      valid(false),
      uav_state_ptr(nullptr),
      default_hover_thrust_(-0.5)  // 默认悬停推力值
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
}

void HoverThrust_Reader::feed(const px4_msgs::msg::HoverThrustEstimate::SharedPtr pMsg)
{
    rcv_stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    hover_thrust = pMsg->hover_thrust;
    valid = pMsg->valid;

    if (uav_state_ptr != nullptr && valid)
    {
        uav_state_ptr->hover_thrust = static_cast<double>(hover_thrust);
        uav_state_ptr->timestamp = rcv_stamp.seconds();
    }
    else if (uav_state_ptr != nullptr && !valid)
    {
        // 如果数据无效，使用可配置的默认值
        uav_state_ptr->hover_thrust = default_hover_thrust_;
    }
}

void HoverThrust_Reader::setUAVStatePtr(fsmpx4::types::UAVState* ptr)
{
    uav_state_ptr = ptr;
}

void HoverThrust_Reader::setDefaultHoverThrust(double default_value)
{
    default_hover_thrust_ = default_value;
    hover_thrust = static_cast<float>(default_hover_thrust_);

    if (uav_state_ptr != nullptr)
    {
        uav_state_ptr->hover_thrust = default_hover_thrust_;
    }
}

bool HoverThrust_Reader::is_received(const rclcpp::Time &now_time)
{
    constexpr double timeout = 1.0; // seconds
    if (!valid)
    {
        return false;
    }
    return (now_time - rcv_stamp).seconds() <= timeout;
}

} // namespace input
} // namespace fsmpx4
