#ifndef __FSMPX4_INPUT_H
#define __FSMPX4_INPUT_H

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>

#include "types.h"  // 包含所有数据类型定义

namespace fsmpx4 {
namespace input {

//=============================================================================
// 数据接收层 (Input Layer)
//=============================================================================

/**
 * Module 1: RC_Receiver - 遥控器数据接收
 * 负责接收和处理遥控器输入信号
 */
class RC_Receiver
{
public:
    // 遥控器通道数据
    double ch[4];                   // 通道值 [roll, pitch, yaw, throttle]
    double mode;                    // 模式开关
    double gear;                    // 档位开关
    double last_mode;               // 上一次模式值
    double last_gear;               // 上一次档位值
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};
    
    // 信号滤波
    double filtered_mode{0.0};
    static constexpr double FILTER_ALPHA = 0.8;
    
    // 原始消息和时间戳
    px4_msgs::msg::ManualControlSetpoint msg;
    rclcpp::Time rcv_stamp;
    
    // 模式判断相关
    bool is_command_mode;
    bool enter_command_mode;
    bool is_hover_mode;
    bool enter_hover_mode;
    bool is_offboard_stabilized_mode;
    bool enter_offboard_stabilized_mode;
    
    // 阈值常量
    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
    static constexpr double API_MODE_HYSTERESIS = 0.15;
    static constexpr double CMD_MODE_THRESHOLD = 0.33;
    static constexpr double HOVER_THRESHOLD = 0.45;
    static constexpr double CMD_THRESHOLD = 0.75;
    static constexpr double DEAD_ZONE = 0.1;
    
    // 构造函数
    RC_Receiver();
    
    // 核心方法
    void feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr pMsg);
    void check_validity();
    bool check_centered();
    bool is_received(const rclcpp::Time &now_time);
    
    // 遥控器输出计算
    double getDesiredThrust();
    fsmpx4::types::Matrix3 getDesiredRotationMatrix();
    void setUAVStatePtr(fsmpx4::types::UAVState* ptr);
    double getManualYawSetpoint() const;

private:
    // 偏航角控制相关
    double manual_yaw_sp_{0.0};
    bool yaw_sp_initialized_{false};
    rclcpp::Time last_yaw_update_stamp_;
    fsmpx4::types::UAVState* uav_state_ptr{nullptr};
};

/**
 * Module 2: IMU_Reader - IMU数据接收
 * 负责接收和处理IMU传感器数据
 */
class IMU_Reader
{
public:
    rclcpp::Time rcv_stamp;
    fsmpx4::types::UAVState* uav_state_ptr;        // UAV状态指针
    
    // 构造函数
    IMU_Reader();
    
    // 核心方法
    void feedAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr pMsg);
    void feedAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr pMsg);
    void setUAVStatePtr(fsmpx4::types::UAVState* ptr);
    bool is_received(const rclcpp::Time &now_time);
};

/**
 * Module 3: Position_Reader - 定位数据接收
 * 负责接收和处理位置定位数据
 */
class Position_Reader
{
public:
    rclcpp::Time rcv_stamp;
    fsmpx4::types::UAVState* uav_state_ptr;        // UAV状态指针
    
    // 数据有效性标志
    bool xy_valid;
    bool z_valid;
    bool v_xy_valid;
    bool v_z_valid;
    
    // 构造函数
    Position_Reader();
    
    // 核心方法
    void feed(const px4_msgs::msg::VehicleLocalPosition::SharedPtr pMsg);
    void setUAVStatePtr(fsmpx4::types::UAVState* ptr);
    bool isPositionValid() const { return xy_valid && z_valid; }
    bool isVelocityValid() const { return v_xy_valid && v_z_valid; }
    bool is_received(const rclcpp::Time &now_time);
};

/**
 * 辅助模块：HoverThrust_Reader - 悬停推力估计接收
 * 负责接收悬停推力估计数据
 */
class HoverThrust_Reader
{
public:
    float hover_thrust;
    bool valid;
    rclcpp::Time rcv_stamp;
    fsmpx4::types::UAVState* uav_state_ptr;
    
    // 构造函数
    HoverThrust_Reader();
    
    // 核心方法
    void feed(const px4_msgs::msg::HoverThrustEstimate::SharedPtr pMsg);
    void setUAVStatePtr(fsmpx4::types::UAVState* ptr);
    void setDefaultHoverThrust(double default_value);  // 设置默认悬停推力值
    bool is_received(const rclcpp::Time &now_time);

private:
    double default_hover_thrust_;  // 默认悬停推力值
};


} // namespace input
} // namespace fsmpx4

#endif // __FSMPX4_INPUT_H
