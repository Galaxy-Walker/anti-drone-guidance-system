#ifndef FSMPX4_FSM_H
#define FSMPX4_FSM_H

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <fsmpx4/msg/fsm_debug.hpp>

#include "control.h"
#include "input.h"
#include "param_loader.h"
#include "types.h"

namespace fsmpx4
{

class FSMPX4 : public rclcpp::Node
{
public:
    explicit FSMPX4(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // 有限状态机的全部运行状态
    enum class State
    {
        MANUAL_CTRL,          // 纯手动控制
        OFFBOARD_STABILIZED,  // 外部姿态控制（稳定）
        AUTO_HOVER,           // 自动悬停
        CMD_CTRL              // 自动指令跟踪
    };

    // 周期性调度入口，由 wall timer 调用
    void process();

private:
    // 初始化发布器/订阅器/控制循环
    void initializePublishers();
    void initializeSubscribers();
    void ControlLoop(double frequency_hz);

    // 各状态处理函数，
    void handleManual(const rclcpp::Time& now);
    void handleOffboardStabilized(const rclcpp::Time& now);
    void handleAutoHover(const rclcpp::Time& now);
    void handleCommandControl(const rclcpp::Time& now);

    // 状态切换辅助：打印并保存上一次状态
    void enterState(State next_state);

    // 输入有效性检查
    bool imuReady(const rclcpp::Time& now);
    bool positionReady(const rclcpp::Time& now);

    // PX4 offboard 模式相关发布接口
    void publishOffboardMode(bool use_attitude);
    void publishAttitudeCommand(const types::Matrix3& attitude, double thrust, const rclcpp::Time& stamp);
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    bool toggleOffboardMode(bool on_off);
    void logLoadedParams() const;
    void publishDebugMessage(const rclcpp::Time& stamp);
    
    // 数据打印函数
    void printUAVAttitude();
    void printControlData();
    
    // 回调函数
    void onVehicleRatesSetpoint(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg);
    void onVehicleThrustSetpoint(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg);
    void onVehicleTorqueSetpoint(const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr msg);

    param_loader::FSMParams params_;  // YAML 载入的参数快照

    State state_;
    State previous_state_;
    bool target_initialized_;  // 指令模式下的目标位姿是否已锁定
    types::Vector3 auto_hover_position_target_;  // 自动悬停目标位置
    double auto_hover_yaw_target_;               // 自动悬停目标偏航角
    types::Vector3 auto_cmd_position_target_;    // 自动指令模式目标位置
    double auto_cmd_yaw_target_;                 // 自动指令模式目标偏航角

    control::PositionAttitudeController controller_;
    std::shared_ptr<types::UAVState> current_state_;
    types::UAVState* uav_state_;  // 直接指向 UAVState 的指针，避免重复调用 get()
    types::ControlOutput output_;
    types::UAVCommand cmd_;
    
    // Vehicle Rates Setpoint 数据
    types::Vector3 wd_;  // 角速度设定值 [roll_rate, pitch_rate, yaw_rate]
    
    // Vehicle Thrust 和 Torque Setpoint 数据
    types::Vector3 thrust_;  // 推力设定值 [thrust_x, thrust_y, thrust_z]
    types::Vector3 torque_;  // 力矩设定值 [torque_x, torque_y, torque_z]

    // 原始传感器/遥控输入模块
    input::RC_Receiver rc_input_;
    input::IMU_Reader imu_input_;
    input::Position_Reader position_input_;
    input::HoverThrust_Reader hover_input_;

    // PX4 offboard 输出
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
    rclcpp::Publisher<fsmpx4::msg::FSMDebug>::SharedPtr debug_pub_;

    // 订阅器：获取遥控、姿态、角速度、位置以及悬停推力估计
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr rc_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // Offboard 模式切换辅助，复用 PX4CtrlFSM 的时间窗口逻辑
    bool offboard_toggle_active_{false};
    bool offboard_last_command_state_{false};
    bool offboard_has_last_command_{false};
    rclcpp::Time offboard_toggle_start_time_{};
    rclcpp::Duration offboard_toggle_duration_{rclcpp::Duration::from_seconds(0.5)};
};

} // namespace fsmpx4

#endif // FSMPX4_FSM_H
