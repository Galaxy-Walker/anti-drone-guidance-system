#include "input.h"

#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace fsmpx4 {
namespace debug {

using namespace std::chrono_literals;

class InputDebugNode : public rclcpp::Node
{
public:
    InputDebugNode()
        : rclcpp::Node("fsmpx4_input_debug_node")
    {
        // Declare parameters so the user can remap topics without recompiling.
        manual_control_topic_ = declare_parameter<std::string>(
            "manual_control_topic", "/rc/manual_control_setpoint");
        attitude_topic_ = declare_parameter<std::string>(
            "attitude_topic", "/fmu/out/vehicle_attitude");
        local_position_topic_ = declare_parameter<std::string>(
            "local_position_topic", "/fmu/out/vehicle_local_position");
        hover_thrust_topic_ = declare_parameter<std::string>(
            "hover_thrust_topic", "/fmu/out/hover_thrust_estimate");
        
        // 声明默认悬停推力参数
        double default_hover_thrust = declare_parameter<double>(
            "default_hover_thrust", -0.5);

        imu_reader_.setUAVStatePtr(&state_);
        position_reader_.setUAVStatePtr(&state_);
        hover_reader_.setUAVStatePtr(&state_);
        hover_reader_.setDefaultHoverThrust(default_hover_thrust);  // 设置默认悬停推力

        manual_control_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
            manual_control_topic_, rclcpp::SensorDataQoS(),
            std::bind(&InputDebugNode::onManualControl, this, std::placeholders::_1));

        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            attitude_topic_, rclcpp::SensorDataQoS(),
            std::bind(&InputDebugNode::onAttitude, this, std::placeholders::_1));

        local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            local_position_topic_, rclcpp::SensorDataQoS(),
            std::bind(&InputDebugNode::onLocalPosition, this, std::placeholders::_1));

        hover_thrust_sub_ = create_subscription<px4_msgs::msg::HoverThrustEstimate>(
            hover_thrust_topic_, rclcpp::SensorDataQoS(),
            std::bind(&InputDebugNode::onHoverThrust, this, std::placeholders::_1));

        heartbeat_timer_ = create_wall_timer(1s, [this]() { logState("timer"); });

        RCLCPP_INFO(get_logger(),
                    "Listening to PX4 topics:\n  manual_control: %s\n  attitude: %s\n  local_position: %s\n  hover_thrust: %s",
                    manual_control_topic_.c_str(), attitude_topic_.c_str(),
                    local_position_topic_.c_str(), hover_thrust_topic_.c_str());

        logState("startup");
    }

private:
    void onManualControl(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
    {
        rc_receiver_.feed(msg);

        RCLCPP_INFO(get_logger(),
                    "RC update -> mode: %.2f gear: %.2f | hover:%d cmd:%d stabilized:%d",
                    rc_receiver_.mode, rc_receiver_.gear,
                    rc_receiver_.is_hover_mode, rc_receiver_.is_command_mode,
                    rc_receiver_.is_offboard_stabilized_mode);
        RCLCPP_INFO(get_logger(),
                    "  flags: enter_cmd:%d enter_hover:%d enter_stabilized:%d",
                    rc_receiver_.enter_command_mode,
                    rc_receiver_.enter_hover_mode,
                    rc_receiver_.enter_offboard_stabilized_mode);
        RCLCPP_INFO(get_logger(),
                    "  channels: roll=%.2f pitch=%.2f yaw=%.2f thr=%.2f",
                    rc_receiver_.ch[0], rc_receiver_.ch[1], rc_receiver_.ch[3], rc_receiver_.ch[2]);

        logState("manual_control");
    }

    void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        imu_reader_.feedAttitude(msg);
        logState("attitude");
    }

    void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        position_reader_.feed(msg);
        RCLCPP_INFO(get_logger(),
                    "Local position flags -> pos:%d vel:%d",
                    position_reader_.isPositionValid(), position_reader_.isVelocityValid());
        logState("local_position");
    }

    void onHoverThrust(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg)
    {
        hover_reader_.feed(msg);
        RCLCPP_INFO(get_logger(), "Hover thrust update -> value: %.3f valid:%d",
                    hover_reader_.hover_thrust, hover_reader_.valid);
        logState("hover_thrust");
    }

    std::string formatVector(const Eigen::Vector3d &vec) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << '[' << vec.x() << ", " << vec.y() << ", " << vec.z() << ']';
        return oss.str();
    }

    std::string formatEuler(const Eigen::Matrix3d &rot) const
    {
        const Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "[yaw: " << euler[0] << ", pitch: " << euler[1]
            << ", roll: " << euler[2] << ']';
        return oss.str();
    }

    void logState(const std::string &source)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "UAVState snapshot from " << source << '\n';
        oss << "  position       = " << formatVector(state_.position) << '\n';
        oss << "  velocity       = " << formatVector(state_.velocity) << '\n';
        oss << "  angular_vel    = " << formatVector(state_.angular_velocity) << '\n';
        oss << "  hover_thrust   = " << state_.hover_thrust << '\n';
        oss << "  timestamp      = " << state_.timestamp << '\n';
        oss << "  attitude (YPR) = " << formatEuler(state_.rotation) << '\n';
        oss << "  rc flags       = hover:" << rc_receiver_.is_hover_mode
            << " cmd:" << rc_receiver_.is_command_mode
            << " stabilized:" << rc_receiver_.is_offboard_stabilized_mode << '\n';
        oss << "  rc transitions = enter_cmd:" << rc_receiver_.enter_command_mode
            << " enter_hover:" << rc_receiver_.enter_hover_mode
            << " enter_stabilized:" << rc_receiver_.enter_offboard_stabilized_mode;

        RCLCPP_INFO_STREAM(get_logger(), oss.str());
    }

    // Topic names (configurable via parameters)
    std::string manual_control_topic_;
    std::string attitude_topic_;
    std::string local_position_topic_;
    std::string hover_thrust_topic_;

    // Data handlers
    fsmpx4::input::RC_Receiver rc_receiver_;
    fsmpx4::input::IMU_Reader imu_reader_;
    fsmpx4::input::Position_Reader position_reader_;
    fsmpx4::input::HoverThrust_Reader hover_reader_;

    fsmpx4::types::UAVState state_;

    // Subscriptions
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace debug
}  // namespace fsmpx4

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fsmpx4::debug::InputDebugNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
