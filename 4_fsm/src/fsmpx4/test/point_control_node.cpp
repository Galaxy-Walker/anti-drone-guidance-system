#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

#include "input.h"
#include "control.h"
#include "param_loader.h"
#include "types.h"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

namespace fsmpx4 {
namespace point_control {

using namespace std::chrono_literals;
using fsmpx4::control::PositionAttitudeController;
using fsmpx4::param_loader::FSMParams;
using fsmpx4::types::ControlOutput;
using fsmpx4::types::UAVCommand;
using fsmpx4::types::UAVState;
using fsmpx4::types::Vector3;

class PointControlNode : public rclcpp::Node
{
public:
    PointControlNode()
        : rclcpp::Node("px4ctrl_fsm"),
          last_status_log_(this->get_clock()->now())
    {
        position_reader_.setUAVStatePtr(&uav_state_);
        imu_reader_.setUAVStatePtr(&uav_state_);
        hover_reader_.setUAVStatePtr(&uav_state_);

        initializePublishers();
        initializeSubscribers();
    }

    bool initialize()
    {
        if (!params_.load_from_ros_node(shared_from_this())) {
            RCLCPP_FATAL(get_logger(), "Failed to load FSM parameters");
            return false;
        }

        thrust_min_ = params_.limits.min_thrust;
        thrust_max_ = params_.limits.max_thrust;
        control_frequency_hz_ = params_.basic.ctrl_freq_max;
        hover_reader_.setDefaultHoverThrust(params_.thr_map.hover_percentage);
        uav_state_.hover_thrust = params_.thr_map.hover_percentage;

        controller_cfg_.load_from_params(params_);

        if (!controller_.initialize(controller_cfg_)) {
            RCLCPP_FATAL(get_logger(), "Controller failed to initialize");
            return false;
        }

        controller_ready_ = true;

        if (control_frequency_hz_ <= 0.0) {
            control_frequency_hz_ = 50.0;
            RCLCPP_WARN(get_logger(), "Invalid control_frequency_hz. Falling back to %.1f Hz",
                        control_frequency_hz_);
        }

        const int timer_period_ms = static_cast<int>(std::round(1000.0 / control_frequency_hz_));
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&PointControlNode::controlLoop, this));

        RCLCPP_INFO(get_logger(),
                    "Controller: Kp=(%.2f, %.2f, %.2f) Kv=(%.2f, %.2f, %.2f) mass=%.2f",
                    controller_cfg_.kp.x(), controller_cfg_.kp.y(), controller_cfg_.kp.z(),
                    controller_cfg_.kv.x(), controller_cfg_.kv.y(), controller_cfg_.kv.z(),
                    controller_cfg_.mass);

        return true;
    }

private:
    static double degToRad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    void initializePublishers()
    {
        auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                            .durability(rclcpp::DurabilityPolicy::Volatile);

        offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", px4_qos);
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", px4_qos);
        attitude_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", px4_qos);
    }

    void initializeSubscribers()
    {
        auto sensor_qos = rclcpp::SensorDataQoS();

        manual_control_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
            "/rc/manual_control_setpoint", sensor_qos,
            std::bind(&PointControlNode::onManualControl, this, std::placeholders::_1));

        position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&PointControlNode::onLocalPosition, this, std::placeholders::_1));

        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", sensor_qos,
            std::bind(&PointControlNode::onAttitude, this, std::placeholders::_1));

        angular_velocity_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", sensor_qos,
            std::bind(&PointControlNode::onAngularVelocity, this, std::placeholders::_1));

        hover_thrust_sub_ = create_subscription<px4_msgs::msg::HoverThrustEstimate>(
            "/fmu/out/hover_thrust_estimate", sensor_qos,
            std::bind(&PointControlNode::onHoverThrust, this, std::placeholders::_1));
    }

    void onManualControl(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
    {
        rc_receiver_.feed(msg);
        manual_control_received_ = true;

    }

    void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        position_reader_.feed(msg);
        have_position_ = position_reader_.isPositionValid();
    }

    void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        imu_reader_.feedAttitude(msg);
        have_attitude_ = true;
    }

    void onAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
    {
        imu_reader_.feedAngularVelocity(msg);
        have_angular_velocity_ = true;
    }

    void onHoverThrust(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg)
    {
        hover_reader_.feed(msg);
    }

    bool sensorsReady(const rclcpp::Time& now)
    {
        return have_position_ && have_attitude_ && have_angular_velocity_ &&
               position_reader_.is_received(now) && imu_reader_.is_received(now);
    }

    void controlLoop()
    {
        if (!controller_ready_) {
            return;
        }

        const auto now = this->get_clock()->now();
        publishOffboardMode(now);

        if (!sensorsReady(now)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for sensors - pos:%d att:%d ang:%d",
                                 have_position_, have_attitude_, have_angular_velocity_);
            return;
        }

        if (!manual_control_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for manual control input on /fmu/out/manual_control_setpoint");
            return;
        }

        uav_state_.timestamp = now.seconds();

        if (!control_active_) {
            control_active_ = true;
            publishArmCommand(now);
            publishOffboardModeCommand(now);
            RCLCPP_INFO(get_logger(), "Offboard control activated.");
        }

        UAVCommand command;
        command.position = Vector3(4.0, 4.0, -2.0);
        command.velocity = Vector3::Zero();
        command.acceleration = Vector3::Zero();
        command.yaw_desired = 0.0;
        command.b1d = Vector3::UnitX();
        command.timestamp = now.seconds();

        const ControlOutput output = controller_.computeControl(uav_state_, command);
        if (!output.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Controller produced invalid output (zero thrust vector)");
            return;
        }

        const double normalized_thrust = computeNormalizedThrust(output);

        publishAttitudeSetpoint(now, output, normalized_thrust);
        maybeLogStatus(now, output);
    }

    void publishOffboardMode(const rclcpp::Time& stamp)
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = stamp.nanoseconds() / 1000;
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;
        offboard_mode_pub_->publish(msg);
    }

    void publishAttitudeSetpoint(const rclcpp::Time& stamp, const ControlOutput& output,
                                 double normalized_thrust)
    {
        px4_msgs::msg::VehicleAttitudeSetpoint msg{};
        msg.timestamp = stamp.nanoseconds() / 1000;
        msg.q_d[0] = static_cast<float>(output.qd.w());
        msg.q_d[1] = static_cast<float>(output.qd.x());
        msg.q_d[2] = static_cast<float>(output.qd.y());
        msg.q_d[3] = static_cast<float>(output.qd.z());
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = static_cast<float>(normalized_thrust);
        attitude_setpoint_pub_->publish(msg);
    }

    void publishArmCommand(const rclcpp::Time& stamp)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = stamp.nanoseconds() / 1000;
        msg.param1 = 1.0f;  // arm
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void publishOffboardModeCommand(const rclcpp::Time& stamp)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = stamp.nanoseconds() / 1000;
        msg.param1 = 1.0f;
        msg.param2 = 6.0f;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    double computeNormalizedThrust(const ControlOutput& output) const
    {
        return std::clamp(output.thrust, thrust_min_, thrust_max_);
    }

    void maybeLogStatus(const rclcpp::Time& stamp, const ControlOutput& output)
    {
        if ((stamp - last_status_log_).seconds() < 1.0) {
            return;
        }

        const double distance = uav_state_.position.norm();
        const double speed = uav_state_.velocity.norm();

        RCLCPP_INFO(get_logger(),
                    "[PointCtrl] pos=[%.2f, %.2f, %.2f] err=%.3f vel=%.2f thrust=%.3f",
                    uav_state_.position.x(), uav_state_.position.y(), uav_state_.position.z(),
                    distance, speed, computeNormalizedThrust(output));

        last_status_log_ = stamp;
    }

    FSMParams params_;

    // 状态 & 控制器
    UAVState uav_state_;
    PositionAttitudeController controller_;
    PositionAttitudeController::Config controller_cfg_;

    double thrust_min_;
    double thrust_max_;
    double control_frequency_hz_;

    bool controller_ready_{false};
    bool have_position_{false};
    bool have_attitude_{false};
    bool have_angular_velocity_{false};
    bool control_active_{false};

    // 输入层
    fsmpx4::input::Position_Reader position_reader_;
    fsmpx4::input::IMU_Reader imu_reader_;
    fsmpx4::input::HoverThrust_Reader hover_reader_;
    fsmpx4::input::RC_Receiver rc_receiver_;

    // ROS2 interfaces
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool manual_control_received_{false};
    rclcpp::Time last_status_log_;
};

}  // namespace point_control
}  // namespace fsmpx4

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fsmpx4::point_control::PointControlNode>();
    if (!node->initialize()) {
        RCLCPP_FATAL(node->get_logger(), "Point control node failed to initialize");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
