#ifndef OFFBOARD_PX4_H
#define OFFBOARD_PX4_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
 	
	OffboardControl();
	void arm();
	void disarm();
	void process();

private:
	enum class FlightState {
		INIT,
        SENDING_CONTROL,
        REQUESTING_OFFBOARD,
        // WAITING_OFFBOARD_CONFIRM,
        ARMING,
        WAITING_ARM_CONFIRM,
        OFFBOARD_ACTIVE,
		RETURN_REQUESTED,//返航
        ERROR
    };
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr   offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr    trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr        vehicle_command_publisher_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr      vehicle_status_subscriber_;
	rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
	rclcpp::Subscription<BatteryStatus>::SharedPtr      battery_status_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    bool offboard_requested_;    // 是否已请求offboard
	FlightState current_state_;
    rclcpp::Time offboard_request_time_;
    const rclcpp::Duration offboard_timeout_{2s}; 
	// PX4状态信息（通过回调更新）
    std::atomic<uint8_t> current_nav_state_{0};
    std::atomic<uint8_t> current_arming_state_{0};
    std::atomic<uint64_t> last_status_timestamp_{0};

	// std::atomic<int> current_nav_state_{0};
    std::atomic<bool> control_offboard_enabled_{false};
    // std::atomic<int> current_arming_state_{0};
    std::atomic<bool> is_fully_offboard_{false};
    std::atomic<bool> is_offboard_transitioning_{false};

	std::atomic<float> remaining_{1.0f};           // 剩余电量 (0.0-1.0)
    std::atomic<bool> connected_{false};           // 电池是否连接
    std::atomic<uint8_t> warning_{0};              // PX4警告级别
    std::atomic<bool> safe_to_fly_{false};         // 是否可以起飞
	std::atomic<bool> last_safe_state_{false};      // 上次安全状态（用于检测变化）

    rclcpp::Time state_start_time_;

    const rclcpp::Duration arm_timeout_{2s};

	//返航
	bool rtl_command_sent_{false};
	rclcpp::Time rtl_start_time_;
	rclcpp::Duration rtl_command_duration_{1s};  // 1 秒足够

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y,float z, float yaw);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void initializePublishers();
	void initializeSubscribers();

	void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
	void vehicle_control_mode_callback(const VehicleControlMode::SharedPtr msg);
	void battery_status_callback(const BatteryStatus::SharedPtr msg);
	void update_state(FlightState new_state);
	bool check_offboard_mode();
    bool check_armed_state();
	void monitor_safety(uint8_t nav_state, uint8_t arming_state);
	bool check_pre_arm_conditions();
	void handle_error_state(uint8_t nav_state, uint8_t arming_state);
	void check_battery_safety();
    void return_flight(uint8_t nav_current_state);
};

#endif // OFFBOARD_PX4_H


