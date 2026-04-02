#include "pixhawk/offboard_px4.hpp"
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

OffboardControl::OffboardControl() : Node("offboard_control")
{
	RCLCPP_INFO(this->get_logger(), "px4_node节点初始化成功");
    initializePublishers();
    initializeSubscribers();
    timer_=this->create_wall_timer(100ms,std::bind(&OffboardControl::process, this));
}

void OffboardControl::initializePublishers()   //发布话题
{
	offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    RCLCPP_INFO(this->get_logger(), "offboard_control_mode话题创建成功");
	trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    RCLCPP_INFO(this->get_logger(), "trajectory_setpoint话题创建成功");
	vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    RCLCPP_INFO(this->get_logger(), "vehicle_command话题创建成功");

}
void OffboardControl::initializeSubscribers()   //订阅话题
{
    // 完全匹配PX4的QoS设置
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))  // 通常深度是10或更多
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)      // BEST_EFFORT
        .durability(rclcpp::DurabilityPolicy::TransientLocal)    // TRANSIENT_LOCAL
        .history(rclcpp::HistoryPolicy::KeepLast);               // KEEP_LAST

    vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>(   //“我打算进入 某种模式Offboard / RTL / Mission”
    "/fmu/out/vehicle_status_v1", qos,
    std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "vehicle_status_v1话题订阅成功");

    vehicle_control_mode_subscriber_ = this->create_subscription<VehicleControlMode>(//“我是否真的允许你通过这个控制器控制我”
    "/fmu/out/vehicle_control_mode", qos,
    std::bind(&OffboardControl::vehicle_control_mode_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "vehicle_control_mode话题订阅成功");

    battery_status_subscriber_ = this->create_subscription<BatteryStatus>(
    "/fmu/out/battery_status",  qos,
    std::bind(&OffboardControl::battery_status_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "battery_status话题订阅成功");
}

void OffboardControl::vehicle_status_callback(const VehicleStatus::SharedPtr msg) {
    // 更新状态信息
    // RCLCPP_INFO(this->get_logger(),
    //     "收到VehicleStatus: timestamp=%lu, nav_state=%d",
    //     msg->timestamp, msg->nav_state);
    current_nav_state_.store(msg->nav_state, std::memory_order_relaxed);
    current_arming_state_.store(msg->arming_state, std::memory_order_relaxed);
    last_status_timestamp_.store(msg->timestamp, std::memory_order_relaxed);

}

void OffboardControl::battery_status_callback(const  BatteryStatus::SharedPtr msg) {
    connected_.store(msg->connected, std::memory_order_relaxed);
    remaining_.store(msg->remaining, std::memory_order_relaxed);
    warning_.store(msg->warning, std::memory_order_relaxed);
}
void OffboardControl::check_battery_safety() {
        // 1. 读取原子变量（使用acquire保证看到最新值）
        bool connected = connected_.load(std::memory_order_relaxed);
        float remaining = remaining_.load(std::memory_order_relaxed);//考虑下用memory_order_acquire
        uint8_t warning = warning_.load(std::memory_order_relaxed);

        // 2. 安全检查逻辑
        bool is_safe = true;

        if (!connected) {
            is_safe = false;
            RCLCPP_WARN(this->get_logger(), "电池未连接");
        }
        else if (remaining < 0.20f) {  // 低于20%
            is_safe = false;
        }
        else if (warning >= BatteryStatus::WARNING_CRITICAL) {
            is_safe = false;
        }

        // 3. 更新安全状态（使用release保证写入对其他线程可见）
        bool old_safe = safe_to_fly_.exchange(is_safe, std::memory_order_release);

        // 4. 只在状态变化时输出
        if (is_safe != old_safe) {
            if (is_safe) {
                RCLCPP_INFO(this->get_logger(),
                    "✅ 电池安全: %.1f%%", remaining * 100);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "❌ 电池不安全: %.1f%%", remaining * 100);
            }
        }

        // 5. 低电量警告（不频繁）
        if (connected && remaining < 0.30f) {  // 低于30%
            static auto last_warn = this->now();
            auto now = this->now();

            if ((now - last_warn).seconds() > 30.0) {  // 每30秒警告一次
                RCLCPP_WARN(this->get_logger(),
                    "⚠️ 电池电量低: %.1f%%", remaining * 100);
                last_warn = now;
            }
        }
    }

void OffboardControl::vehicle_control_mode_callback(const VehicleControlMode::SharedPtr msg)
{
    bool was_offboard = control_offboard_enabled_.load(std::memory_order_relaxed);
    bool now_offboard = msg->flag_control_offboard_enabled;
    control_offboard_enabled_.store(now_offboard, std::memory_order_relaxed);
    if (!was_offboard && now_offboard) {
        RCLCPP_INFO(get_logger(), "🚀 控制器已切换到Offboard模式");
        is_offboard_transitioning_.store(true);
    } else if (was_offboard && !now_offboard) {
        RCLCPP_WARN(get_logger(), "⚠️ 控制器已退出Offboard模式");
        is_fully_offboard_.store(false);
    }
}
void OffboardControl::process()
{
	// 获取当前状态（原子读取）
    uint8_t nav_state = current_nav_state_.load(std::memory_order_relaxed);
    uint8_t arming_state = current_arming_state_.load(std::memory_order_relaxed);
    bool was_offboard = control_offboard_enabled_.load(std::memory_order_relaxed);

    // 状态机主逻辑
    switch (current_state_) {
        case FlightState::INIT:
            RCLCPP_INFO(this->get_logger(), "初始化...");
            offboard_setpoint_counter_ = 0;
            update_state(FlightState::SENDING_CONTROL);
            break;

        case FlightState::SENDING_CONTROL:
            // 持续发送控制指令1秒（10次×100ms）
            // 如果publish_offboard_control_mode(); 和publish_trajectory_setpoint();之前已经持续发送信号，符合要求则px4内部offboard_available == true
            // publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);用来切换模式，并且offboard_available == true
            // 则切换为offboard模式可以立刻执行并不会失控 实现nav_state → OFFBOARD flag_control_offboard_enabled = true
            publish_offboard_control_mode();
            publish_trajectory_setpoint(0, 0, -5, -3.14);

            if (++offboard_setpoint_counter_ >= 10) {
                RCLCPP_INFO(this->get_logger(),
                           "已发送%lu次控制指令，请求Offboard模式",
                           offboard_setpoint_counter_);
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                update_state(FlightState::REQUESTING_OFFBOARD);
            }
            break;

        case FlightState::REQUESTING_OFFBOARD:
            // 继续发送控制指令
            publish_offboard_control_mode();
            publish_trajectory_setpoint(0, 0, -5, -3.14);

            // 检查超时
            if ((this->now() - state_start_time_) > offboard_timeout_) {
                RCLCPP_ERROR(this->get_logger(),
                           "请求Offboard模式超时！当前模式: %d",
                           nav_state);
                update_state(FlightState::ERROR);
                break;
            }

            // 检查是否已进入Offboard模式
            if (nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD && was_offboard==true) {
                RCLCPP_INFO(this->get_logger(), "成功进入Offboard模式！");
                update_state(FlightState::ARMING);
            }
            break;

        case FlightState::ARMING:
            // 发送解锁命令
            publish_offboard_control_mode();
            publish_trajectory_setpoint(0, 0, -5, -3.14);

            // 检查预解锁条件
            if (check_pre_arm_conditions()) {
                arm();
                update_state(FlightState::WAITING_ARM_CONFIRM);
            } else {
                RCLCPP_WARN(this->get_logger(), "预解锁检查未通过，等待...");
                // 可以在这里添加重试逻辑
            }
            break;

        case FlightState::WAITING_ARM_CONFIRM:
            // 继续发送控制指令
            publish_offboard_control_mode();
            publish_trajectory_setpoint(0, 0, -5, -3.14);

            // 检查超时
            if ((this->now() - state_start_time_) > arm_timeout_) {
                RCLCPP_ERROR(this->get_logger(), "解锁确认超时！当前解锁状态: %d",
                           arming_state);
                update_state(FlightState::ERROR);
                break;
            }

            // 检查是否已解锁
            if (arming_state == VehicleStatus::ARMING_STATE_ARMED) {
                RCLCPP_INFO(this->get_logger(), "无人机已解锁！");
                update_state(FlightState::OFFBOARD_ACTIVE);
            }
            break;

        case FlightState::OFFBOARD_ACTIVE:
            // 正常控制循环
            publish_offboard_control_mode();
            publish_trajectory_setpoint(0, 0, -5, -3.14);

            // 监控状态
            // monitor_safety(nav_state, arming_state);

            // 可以在这里添加任务逻辑
            // execute_mission();
            break;

        case FlightState::RETURN_REQUESTED:
            return_flight( nav_state);
            break;

        case FlightState::ERROR:
            // handle_error_state(nav_state, arming_state);
            break;
    }

}
void OffboardControl::return_flight(uint8_t nav_current_state)
{
    static bool rtl_logged = false;
    if (!rtl_command_sent_) {
        RCLCPP_INFO(this->get_logger(),
                     "进入 ERROR 状态，触发返航（RTL）");

        // 请求 PX4 执行 RTL
        publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
            0.0f, 0.0f);

        rtl_command_sent_ = true;
        rtl_start_time_ = this->now();
    }

    // 在短时间内重复发送 RTL，防止丢包
    if (nav_current_state != VehicleStatus::NAVIGATION_STATE_AUTO_RTL) {
        publish_offboard_control_mode();
        publish_trajectory_setpoint(0.0f, 0.0f, -5.0f, -3.14f);
    }

    if (nav_current_state == VehicleStatus::NAVIGATION_STATE_AUTO_RTL && !rtl_logged) {
        RCLCPP_WARN(this->get_logger(),
                    "PX4 已进入 RTL，Offboard 控制结束");
        rtl_logged = true;
    }

}
void OffboardControl::update_state(FlightState new_state)
{
    if (current_state_ != new_state) {
        RCLCPP_INFO(this->get_logger(),
                   "状态转换: %d -> %d",
                   static_cast<int>(current_state_),
                   static_cast<int>(new_state));

        current_state_ = new_state;
        state_start_time_ = this->now();
    }
}
bool OffboardControl::check_pre_arm_conditions()
{
    // 这里可以添加更多预解锁检查
    // 例如：检查安全开关、GPS状态等

    // 基本检查：确保已经在Offboard模式
    uint8_t nav_state = current_nav_state_.load(std::memory_order_relaxed);
    return (nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD);
}

void OffboardControl::monitor_safety(uint8_t nav_state, uint8_t arming_state)
{
    // 检查是否意外退出Offboard模式
    if (nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        RCLCPP_WARN(this->get_logger(),
                   "意外退出Offboard模式！当前模式: %d",
                   nav_state);
        // 可以考虑切换到错误状态或尝试恢复
    }

    // 检查是否意外上锁
    if (arming_state != VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_WARN(this->get_logger(), "无人机意外上锁！");
        update_state(FlightState::ERROR);
    }

    // 检查通信超时（通过VehicleStatus更新频率）
    static auto last_status_time = this->now();
    auto now = this->now();
    if ((now - last_status_time).seconds() > 1.0) {
        RCLCPP_WARN(this->get_logger(), "VehicleStatus更新超时！");
    }
    last_status_time = now;
}

void OffboardControl::handle_error_state(uint8_t nav_state, uint8_t arming_state)
{
    static int recovery_attempts = 0;
    const int max_recovery_attempts = 3;

    RCLCPP_ERROR(this->get_logger(),
                "错误状态处理中... 当前模式: %d, 解锁状态: %d",
                nav_state, arming_state);

    // 尝试上锁以确保安全
    if (arming_state == VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_WARN(this->get_logger(), "尝试上锁...");
        disarm();
    }

    // 尝试切换到手动模式
    if (nav_state != VehicleStatus::NAVIGATION_STATE_MANUAL) {
        if (recovery_attempts < max_recovery_attempts) {
            RCLCPP_INFO(this->get_logger(),
                       "尝试切换到手动模式 (尝试 %d/%d)",
                       recovery_attempts + 1, max_recovery_attempts);
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
            recovery_attempts++;
        } else {
            RCLCPP_FATAL(this->get_logger(), "恢复失败，停止控制！");
            timer_->cancel();  // 停止定时器
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "已成功切换到手动模式");
        // 可以尝试重新开始
        recovery_attempts = 0;
        update_state(FlightState::INIT);
    }
}
/**
 * @brief Send a command to Arm the vehicle 等价于左摇杆右下解锁无人机
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle 等价于左摇杆左下锁定无人机
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y,float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}
