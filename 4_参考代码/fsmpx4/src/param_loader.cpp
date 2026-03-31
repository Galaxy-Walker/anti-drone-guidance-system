#include <rclcpp/rclcpp.hpp>
#include <exception>
#include "../include/param_loader.h"

namespace fsmpx4 {
namespace param_loader {

FSMParams::FSMParams()
{
    gains.Kp_x = 8.5;
    gains.Kp_y = 2.5;
    gains.Kp_z = 4.0;
    gains.Kv_x = 3.0;
    gains.Kv_y = 3.0;
    gains.Kv_z = 4.5;
    gains.Kvi_x = 0.02;
    gains.Kvi_y = 0.02;
    gains.Kvi_z = 0.1;

    physical.mass = 2.0;
    physical.gravity = 9.81;
    physical.Ixx = 0.03232;
    physical.Iyy = 0.03232;
    physical.Izz = 0.04238;

    limits.max_thrust = -0.1;
    limits.min_thrust = -0.9;
    limits.max_torque = 0.5;

    thr_map.hover_percentage = 0.67;

    basic.ctrl_freq_max = 200.0;
    basic.max_manual_vel = 1.0;
    basic.use_integral = false;
    basic.use_fmu_manual_topic = false;
}

bool FSMParams::load_from_ros_node(const std::shared_ptr<rclcpp::Node>& node)
{
    return load_params_from_node(*node, *this);
}

bool FSMParams::load_from_node(rclcpp::Node& node)
{
    return load_params_from_node(node, *this);
}

bool load_params_from_node(const std::shared_ptr<rclcpp::Node>& node, FSMParams& params)
{
    return load_params_from_node(*node, params);
}

bool load_params_from_node(rclcpp::Node& node, FSMParams& params)
{
    try {
        params.basic.ctrl_freq_max = node.declare_parameter<double>("ctrl_freq_max", params.basic.ctrl_freq_max);
        params.basic.max_manual_vel = node.declare_parameter<double>("max_manual_vel", params.basic.max_manual_vel);
        params.basic.use_integral = node.declare_parameter<bool>("control.use_integral", params.basic.use_integral);
        params.basic.use_fmu_manual_topic = node.declare_parameter<bool>("manual_control_topic", params.basic.use_fmu_manual_topic);

        params.thr_map.hover_percentage = node.declare_parameter<double>("thr_map.hover_percentage", params.thr_map.hover_percentage);

        params.gains.Kp_x = node.declare_parameter<double>("control.Kp_x", params.gains.Kp_x);
        params.gains.Kp_y = node.declare_parameter<double>("control.Kp_y", params.gains.Kp_y);
        params.gains.Kp_z = node.declare_parameter<double>("control.Kp_z", params.gains.Kp_z);
        params.gains.Kv_x = node.declare_parameter<double>("control.Kv_x", params.gains.Kv_x);
        params.gains.Kv_y = node.declare_parameter<double>("control.Kv_y", params.gains.Kv_y);
        params.gains.Kv_z = node.declare_parameter<double>("control.Kv_z", params.gains.Kv_z);
        params.gains.Kvi_x = node.declare_parameter<double>("control.Kvi_x", params.gains.Kvi_x);
        params.gains.Kvi_y = node.declare_parameter<double>("control.Kvi_y", params.gains.Kvi_y);
        params.gains.Kvi_z = node.declare_parameter<double>("control.Kvi_z", params.gains.Kvi_z);

        params.physical.mass = node.declare_parameter<double>("control.mass", params.physical.mass);
        params.physical.gravity = node.declare_parameter<double>("control.gravity", params.physical.gravity);
        params.physical.Ixx = node.declare_parameter<double>("control.Ixx", params.physical.Ixx);
        params.physical.Iyy = node.declare_parameter<double>("control.Iyy", params.physical.Iyy);
        params.physical.Izz = node.declare_parameter<double>("control.Izz", params.physical.Izz);

        params.limits.max_thrust = node.declare_parameter<double>("control.max_thrust", params.limits.max_thrust);
        params.limits.min_thrust = node.declare_parameter<double>("control.min_thrust", params.limits.min_thrust);
        params.limits.max_torque = node.declare_parameter<double>("control.max_torque", params.limits.max_torque);

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node.get_logger(), "参数加载失败: %s", e.what());
        return false;
    }
}

bool load_controller_config(const std::shared_ptr<rclcpp::Node>& node,
                            fsmpx4::control::PositionAttitudeController::Config& cfg)
{
    FSMParams params;
    if (!load_params_from_node(*node, params)) {
        return false;
    }

    cfg.load_from_params(params);
    return true;
}

} // namespace param_loader
} // namespace fsmpx4
