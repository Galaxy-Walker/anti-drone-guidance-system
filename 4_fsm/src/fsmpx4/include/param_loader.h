#ifndef FSMPX4_PARAM_LOADER_H
#define FSMPX4_PARAM_LOADER_H

#include "control.h"

#include <memory>

namespace rclcpp {
class Node;
}

namespace fsmpx4 {
namespace param_loader {

struct FSMParams
{
    struct Gains {
        double Kp_x;
        double Kp_y;
        double Kp_z;
        double Kv_x;
        double Kv_y;
        double Kv_z;
        double Kvi_x;
        double Kvi_y;
        double Kvi_z;
    } gains;

    struct Physical {
        double mass;
        double gravity;
        double Ixx;
        double Iyy;
        double Izz;
    } physical;

    struct Limits {
        double max_thrust;
        double min_thrust;
        double max_torque;
    } limits;

    struct ThrustMapping {
        double hover_percentage;
    } thr_map;

    struct Basic {
        double ctrl_freq_max;
        double max_manual_vel;
        bool use_integral;
        bool use_fmu_manual_topic;
    } basic;

    FSMParams();

    bool load_from_ros_node(const std::shared_ptr<rclcpp::Node>& node);
    bool load_from_node(rclcpp::Node& node);
};

bool load_params_from_node(const std::shared_ptr<rclcpp::Node>& node, FSMParams& params);
bool load_params_from_node(rclcpp::Node& node, FSMParams& params);

bool load_controller_config(const std::shared_ptr<rclcpp::Node>& node,
                            fsmpx4::control::PositionAttitudeController::Config& cfg);

} // namespace param_loader
} // namespace fsmpx4

#endif // FSMPX4_PARAM_LOADER_H
