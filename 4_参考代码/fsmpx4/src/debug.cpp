#include <rclcpp/rclcpp.hpp>
#include "../include/param_loader.h"

namespace fsmpx4 {
namespace debug {

using fsmpx4::control::PositionAttitudeController;

// 测试/调试函数：从参数服务器读取并打印控制参数。
// 使用方式（示例）：
//   auto node = std::make_shared<rclcpp::Node>("px4ctrl_fsm");
//   fsmpx4::debug::print_loaded_params(node);
void print_loaded_params(const std::shared_ptr<rclcpp::Node>& node)
{
    PositionAttitudeController::Config cfg;
    if (!fsmpx4::param_loader::load_controller_config(node, cfg)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter loading failed in debug");
        return;
    }
    RCLCPP_INFO(node->get_logger(),
                "[DEBUG] Kp=(%.3f, %.3f, %.3f) Kv=(%.3f, %.3f, %.3f) mass=%.3f g=%.3f",
                cfg.kp.x(), cfg.kp.y(), cfg.kp.z(), cfg.kv.x(), cfg.kv.y(), cfg.kv.z(), cfg.mass, cfg.gravity);
}

} // namespace debug
} // namespace fsmpx4

