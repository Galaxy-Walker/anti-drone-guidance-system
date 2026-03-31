#include <rclcpp/rclcpp.hpp>
#include "../include/fsmpx4.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<fsmpx4::FSMPX4>();
    RCLCPP_INFO(node->get_logger(), "FSMPX4 状态机节点启动");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
