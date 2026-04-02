#include <rclcpp/rclcpp.hpp>
#include "pixhawk/offboard_px4.hpp"

int main(int argc, char *argv[])
{
	// std::cout << "Starting offboard control node..." << std::endl;
	// setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
