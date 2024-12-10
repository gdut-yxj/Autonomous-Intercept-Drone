//
// Created by verse on 24-11-28.
//
#include <rclcpp/rclcpp.hpp>

#include "vehicle_odometry.hpp"


int main(int argc, char *argv[])
{
	std::cout << "Starting intel node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<vehicle_odometry_node>());

	rclcpp::shutdown();

	return 0;
}