//
// Created by verse on 24-11-13.
//

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "uav_ibvs_png.h"

int main(int argc, char const *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);


	rclcpp::spin(std::make_shared<uav_ibvs_png>());

	rclcpp::shutdown();


	return 0;




}