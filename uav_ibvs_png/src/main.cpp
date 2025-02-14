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
	
    rclcpp::init(argc, argv);
    
    auto uav_ibvs_png_node = std::make_shared<uav_ibvs_png>();

    rclcpp::WallRate loop_rate(100);

    while(rclcpp::ok())
    {
        RCLCPP_INFO(uav_ibvs_png_node->get_logger(), "-------main-------");

        uav_ibvs_png_node->uav_ibvs_controller();

	    rclcpp::spin_some(uav_ibvs_png_node);

        loop_rate.sleep();

    }


	rclcpp::shutdown();

	return 0;

}