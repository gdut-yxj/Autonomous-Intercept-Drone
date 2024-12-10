//
// Created by verse on 24-10-16.
//
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "uav_target_sim.hpp"



int main(int argc, char const *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavTargetControl>());

    rclcpp::shutdown();
    return 0;
}