//
// Created by verse on 24-11-24.

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executor.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>


#include "uav_vehicle_controller.hpp"

using namespace std::chrono;
using namespace px4_msgs::msg;



int main(int argc, char *argv[])
{
    std::cout << "Starting uav_vehicle_controller node..." << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto uav_vehicle_controller_node = std::make_shared<uav_vehicle_controller>();

    rclcpp::spin(uav_vehicle_controller_node);

    rclcpp::shutdown();
    
    return 0;
}
