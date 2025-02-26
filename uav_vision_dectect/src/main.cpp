//
// Created by verse on 24-10-9.
//
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>


#include "uav_topic_subscrib.hpp"
#include "uav_control.hpp"


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    //图像检测节点
    auto uav_image_detector_node = std::make_shared<UavTopicSubscrib>();

    auto uav_offboard_control_node = std::make_shared<UavControl>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(uav_image_detector_node);
    executor.add_node(uav_offboard_control_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
