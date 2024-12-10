//
// Created by verse on 24-11-28.
//

#include "vehicle_odometry.hpp"
#include <termios.h>
#include <unistd.h>



vehicle_odometry_node::vehicle_odometry_node() : Node("vehicle_odometry_node")
{

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_odometry_subscription_   = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,  std::bind(&vehicle_odometry_node::vehicle_odometry_callback, this, std::placeholders::_1));

}




void vehicle_odometry_node::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
    double roll, pitch, yaw;
    double q0, q1, q2, q3;

    q0 = msg->q[0];
    q1 = msg->q[1];
    q2 = msg->q[2];
    q3 = msg->q[3];

    // RCLCPP_INFO(this->get_logger(), "===============================================");

    roll  = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    pitch = std::asin(2 * (q0 * q2 - q3 * q1));
    yaw   = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));



    RCLCPP_INFO(this->get_logger(), "roll:%f, pitch:%f, yaw:%f", roll, pitch, yaw);

}


