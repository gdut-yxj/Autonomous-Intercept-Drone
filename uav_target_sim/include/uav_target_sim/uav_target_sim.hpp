//
// Created by verse on 24-10-16.
//

#ifndef UAV_TARGET_SIM_H
#define UAV_TARGET_SIM_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <cmath>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace GeographicLib;


class UavTargetControl : public rclcpp::Node
{
public:
    UavTargetControl();

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr	offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr	trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr		vehicle_command_publisher_;


    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr global_position_sub_;



    std::vector<double> enu_xyz = {0, 0, 0};
    std::vector<double> init_enu_xyz = {0, 0, 0};


    //画圆参数
    float radius_ = 5.0;        // Circle radius in meters
    float center_x_ = -20.0;    // Circle center x-coordinate
    float center_y_ = 5.0;      // Circle center y-coordinate
    float angular_speed_ = 0.1; // Radians per iteration
    float theta_ = 0.0;         // Initial angle for circular motion

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode();

    void publish_trajectory_setpoint(float x, float y, float z);
    void move_in_circle();


    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void global_position_callback(const px4_msgs::msg::SensorGps::SharedPtr msg);
};




#endif //UAV_TARGET_SIM_H
