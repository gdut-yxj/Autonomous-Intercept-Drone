//
// Created by verse on 24-11-24.
//

#ifndef UAV_VEHICLE_CONTROLLER_H
#define UAV_VEHICLE_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "../../../../build/px4_msgs/rosidl_generator_cpp/px4_msgs/msg/detail/vehicle_odometry__struct.hpp"

class uav_vehicle_controller : public rclcpp::Node
{
    public:
        float uav_current_yaw;
        float uav_current_roll;
        float uav_current_pitch;
        char  uav_Keyboard;
        uav_vehicle_controller();
        void run();

    private:

        px4_msgs::msg::TrajectorySetpoint keyboard_msg{};      //全局位置控制
        rclcpp::TimerBase::SharedPtr timer_;
        float uav_vehicle_ = 0;

        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr         vehicle_command_publisher_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr	offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr	    trajectory_setpoint_publisher_;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr     vehicle_odometry_subscription_;

        std::thread control_thread_; // 用于存储新线程
        std::thread keyboard_thread_;


        void uav_takeoff_loop();
        void publish_offboard_control_mode();
        void publish_trajectory_setpoint();
        void publish_vehicle_command(uint16_t command, float param1, float param2);
        void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

};



#endif //UAV_VEHICLE_CONTROLLER_H
