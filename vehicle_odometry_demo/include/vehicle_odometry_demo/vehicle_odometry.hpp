//
// Created by verse on 24-11-28.
//

#ifndef VEHICLE_ODOMETRY_H
#define VEHICLE_ODOMETRY_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>



class vehicle_odometry_node : public rclcpp::Node
{

    public:
        vehicle_odometry_node();


    private:

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr     vehicle_odometry_subscription_;

        void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);


};


#endif //VEHICLE_ODOMETRY_H
