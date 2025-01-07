#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../../../../build/px4_msgs/rosidl_generator_cpp/px4_msgs/msg/detail/vehicle_global_position__struct.hpp"
#include "../../../../build/px4_msgs/rosidl_generator_cpp/px4_msgs/msg/detail/vehicle_local_position__struct.hpp"


#include "LightTrack.h"
#include <px4_msgs/msg/sensor_gps.hpp>

#include "uav_common_msg/msg/rect_msg.hpp"




class UavTopicSubscrib : public rclcpp::Node
{

    public:
        cv::Mat uav_camera_frame;

        cv::Rect uav_result_rect;

        UavTopicSubscrib();



    private:



        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uav_image_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
        rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_position_sub_;


        rclcpp::Publisher<uav_common_msg::msg::RectMsg>::SharedPtr uav_detect_result_publisher_;
        
        
        uav_common_msg::msg::RectMsg pub_uav_result_rect;

        cv_bridge::CvImagePtr orig_cv_ptr;


        cv::Rect trackWindow;
        cv::Mat init_window;

        LightTrack *siam_tracker;

        int light_track_flag = 0;

        std::thread uav_detect_result_thread_;

        void uav_detect_result_loop();

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        void global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

        void cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect &rect);

};



