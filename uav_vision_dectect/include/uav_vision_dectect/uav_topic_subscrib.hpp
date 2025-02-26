#pragma once

#include <opencv2/core/mat.hpp>
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


        /************************LightTrack跟踪部分************************/ 
        cv::Rect trackWindow;
        cv::Mat init_window;

        LightTrack *siam_tracker;
        int light_track_flag = 0;


        /************************PNP解算************************/ 
        Mat objPM;
        double obj_depth;
        std::vector<Point3f> objectPoints;             //三维坐标点
        std::vector<Point2f> projectedPoints;          //三维点投影到二维点的向量用于重画

        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F); // 创建旋转向量矩阵
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F); // 创建平移向量矩阵

        //矩阵K、D来自话题/camera/camera_info
        const double cameraD[3][3] = 
        {
            {454.68577, 0, 424.5},
            {0.000000, 454.68577, 240.5},
            {0, 0, 1.0000}
        };
        const double distC[5] = {0, 0, 0, 0, 0};

        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(cameraD[0]));
        cv::Mat distCoeffs = cv::Mat(5, 1, CV_64F, const_cast<double*>(distC));


        /**************************************************/
        std::thread uav_detect_result_thread_;

        void uav_detect_result_loop();
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
        void cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect &rect);


        void getTarget2dPoinstion(const cv::RotatedRect & rect, std::vector<Point2f> & target2d, const cv::Point2f & offset);
        void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
        void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz);
        void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz);

    
};



