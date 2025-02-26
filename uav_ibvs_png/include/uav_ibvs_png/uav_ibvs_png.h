//
// Created by verse on 24-11-13.
//

#ifndef UAV_IBVS_PNG_H
#define UAV_IBVS_PNG_H

#include <rclcpp/rclcpp.hpp>
#include "opencv2/opencv.hpp"
#include <vector>
#include "uav_common_msg/msg/rect_msg.hpp"


#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPoint.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>



#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/detail/imu__builder.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


#include <visp3/gui/vpPlot.h>


class uav_ibvs_png : public rclcpp::Node
{
    public:
        uav_ibvs_png();
        vpColVector uav_ibvs_controller();

    private:

        double depth_z =  0;
        vpHomogeneousMatrix cdMo;     // 相机目标位置
        vpHomogeneousMatrix cMo;      // 相机初始位置

        std::vector<vpPoint> ibvs_set_point;
        std::vector<vpPoint> ibvs_current_point;
        vpServo ibvs_servo_task;

        vpHomogeneousMatrix wMc, wMo;
        vpSimulatorCamera robot;

        vpPlot plotter;
        vpImage<unsigned char> I; // Create a gray level image container

        vpFeaturePoint p[4], pd[4];
        vpFeaturePoint current_point[4], desired_point[4];



        std::vector<cv::Point> desire_pos_ = {cv::Point(400, 200), cv::Point(500, 250)};
        std::vector<cv::Point> current_pos_ = {cv::Point(374, 190), cv::Point(474, 290)};
        cv::Rect callback_detect_result;

        px4_msgs::msg::TrajectorySetpoint ibvs_png_msg{};

        std::vector<std::float_t> K = {0.0, 0.0, 0.0, 0, 0, 0, 0, 0 ,0};
        float target_angle;                                         //计算视线角度


        rclcpp::Subscription<uav_common_msg::msg::RectMsg>::SharedPtr uav_detect_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr uav_camera_sub_;


        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr	offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr         ibvs_vehicle_command_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr	    ibvs_trajectory_setpoint_publisher_;

        std::thread uav_takeoff_thread_;

        void uav_takeoff_loop();
        void uav_detect_callback(const uav_common_msg::msg::RectMsg::SharedPtr msg);
        void uav_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void publish_offboard_control_mode();
        void publish_vehicle_command(uint16_t command, float param1, float param2);

};



#endif //UAV_IBVS_PNG_H
