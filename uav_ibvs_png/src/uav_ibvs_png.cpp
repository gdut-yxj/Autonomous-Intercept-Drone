//
// Created by verse on 24-11-13.
//
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "sensor_msgs/msg/camera_info.hpp"

#include "uav_ibvs_png.h"

using namespace px4_msgs::msg;

uav_ibvs_png::uav_ibvs_png() : Node("uav_ibvs_png")
{

    /************************************订阅话题************************************/
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto os = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    uav_detect_sub_ = this->create_subscription<uav_common_msg::msg::RectMsg>("/uav_detect_result", 10,  std::bind(&uav_ibvs_png::uav_detect_callback, this, std::placeholders::_1));
    // uav_camera_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10,  std::bind(&uav_ibvs_png::uav_camera_info_callback, this, std::placeholders::_1));

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
    ibvs_vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
    ibvs_trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);


    /************************************设置期望位置************************************/
    //初始化数据变量
    cdMo = vpHomogeneousMatrix(0, 0, 0.05, 0, 0, 0);  //相机终点位置
    cMo = vpHomogeneousMatrix(0.15, -0.1, 10.0, vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));  //相机起始位置

    //角点
    ibvs_set_point.push_back(vpPoint(desire_pos_[0].x, desire_pos_[0].y, 0));
    ibvs_set_point.push_back(vpPoint(desire_pos_[1].x, desire_pos_[1].y, 0));
    ibvs_set_point.push_back(vpPoint(desire_pos_[2].x, desire_pos_[2].y, 0));
    ibvs_set_point.push_back(vpPoint(desire_pos_[3].x, desire_pos_[3].y, 0));


    ibvs_servo_task.setServo(vpServo::EYEINHAND_CAMERA);
    ibvs_servo_task.setInteractionMatrixType(vpServo::CURRENT);
    ibvs_servo_task.setLambda(0.001);   //伺服增益


    for (unsigned int i = 0; i < 4; i++)
    {
        ibvs_set_point[i].track(cdMo);
        vpFeatureBuilder::create(pd[i], ibvs_set_point[i]);
        ibvs_set_point[i].track(cMo);
        vpFeatureBuilder::create(p[i], ibvs_set_point[i]);
        ibvs_servo_task.addFeature(p[i], pd[i]);
    }


    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    vpImage<unsigned char> Iint(480, 640, 255);
    vpImage<unsigned char> Iext(480, 640, 255);

    vpCameraParameters cam(840, 840, Iint.getWidth() / 2, Iint.getHeight() / 2);
    vpHomogeneousMatrix cextMo(0, 0, 3, 0, 0, 0);


    /************************************启动TakeOff线程************************************/
    ibvs_png_msg.position = {5.0, 5.0, -5.0};
    ibvs_png_msg.velocity = {0, 0, 0};
    ibvs_png_msg.yaw = 3.14;
    uav_takeoff_thread_ = std::thread(&uav_ibvs_png::uav_takeoff_loop, this);


}


void uav_ibvs_png::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}



void uav_ibvs_png::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 2;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    ibvs_vehicle_command_publisher_->publish(msg);
}




void uav_ibvs_png::uav_detect_callback(const uav_common_msg::msg::RectMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "------------uav_detect_callback------------");

    current_pos_[0] = cv::Point(msg->x, msg->y);
    current_pos_[1] = cv::Point(msg->x + msg->width, msg->y);
    current_pos_[2] = cv::Point(msg->x + msg->width, msg->y + msg->height);
    current_pos_[3] = cv::Point(msg->x, msg->y + msg->height);

}

void uav_ibvs_png::uav_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    /**
     * K：相机内参矩阵，使用焦距(fx, fy)和主点坐标(cx, cy)，单位为像素，内参矩阵可以将相机坐标中的3D点投影到2D像素坐标
     *         fx  0   cx
     *         0   fy  cy
     *         0   0   1
     */

    for (int i = 0; i < 9; ++i)
    {
        this->K.push_back(msg->k.at(i));
        RCLCPP_INFO(this->get_logger(), "k:%f ",  msg->k.at(0));
    }


}



vpColVector uav_ibvs_png::uav_ibvs_controller()
{
    /************************************计算当前与期望************************************/
    robot.getPosition(wMc);
    cMo = wMc.inverse() * wMo;

    ibvs_current_point.push_back(vpPoint(current_pos_[0].x, current_pos_[0].y, 0));
    ibvs_current_point.push_back(vpPoint(current_pos_[1].x, current_pos_[1].y, 0));
    ibvs_current_point.push_back(vpPoint(current_pos_[2].x, current_pos_[2].y, 0));
    ibvs_current_point.push_back(vpPoint(current_pos_[3].x, current_pos_[3].y, 0));


    for (unsigned int i = 0; i < 4; i++)
    {
        ibvs_set_point[i].track(cMo);
        vpFeatureBuilder::create(p[i], ibvs_current_point[i]);
    }
    vpColVector v = ibvs_servo_task.computeControlLaw();
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);
    
    std::cout << "=========V=========" << std::endl;
    
    for (unsigned int i = 0; i < v.size(); ++i)
    {
        std::cout << "v[" << i << "] = " << v[i] << std::endl;
    }


    /********************************************************/
    ibvs_png_msg.velocity[0] = v[0];
    ibvs_png_msg.velocity[1] = v[1];
    ibvs_png_msg.velocity[2] = v[2];

    // ibvs_png_msg.velocity[0] = 0.01;
    // ibvs_png_msg.velocity[1] = 0;
    // ibvs_png_msg.velocity[2] = 0;

    ibvs_png_msg.acceleration[0] = 3; // X方向加速度
    ibvs_png_msg.acceleration[1] = 0.0; // Y方向加速度
    ibvs_png_msg.acceleration[2] = 0; // Z方向加速度

    ibvs_png_msg.yaw = vpMath::rad(v[5]) + 3.14;

    ibvs_png_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    ibvs_trajectory_setpoint_publisher_->publish(ibvs_png_msg);



    return v;

}



void uav_ibvs_png::uav_takeoff_loop()
{
    RCLCPP_INFO(this->get_logger(), "Arm command send");

    rclcpp::WallRate loop_rate(10);

    while (rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "uav_take_off loop");

        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0);
        publish_offboard_control_mode();
        ibvs_png_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        ibvs_trajectory_setpoint_publisher_->publish(ibvs_png_msg);
        loop_rate.sleep();
    }
}

