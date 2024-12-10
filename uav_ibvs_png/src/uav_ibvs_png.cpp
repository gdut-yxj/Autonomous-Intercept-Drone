//
// Created by verse on 24-11-13.
//


#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/camera_info.hpp"

#include "uav_ibvs_png.h"

using namespace px4_msgs::msg;

uav_ibvs_png::uav_ibvs_png() : Node("uav_ibvs_png")
{

    /************************************订阅话题************************************/
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto os = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    uav_detect_sub_ = this->create_subscription<uav_common_msg::msg::RectMsg>("/uav_detect_result", 10,  std::bind(&uav_ibvs_png::uav_detect_callback, this, std::placeholders::_1));
    uav_camera_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10,  std::bind(&uav_ibvs_png::uav_camera_info_callback, this, std::placeholders::_1));
    ibvs_vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);



    /************************************设置期望位置************************************/
    //初始化数据变量
    cdMo = vpHomogeneousMatrix(20, 0, 0.75, 0, 0, 0);  //相机终点位置
    cMo = vpHomogeneousMatrix(0.15, -0.1, 10.0, vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));  //相机起始位置

    //角点
    ibvs_set_point.push_back(vpPoint(ibvs_desire_pos[0].x, ibvs_desire_pos[0].y, 0));
    ibvs_set_point.push_back(vpPoint(ibvs_desire_pos[1].x, ibvs_desire_pos[1].y, 0));
    ibvs_set_point.push_back(vpPoint(ibvs_desire_pos[2].x, ibvs_desire_pos[2].y, 0));
    ibvs_set_point.push_back(vpPoint(ibvs_desire_pos[3].x, ibvs_desire_pos[3].y, 0));


    ibvs_servo_task.setServo(vpServo::EYEINHAND_CAMERA);
    ibvs_servo_task.setInteractionMatrixType(vpServo::CURRENT);
    ibvs_servo_task.setLambda(0.5);   //伺服增益

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


    RCLCPP_INFO(this->get_logger(), "--------");


    rclcpp::WallRate loop_rate(500);

    while(1)
    {
        uav_ibvs_controller();

    }

}


void uav_ibvs_png::uav_detect_callback(const uav_common_msg::msg::RectMsg::SharedPtr msg)
{
    ibvs_current_pos[0] = cv::Point(msg->x, msg->y);
    ibvs_current_pos[1] = cv::Point(msg->x + msg->width, msg->y);
    ibvs_current_pos[2] = cv::Point(msg->x + msg->width, msg->y + msg->height);
    ibvs_current_pos[3] = cv::Point(msg->x, msg->y + msg->height);

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
    for (unsigned int i = 0; i < 4; i++)
    {
        ibvs_set_point[i].track(cMo);
        vpFeatureBuilder::create(p[i], ibvs_set_point[i]);
    }
    vpColVector v = ibvs_servo_task.computeControlLaw();
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);

    std::cout << "=========V=========" << std::endl;

    for (unsigned int i = 0; i < v.size(); ++i)
    {
        std::cout << "v[" << i << "] = " << v[i] << std::endl;
    }




    // /********************************************************/
    // OffboardControlMode msg{};
    // msg.position = true;
    // msg.velocity =false;
    // msg.acceleration = false;
    // msg.attitude = false;
    // msg.body_rate = false;
    // msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // offboard_control_mode_publisher_->publish(msg);
    //
    // /********************************************************/
    // TrajectorySetpoint msg{};
    // msg.position = {x, y, z};
    // msg.yaw = 0; // [-PI:PI]
    //
    // msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // ibvs_vehicle_command_publisher_->publish(msg);



    return v;

}


