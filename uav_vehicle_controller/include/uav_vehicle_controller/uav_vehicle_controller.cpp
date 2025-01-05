//
// Created by verse on 24-11-25.
//
#include "uav_vehicle_controller.hpp"
#include <cmath>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


using namespace px4_msgs::msg;


// 读取键盘输入
char getKey()
{
    termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}



char getKeyNonBlocking()
{
    struct termios oldt, newt;
    int oldf;
    char ch;

    // 获取当前终端设置
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 设置为非规范模式，无回显

    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // 设置为非阻塞模式

    ch = getchar();

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return ch;
}




uav_vehicle_controller::uav_vehicle_controller() : Node("uav_vehicle_controller")
{

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    //参数初始化
    keyboard_msg.position = {nan(""), nan(""), nan("")};
    keyboard_msg.velocity = {0, 0, 0};

    keyboard_msg.yaw = 3.14; // [-PI:PI]

    // 创建无人机控制命令发布者
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_   = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_       = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
    vehicle_odometry_subscription_   = this->create_subscription<VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry", qos,  std::bind(&uav_vehicle_controller::vehicle_odometry_callback, this, std::placeholders::_1));
    control_thread_                  = std::thread(&uav_vehicle_controller::uav_takeoff_loop, this);
    keyboard_thread_                 = std::thread(&uav_vehicle_controller::run, this);

    RCLCPP_INFO(this->get_logger(), "Drone Keyboard Control Node Started. Use W/A/S/D for movement and Q/E for altitude.");

}


void uav_vehicle_controller::uav_takeoff_loop()
{
    RCLCPP_INFO(this->get_logger(), "Arm command send");

    rclcpp::WallRate loop_rate(30);

    while (rclcpp::ok())
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0);
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        loop_rate.sleep();
    }
}



void uav_vehicle_controller::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}


void uav_vehicle_controller::publish_trajectory_setpoint()
{
    RCLCPP_INFO(this->get_logger(), "----------x:%f, y:%f, z:%f", keyboard_msg.velocity[0], keyboard_msg.velocity[1], keyboard_msg.velocity[2]);
    keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(keyboard_msg);

}

void uav_vehicle_controller::publish_vehicle_command(uint16_t command, float param1, float param2)
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
    vehicle_command_publisher_->publish(msg);
}



void uav_vehicle_controller::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
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

    uav_current_pitch = pitch;
    uav_current_yaw   = yaw;
    uav_current_roll  = roll;

    // RCLCPP_INFO(this->get_logger(), "roll:%f, pitch:%f, yaw:%f", roll, pitch, yaw);

}





void uav_vehicle_controller:: run()
{
    rclcpp::WallRate loop_rate(500.0);
    while (rclcpp::ok())
    {
        loop_rate.sleep();
        uav_Keyboard = getKeyNonBlocking();

        if (uav_Keyboard != EOF)
        {

            switch (uav_Keyboard)
            {
                case 'w':

                    uav_vehicle_ += 0.05;
                    keyboard_msg.velocity[0] = uav_vehicle_ * cos(uav_current_yaw);
                    keyboard_msg.velocity[1] = uav_vehicle_ * sin(uav_current_yaw);
                    RCLCPP_INFO(this->get_logger(), "Moving Forward");
                    RCLCPP_INFO(this->get_logger(), "x:%f, y:%f",  keyboard_msg.velocity[0],  keyboard_msg.velocity[1]);
                    RCLCPP_INFO(this->get_logger(), "roll:%f, pitch:%f, yaw:%f", uav_current_roll, uav_current_pitch, uav_current_yaw);
                    break;

                case 's':

                    uav_vehicle_ -= 0.05;
                    keyboard_msg.velocity[0] = uav_vehicle_ * cos(uav_current_yaw);
                    keyboard_msg.velocity[1] = uav_vehicle_ * sin(uav_current_yaw);
                    RCLCPP_INFO(this->get_logger(), "Moving Backward");
                    RCLCPP_INFO(this->get_logger(), "x:%f, y:%f",  keyboard_msg.velocity[0],  keyboard_msg.velocity[1]);
                    RCLCPP_INFO(this->get_logger(), "roll:%f, pitch:%f, yaw:%f", uav_current_roll, uav_current_pitch, uav_current_yaw);
                    break;


                case 'a':
                    keyboard_msg.yaw += 0.05;
                    RCLCPP_INFO(this->get_logger(), "Moving Left");
                    RCLCPP_INFO(this->get_logger(), "roll:%f, pitch:%f, yaw:%f", uav_current_roll, uav_current_pitch, uav_current_yaw);
                    break;


                case 'd':
                    keyboard_msg.yaw -= 0.05;
                    RCLCPP_INFO(this->get_logger(), "Moving Right");
                    RCLCPP_INFO(this->get_logger(), "roll:%f, pitch:%f, yaw:%f", uav_current_roll, uav_current_pitch, uav_current_yaw);

                    break;

                case 'q':
                    keyboard_msg.velocity[2] -= 0.05;
                    RCLCPP_INFO(this->get_logger(), "Moving Up");
                    RCLCPP_INFO(this->get_logger(), "z:%f",  keyboard_msg.velocity[2]);
                    break;

                case 'e':
                    keyboard_msg.velocity[2] += 0.05;
                    RCLCPP_INFO(this->get_logger(), "Moving Down");
                    RCLCPP_INFO(this->get_logger(), "z:%f",  keyboard_msg.velocity[2]);
                    break;

                case 'x':
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    return;

                default:
                    RCLCPP_WARN(this->get_logger(), "Unknown command");

            }
        }

    }
}


