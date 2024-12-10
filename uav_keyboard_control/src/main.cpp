//
// Created by verse on 24-10-30.
//
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono;
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

class DroneKeyboardControl : public rclcpp::Node
{
    public:
        DroneKeyboardControl() : Node("drone_keyboard_control")
        {
            //参数初始化
            keyboard_msg.position = {5.0, 5.0, -5.0};
            keyboard_msg.yaw = -3.14; // [-PI:PI]

            // 创建无人机控制命令发布者
            offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
            status_subscription_ =  this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/px4_1/fmu/out/vehicle_status", 10, std::bind(&DroneKeyboardControl::status_callback, this, std::placeholders::_1));

            control_thread_ = std::thread(&DroneKeyboardControl::uav_takeoff_loop, this);
            RCLCPP_INFO(this->get_logger(), "Drone Keyboard Control Node Started. Use W/A/S/D for movement and Q/E for altitude.");

            run();
        }


        void uav_takeoff_loop()
        {
            RCLCPP_INFO(this->get_logger(), "Arm command send");
            while (rclcpp::ok())
            {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0);
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
                std::this_thread::sleep_for(std::chrono::microseconds(100)); // 每100ms输出一次
            }
        }



        void publish_offboard_control_mode()
        {
            OffboardControlMode msg{};
            msg.position = true;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_publisher_->publish(msg);
        }


        void publish_trajectory_setpoint()
        {
            keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_setpoint_publisher_->publish(keyboard_msg);

        }

        void publish_vehicle_command(uint16_t command, float param1, float param2)
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



        void run()
        {
            while (rclcpp::ok()) {

                char key = getKey();

                switch (key) {
                    case 'w':
                        keyboard_msg.position[0] += 0.5;
                        keyboard_msg.yaw = -3.14; // [-PI:PI]
                        keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        trajectory_setpoint_publisher_->publish(keyboard_msg);
                        RCLCPP_INFO(this->get_logger(), "Moving Forward");
                        break;

                    case 's':
                        keyboard_msg.position[0] -= 0.5;
                        keyboard_msg.yaw = -3.14; // [-PI:PI]
                        keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        trajectory_setpoint_publisher_->publish(keyboard_msg);
                        RCLCPP_INFO(this->get_logger(), "Moving Backward");
                        break;

                    case 'a':
                        keyboard_msg.position[1] += 0.5;
                        // keyboard_msg.yaw = -3.14; // [-PI:PI]
                        keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        trajectory_setpoint_publisher_->publish(keyboard_msg);
                        RCLCPP_INFO(this->get_logger(), "Moving Left");
                        break;

                    case 'd':
                        keyboard_msg.position[1] -= 0.5;
                        // keyboard_msg.yaw = -3.14; // [-PI:PI]
                        keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        trajectory_setpoint_publisher_->publish(keyboard_msg);
                        RCLCPP_INFO(this->get_logger(), "Moving Right");
                        break;

                    case 'q':
                        keyboard_msg.position[2] += 0.5;
                        // keyboard_msg.yaw = -3.14; // [-PI:PI]
                        keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        trajectory_setpoint_publisher_->publish(keyboard_msg);
                        RCLCPP_INFO(this->get_logger(), "Moving Up");
                        break;

                    case 'e':
                        keyboard_msg.position[2] -= 0.5;
                        // keyboard_msg.yaw = -3.14; // [-PI:PI]
                        keyboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        trajectory_setpoint_publisher_->publish(keyboard_msg);
                        RCLCPP_INFO(this->get_logger(), "Moving Down");
                        break;

                    case 'x':
                        RCLCPP_INFO(this->get_logger(), "Exiting...");
                        return;

                    default:
                        RCLCPP_WARN(this->get_logger(), "Unknown command");
                }
            }
        }

        void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
        {
            // 检查无人机是否解锁
            if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
            {
                RCLCPP_INFO(this->get_logger(), "Drone is armed (unlocked).");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Drone is disarmed (locked).");
            }
        }




    private:

        TrajectorySetpoint keyboard_msg{};      //全局位置控制

        uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Publisher<OffboardControlMode>::SharedPtr	offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr	trajectory_setpoint_publisher_;



        std::thread control_thread_; // 用于存储新线程


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DroneKeyboardControl>();

    rclcpp::spin(node);


    rclcpp::shutdown();

    return 0;
}
