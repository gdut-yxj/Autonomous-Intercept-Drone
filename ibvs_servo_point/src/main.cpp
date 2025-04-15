#include <opencv4/opencv2/core/hal/interface.h>
#include <ostream>
#include <rclcpp/rclcpp.hpp>

#include <iostream>

#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>


#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/detail/imu__builder.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

#include "uav_common_msg/msg/rect_msg.hpp"

#include "vpSimulatorPioneer.h"


using namespace px4_msgs::msg;
using namespace rclcpp;

// 定义当前
class ibvs_servo_point : public rclcpp::Node
{
    public:
        ibvs_servo_point(): Node("ibvs_servo_point")
        {
            uav_camera_frame = cv::imread("/home/verse/Pictures/target.png", cv::IMREAD_GRAYSCALE);

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
            
            uav_detect_sub_ = this->create_subscription<uav_common_msg::msg::RectMsg>(
                "/uav_detect_result", 
                rclcpp::SensorDataQoS(),  
                std::bind(&ibvs_servo_point::uav_detect_callback, this, std::placeholders::_1)
            );
     
            uav_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                10,
                std::bind(&ibvs_servo_point::image_callback, this, std::placeholders::_1)
            );

            offboard_control_mode_publisher_    = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
            ibvs_vehicle_command_publisher_     = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
            ibvs_trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
        }

        void uav_detect_callback(const uav_common_msg::msg::RectMsg::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "------------uav_detect_callback------------");
            RCLCPP_INFO(this->get_logger(), "XYWHZ: %d %d %d %d %d", msg->x, msg->y, msg->width, msg->height, msg->depth);

            double fx = 454.68;
            double fy = 454.68;
            double cx = 424.5;
            double cy = 240.5;

            //转换坐标
            current_pos_[0] = cv::Point((msg->x - cx) / fx, (msg->y - cy) / fy);
            current_pos_[1] = cv::Point((msg->x + msg->width - cx) / fx, (msg->y - cy) / fy);
            current_pos_[2] = cv::Point((msg->x + msg->width - cx) / fx, (msg->y + msg->height - cy) / fy);
            current_pos_[3] = cv::Point((msg->x - cx) / fx, (msg->y + msg->height - cy) / fy);
            depth_z = msg->depth;
        }


        cv::Mat& ibvs_get_current_frame()
        {
            return uav_camera_frame;
        }


        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            // 使用cv_bridge将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }


            uav_camera_frame = cv_ptr->image;
            cv::Mat frame = cv_ptr->image;
            frame.copyTo(uav_camera_frame);  
            cv::imshow("Camera Image", frame);
            cv::waitKey(1);

        }

    private:

        double current_x = 0;
        double current_y = 0;
        double depth_z =  0;
    
        cv::Mat uav_camera_frame;
        std::vector<cv::Point> current_pos_;

        rclcpp::Subscription<uav_common_msg::msg::RectMsg>::SharedPtr uav_detect_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uav_image_sub_;


        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr	offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr         ibvs_vehicle_command_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr	    ibvs_trajectory_setpoint_publisher_;

};




int main(int argc, char const *argv[])
{
    vpImage<unsigned char> I; 

    rclcpp::init(argc, argv);

    auto ibvs_servo_point_node = std::make_shared<ibvs_servo_point>();

    double depth = 0.1;
    double lambda = 50;
    double coef = 0.25; 

    vpSimulatorPioneer robot;

    vpCameraParameters cam;

    cam.initPersProjWithoutDistortion(1920, 1080, I.getWidth() / 2, I.getHeight() / 2);

    vpImageConvert::convert(ibvs_servo_point_node->ibvs_get_current_frame(), I);

    vpDisplayX d(I, 0, 0, "Current frame");

    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.setLambda(lambda);
    vpVelocityTwistMatrix cVe;
    cVe = robot.get_cVe();
    task.set_cVe(cVe);

    std::cout << "cVe: \n" << cVe << std::endl;       

    rclcpp::spin(ibvs_servo_point_node);
                         
    rclcpp::shutdown();
    return 0;
}

