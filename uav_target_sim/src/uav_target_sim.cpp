#include "uav_target_sim.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <GeographicLib/LocalCartesian.hpp>


#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <px4_msgs/msg/sensor_gps.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



UavTargetControl::UavTargetControl() : Node("uav_control")
{
	offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_2/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint", 10);
	vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_2/fmu/in/vehicle_command", 10);

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


	// global_position_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>("px4_2/fmu/out/vehicle_gps_position", qos,  std::bind(&UavTargetControl::global_position_callback, this, std::placeholders::_1));


	offboard_setpoint_counter_ = 0;

	auto timer_callback = [this]() -> void {

		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0);

		publish_offboard_control_mode();

		publish_trajectory_setpoint(-15, 5 , -5);



	};
	timer_ = this->create_wall_timer(100ms, timer_callback);
}


/**
 * @brief Send a command to Arm the vehicle
 */
void UavTargetControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void UavTargetControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void UavTargetControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity =false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void UavTargetControl::publish_trajectory_setpoint(float x, float y, float z)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = 0; // [-PI:PI]

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}



/**
 * @brief Circular motion around the specified center.
 */
void UavTargetControl::move_in_circle()
{
	TrajectorySetpoint msg{};
	theta_ += angular_speed_; // Increment the angle to create circular motion
	if (theta_ > 2 * M_PI) {
		theta_ -= 2 * M_PI; // Keep angle within [0, 2π] range
	}

	// Calculate the new position using polar to cartesian conversion
	float new_x = center_x_ + radius_ * cos(theta_);
	float new_y = center_y_ + radius_ * sin(theta_);

	msg.position = {new_x, new_y, -5.0}; // Hover at 5 meters altitude
	msg.yaw = theta_; // Keep yaw aligned with the circle
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	RCLCPP_INFO(this->get_logger(), "Moving to (%.2f, %.2f)", new_x, new_y);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void UavTargetControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 3;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void UavTargetControl::global_position_callback(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
	if (!msg) {
		RCLCPP_WARN(this->get_logger(), "Received null GPS message");
		return;
	}


	static bool initialized = false;
	px4_msgs::msg::SensorGps::SharedPtr target_sim_position = msg;
	Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());


	RCLCPP_INFO(this->get_logger(), "gps_position:%f, %f, %f", target_sim_position->latitude_deg, target_sim_position->longitude_deg, target_sim_position->altitude_msl_m);


	if (!initialized){
		earth.Forward(target_sim_position->latitude_deg, target_sim_position->longitude_deg, target_sim_position->altitude_msl_m, this->init_enu_xyz[0], this->init_enu_xyz[1], this->init_enu_xyz[2]);
		RCLCPP_INFO(this->get_logger(), "init_enu:%f, %f, %f",  this->init_enu_xyz[0], this->init_enu_xyz[1], this->init_enu_xyz[2]);
		initialized = true;
	}

	earth.Forward(target_sim_position->latitude_deg, target_sim_position->longitude_deg, target_sim_position->altitude_msl_m, this->enu_xyz[0], this->enu_xyz[1], this->enu_xyz[2]);
	RCLCPP_INFO(this->get_logger(), "enu:%f, %f, %f",  this->enu_xyz[0], this->enu_xyz[1], this->enu_xyz[2]);

}

