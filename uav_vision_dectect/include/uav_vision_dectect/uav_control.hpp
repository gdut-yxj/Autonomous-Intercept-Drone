#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class UavControl : public rclcpp::Node
{
	public:
		UavControl();

		void arm();
		void disarm();

	private:
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Publisher<OffboardControlMode>::SharedPtr	offboard_control_mode_publisher_;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr	trajectory_setpoint_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr		vehicle_command_publisher_;



		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

		uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

		void publish_offboard_control_mode();
		void publish_trajectory_setpoint();
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};
