//
//  An autonomous ......文章的复现
//
#include <rclcpp/rclcpp.hpp>
//  发送控制信号
#include <px4_msgs/msg/offboard_control_mode.hpp>
//  轨迹点话题
#include <px4_msgs/msg/trajectory_setpoint.hpp>
//  
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

#include "uav_common_msg/msg/rect_msg.hpp"

//	姿态数据信息
#include <px4_msgs/msg/vehicle_odometry.hpp>
//	速度位置信息
#include "px4_msgs/msg/vehicle_local_position.hpp"
//	姿态及升力数据设定信息
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"


#include <chrono>

//	互斥锁
#include "mutex"
using namespace px4_msgs::msg;
using namespace std::chrono;

class uav_chase : public rclcpp::Node
{
public:
	~uav_chase()
	{
		std::cout<<"a"<<this->a_max<<std::endl;
	}
    //  构造函数，当使用类初始化对象时自动执行
    uav_chase():Node("uav_chase")
    {

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		//创建控制模式发布者
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
		//创建轨迹点模式发布者
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
		//控制命令发布者
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
		//创建轨迹及升力控制
		vehicle_rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/px4_1/fmu/in/vehicle_rates_setpoint", 10);
		
		//	创建 识别信息话题接收者
		track_result_subscription_ = this->create_subscription<uav_common_msg::msg::RectMsg>("/uav_detect_result", 10,
		[this](const uav_common_msg::msg::RectMsg::UniquePtr msg)
		{
			// 需要添加互斥锁
			this->x = msg->x;
			this->y = msg->y;
			this->uav_width = msg->width;
			this->uav_height = msg->height;
			this->tracke_msg_flag = true;
			track_time +=1;
		});

		//	创建 姿态信息话题接收者
		vehicle_odometry_subscription_ = this->create_subscription<VehicleOdometry>("px4_1/fmu/out/vehicle_odometry", qos,
		[this](const VehicleOdometry::SharedPtr msg)
		{
			double q0, q1, q2, q3;
			q0 = msg->q[0];
	    	q1 = msg->q[1];
   			q2 = msg->q[2];
    		q3 = msg->q[3];
			this->roll  = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
		    this->pitch = std::asin(2 * (q0 * q2 - q3 * q1));
    		this->yaw = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
		});
		//	创建 速度信息话题接收者
		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>("px4_1/fmu/out/vehicle_local_position", qos,
		[this](const VehicleLocalPosition::SharedPtr msg)
		{
			this->vx = msg->vx;
			this->vy = msg->vy;
			this->vz = msg->vz;
		});


		//	定时器回调函数
		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				// Arm the vehicle
				this->arm();
			}
			if(track_time>10)
			{//	发布控制模式与轨迹点
				double error_y = y - this->image_height/2;
				double error_x = x - this->image_width/2;
				this->w_roll = this->k6*(this->roll - this->desire_roll);
				this->w_pitch = -this->k2*error_y - this->k3*(this->pitch - this->desire_pitch);
				this->w_yaw = this->k5*error_x;
				this->f = -this->m/cos(this->pitch)*(this->k4*(this->vz-this->k1*error_y)+this->g)/this->f_max;	//最后除以f_max进行归一化，得到最后的输出
				std::cout<<"############################################"<<std::endl;
				std::cout<<"f:		"<<this->f*this->f_max<<std::endl;
				std::cout<<"w_roll:	"<<this->w_roll<<std::endl;
				std::cout<<"w_pitch:"<<this->w_pitch<<std::endl;
				std::cout<<"w_yaw:	"<<this->w_yaw<<std::endl;
				std::cout<<"roll:	"<<this->roll<<std::endl;
				std::cout<<"pitch:	"<<this->pitch<<std::endl;
				std::cout<<"yaw:	"<<this->yaw<<std::endl;
				std::cout<<"ERROR_X:	"<<error_x<<std::endl;
				std::cout<<"ERROR_Y:	"<<error_y<<std::endl;
				//this->pitch = 0;
				publish_offboard_control_mode_att();
				publish_rates_setpoint(this->w_roll, this->w_pitch, this->w_yaw, this->f);
			}
			else{
			//	发布角速度控制器
			publish_offboard_control_mode();
			publish_trajectory_setpoint(5.0,5.0,-5.0,3.14);
			}
		//	如果数据均更新，则进入计算
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		//	创建一个20Hz的定时器
		timer_ = this->create_wall_timer(10ms, timer_callback);
    }
private:
    void arm();         //  解锁
    void disarm();      //  上锁
    void publish_offboard_control_mode();   //  发布控制模式
    void publish_trajectory_setpoint(float x,float y,float z,float yaw);    //  发布轨迹控制
    void publish_trajectory_velocity(float x,float y,float z,float yaw);    //  发布速度控制
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);     //发布控制命令
    void take_off();
	void publish_rates_setpoint(double roll, double pitch, double yaw, double thrust);
	void publish_offboard_control_mode_att();
	//  定时器
    rclcpp::TimerBase::SharedPtr timer_;
    //  发布方
        //  发布控制模式
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	    //  发布轨迹控制
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        //  发布
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		//	发布姿态以及升力数据
	rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;

    //  订阅者
	//	GPS订阅者
	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr GPS_subscription_;
	//	追踪数据订阅
	rclcpp::Subscription<uav_common_msg::msg::RectMsg>::SharedPtr track_result_subscription_;
	//	姿态数据订阅
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscription_;
	//	速度数据订阅
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_ = 0;   //!< counter for the number of setpoints sent
	double a_max;
	//	计算过程中所需要使用到的各种信息
	double f_max = 0.9*9.81/0.536501;
	double m = 0.9;
	//	k1 0.011 k2 0.0005 k3 0.3
	double k1 = 0.02;
	double k2 = 0.0015;
	double k3 = 1.5;
	double k4 = 1;
	double k5 = 0.0012;
	double k6 = 0.025;
	double desire_roll = 0;
	double roll;
	double desire_pitch = -0.3;
	double pitch;
	double yaw;
	double g = 9.81;
	double vx;
	double vy;
	//	等于cvy
	double vz;
	//	接收的无人机数据
	double x;
	double y;
	double image_width = 1920;
	double image_height = 1080;
	double uav_width;
	double uav_height;
	//	无人机最后得到的角速度以及力度
	double f = 0;
	double w_roll = 0;
	double w_pitch = 0;
	double w_yaw = 0;
	//	标志位
	bool tracke_msg_flag = false;
	bool uav_msg_flag = false;
	bool start_flag = true;
	double track_time = 0;
	//	时间标志
};


int main(int argc, char *argv[]){
	std::cout << "Start chasing" << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uav_chase>());

    rclcpp::shutdown();
    return 0;
}
/**
 * @brief Send a command to control w and f
 */
 //发送命令控制无人机
void uav_chase::publish_rates_setpoint(double roll, double pitch, double yaw, double thrust)
{
    // 创建消息实例
	VehicleRatesSetpoint message{};
    message.roll = roll;
    message.pitch = pitch;
    message.yaw = yaw;
    message.thrust_body = {0,0,thrust};
    vehicle_rates_setpoint_publisher_->publish(message);
}



/**
 * @brief Send a command to Arm the vehicle
 */
 //发送命令对无人机进行解锁
void uav_chase::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
//发送命令使得无人机上锁
void uav_chase::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
//  发布控制模式，选择什么控制方式
void uav_chase::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	//	启用速度控制
	msg.position = true;
	//	禁用速度控制
	msg.velocity = false;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = false;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
//  发布控制模式，选择什么控制方式
void uav_chase::publish_offboard_control_mode_att()
{
	OffboardControlMode msg{};
	//	启用速度控制
	msg.position = false;
	//	禁用速度控制
	msg.velocity = false;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = true;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
//  发布设点模式 控制无人机的点位
void uav_chase::publish_trajectory_setpoint(float x,float y,float z,float yaw)
{
	TrajectorySetpoint msg{};
	//	使用的是FRD坐标系
	msg.position = {x,y,z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
//  发布速度模式，控制无人机的速度
void uav_chase::publish_trajectory_velocity(float x,float y,float z,float yaw)
{
	TrajectorySetpoint msg{};
	//	使用的是FRD坐标系
	msg.position = {nan(""),nan(""),nan("")};
	msg.velocity = {x,y,z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
//  发布无人机命令
void uav_chase:: publish_vehicle_command(uint16_t command, float param1, float param2)
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