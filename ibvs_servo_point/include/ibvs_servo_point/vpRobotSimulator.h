#ifndef vpROSRobot_H
#define vpROSRobot_H

/*!
\file vpROSRobot.h
\brief vpRobot implementation for ROS2 middleware.
*/

#include <visp3/robot/vpRobot.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>

/*!
\class vpROSRobot
\brief Interface for robots based on ROS2.
*/

class VISP_EXPORT vpRobotSimulator : public vpRobot
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> executor_thread_;

  bool isInitialized;

  vpQuaternionVector q;
  vpTranslationVector p;
  vpColVector pose_prev;
  vpColVector displacement;
  uint32_t _sec, _nsec;
  std::mutex odom_mutex_;
  std::string _master_uri;
  std::string _topic_cmd;
  std::string _topic_odom;
  std::string _nodespace;

private:
  // Disable copy constructor and assignment operator
  vpRobotSimulator(const vpRobotSimulator &) = delete;
  vpRobotSimulator &operator=(const vpRobotSimulator &) = delete;

  void get_eJe(vpMatrix &eJe) override {}
  void get_fJe(vpMatrix &fJe) override {}
  void getArticularDisplacement(vpColVector &qdot) {}

  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);

  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q){}

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void getCameraDisplacement(vpColVector &v);

public:
  vpRobotSimulator();
  virtual ~vpRobotSimulator();

  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) override;
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q, struct timespec &timestamp);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);

  void init();
  void init(int argc, char **argv);

  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel) override;
  void stopMotion();
  void setCmdVelTopic(const std::string &topic_name);
  void setOdomTopic(const std::string &topic_name);
  void setMasterURI(const std::string &master_uri);
  void setNodespace(const std::string &nodespace);
};

#endif