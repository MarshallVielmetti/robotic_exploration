/**
 * @file DiffDriveController.hpp
 * @brief Simple differential drive controller
 */

#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64.hpp"

class DiffDriveController : public rclcpp::Node {
 public:
  DiffDriveController();

 private:
  //   void control_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void angular_callback(const std_msgs::msg::Float64 msg);
  void publish_velocity();

 private:
  //   Eigen::Vector2d velocity_setpoint_;  // linear and angular velocity
  //   setpoint
  Eigen::Vector2d
      control_value_;  // linear and angular acceleration control value

  // ROS2 publishers and subscribers
  //   std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>>
  //       control_subscriber_;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>
      cmd_vel_publisher_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angular_sub_;

  // timer for control update frequency
  rclcpp::TimerBase::SharedPtr timer_;

  static const std::chrono::milliseconds CONTROL_FREQUENCY;
  static const double CONTROL_FREQUENCY_SEC;
};