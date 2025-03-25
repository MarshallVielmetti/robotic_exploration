#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PoseTransformListenerNode : public rclcpp::Node {
public:
  PoseTransformListenerNode();

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
