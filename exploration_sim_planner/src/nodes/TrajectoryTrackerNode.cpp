/**
 * @file TrajectoryTrackerNode.cpp
 * @brief This file implements the TrajectoryTrackerNode class
 */

#include "exploration_sim_planner/nodes/TrajectoryTrackerNode.hpp"

#include <tf2/utils.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "exploration_sim_planner/trajectory_tracker/PurePursuitTracker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define LOOKAHEAD_DISTANCE 3.0f

TrajectoryTrackerNode::TrajectoryTrackerNode() : Node("trajectory_tracker") {
  RCLCPP_INFO(this->get_logger(), "Starting Trajectory Tracker Node");

  // Create a publisher for the steering angle
  steering_publisher_ = this->create_publisher<std_msgs::msg::Float64>("steering_angle", 10);

  lookahead_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 10);

  // Create a subscriber for the path
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "short_term_path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { this->current_path_ = *msg; });

  // Subscribe to the robot pose
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "robot_pose", 10, std::bind(&TrajectoryTrackerNode::pose_callback, this, std::placeholders::_1));

  tracker_ = std::make_shared<PurePursuitTracker>();
}

void TrajectoryTrackerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (current_path_.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path to follow");
    return;
  }

  std::vector<Eigen::Vector2d> path;
  for (auto& pose : current_path_.poses) {
    path.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
  }

  Eigen::Vector2d curr_pose = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);

  double curr_angle = tf2::getYaw(msg->pose.orientation);

  auto [steering_angle, target_point] =
      tracker_->compute_steering_angle(path, curr_pose, curr_angle, LOOKAHEAD_DISTANCE);

  std_msgs::msg::Float64 steering_msg;
  steering_msg.data = steering_angle;

  steering_publisher_->publish(steering_msg);

  geometry_msgs::msg::PointStamped target_point_msg;
  target_point_msg.point.x = target_point.x();
  target_point_msg.point.y = target_point.y();
  target_point_msg.point.z = 0.0;
  target_point_msg.header.frame_id = "diff_drive/odom";
  // target_point_msg.header.stamp = this->now();
  // target_point_msg.header = msg->header;

  lookahead_publisher_->publish(target_point_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTrackerNode>());
  rclcpp::shutdown();
  return 0;
}