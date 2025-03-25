/**
 * @file TrajectoryTrackerNode.cpp
 * @brief This file implements the TrajectoryTrackerNode class
 */

#include "exploration_sim_planner/nodes/TrajectoryTrackerNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "exploration_sim_planner/trajectory_tracker/PurePursuitTracker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define LOOKAHEAD_DISTANCE 3.0

TrajectoryTrackerNode::TrajectoryTrackerNode() : Node("trajectory_tracker") {
  RCLCPP_INFO(this->get_logger(), "Starting Trajectory Tracker Node");

  // Create a publisher for the steering angle
  steering_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("steering_angle", 10);

  // Create a subscriber for the path
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        this->current_path_ = *msg;
      });

  // Subscribe to the robot pose
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "robot_pose", 10,
      std::bind(&TrajectoryTrackerNode::pose_callback, this,
                std::placeholders::_1));

  tracker_ = std::make_shared<PurePursuitTracker>();
}

void TrajectoryTrackerNode::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (current_path_.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path to follow");
    return;
  }

  std::vector<Eigen::Vector2d> path;
  for (auto& pose : current_path_.poses) {
    path.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
  }

  Eigen::Vector2d curr_pose =
      Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);

  auto steering_angle = tracker_->compute_steering_angle(path, curr_pose, 3.0);

  std_msgs::msg::Float64 steering_msg;
  steering_msg.data = steering_angle;

  steering_publisher_->publish(steering_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTrackerNode>());
  rclcpp::shutdown();
  return 0;
}