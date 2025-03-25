/**
 * @file TrajectoryTrackerNode.hpp
 * @brief This file defines the TrajectoryTrackerNode class
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float64.hpp>

#include "exploration_sim_planner/trajectory_tracker/PurePursuitTracker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

class TrajectoryTrackerNode : public rclcpp::Node {
 public:
  TrajectoryTrackerNode();

 private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

 private:
  // members
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_publisher_;

  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::PoseStamped current_pose_;

  std::shared_ptr<PurePursuitTracker> tracker_;
};