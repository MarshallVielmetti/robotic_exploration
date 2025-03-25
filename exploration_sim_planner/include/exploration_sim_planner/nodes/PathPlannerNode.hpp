#pragma once

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "exploration_sim_planner/path_planner/AbstractPathPlanner.hpp"

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode();

private:
  // only changes to the goal will trigger replanning for now
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Replan the path using the path planner, and publish the new path
  void trigger_replanning();

private:
  // Path planner
  std::shared_ptr<AbstractPathPlanner> path_planner;

  // Current State
  geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;

  // ROS2 publishers and subscribers
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_publisher_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>>
      map_subscription_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>
      goal_subscription_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>
      pose_subscription_;
};