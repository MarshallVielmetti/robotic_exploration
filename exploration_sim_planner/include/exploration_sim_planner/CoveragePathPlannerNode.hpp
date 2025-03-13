/**
 * @file CoveragePathPlannerNode.hpp
 */
#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

class CoveragePathPlannerNode : public rclcpp::Node {
 public:
  CoveragePathPlannerNode();

 private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

 private:
};