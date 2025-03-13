/**
 * @file CoveragePathPlannerNode.hpp
 */
#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "exploration_sim_msgs/msg/connectivity_graph.hpp"
#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"

class CoveragePathPlannerNode : public rclcpp::Node {
 public:
  CoveragePathPlannerNode();

 private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

 private:
  ConnectedComponentsLabeling connected_components_util_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<exploration_sim_msgs::msg::ConnectivityGraph>::SharedPtr
      graph_pub_;
};