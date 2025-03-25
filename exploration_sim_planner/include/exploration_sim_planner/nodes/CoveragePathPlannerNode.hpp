/**
 * @file CoveragePathPlannerNode.hpp
 */
#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "exploration_sim_msgs/msg/connectivity_graph.hpp"
#include "exploration_sim_msgs/msg/frontier_clusters.hpp"
#include "exploration_sim_planner/coverage_planner/ConnectedComponentsLabeling.hpp"
#include "exploration_sim_planner/util/AtspSolver.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

class CoveragePathPlannerNode : public rclcpp::Node {
 public:
  CoveragePathPlannerNode();

 private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

 private:
  ConnectedComponentsLabeling connected_components_util_;

  geometry_msgs::msg::PoseStamped robot_pose_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<exploration_sim_msgs::msg::ConnectivityGraph>::SharedPtr
      graph_pub_;
  rclcpp::Publisher<exploration_sim_msgs::msg::FrontierClusters>::SharedPtr
      frontier_pub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr labels_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr zones_pub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::unique_ptr<ATSPSolver> tsp_solver_;
};