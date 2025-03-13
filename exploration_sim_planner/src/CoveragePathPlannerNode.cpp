/**
 * @file CoveragePathPlannerNode.cpp
 *
 * This file implements the CoveragePathPlannerNode class defined in
 * CoveragePathPlannerNode.hpp.
 *
 * The primary implementation details are contained in
 * @see ConnectedComponentsLabeling.hpp
 */

#include "exploration_sim_planner/CoveragePathPlannerNode.hpp"

#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"
#include "exploration_sim_planner/util/MsgUtil.hpp"

CoveragePathPlannerNode::CoveragePathPlannerNode()
    : Node("coverage_path_planner"), connected_components_util_(10, 1) {
  RCLCPP_INFO(get_logger(), "Coverage Path Planner Node has started.");

  // Subscribe to the map topic
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&CoveragePathPlannerNode::map_callback, this,
                std::placeholders::_1));

  // Create a publisher for the connectivity graph
  graph_pub_ = create_publisher<exploration_sim_msgs::msg::ConnectivityGraph>(
      "connectivity_graph", 10);
}

void CoveragePathPlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Received a new map message.");

  // Create an OgmView object to allow interacting with the OGM
  auto ogm_view = std::make_shared<OgmView>(msg);

  // Label the cells in the map
  auto cell_labels = connected_components_util_.label_cells(*ogm_view);

  // Compute zones based on the cell labels
  auto zones = connected_components_util_.compute_zones(cell_labels);

  // Find the centers of the zones
  auto centers = connected_components_util_.find_centers(cell_labels, zones);

  // Compute the incremental connectivity graph
  auto graph =
      connected_components_util_.compute_incremental_connectivity_graph(
          cell_labels, centers);

  // Use the OgmView to modify the locations of the centers to be global
  // coordinates instead of relative to the OGM
  std::transform(graph.nodes.begin(), graph.nodes.end(), graph.nodes.begin(),
                 [&ogm_view](const Eigen::Vector2d& center) {
                   return ogm_view->cell_to_world(center);
                 });

  // Publish the connectivity graph
  auto graph_msg = msg_util::graph_to_msg(graph);

  graph_pub_->publish(graph_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoveragePathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}