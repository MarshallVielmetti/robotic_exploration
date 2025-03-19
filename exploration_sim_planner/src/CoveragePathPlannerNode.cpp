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

#include <rclcpp/logging.hpp>

#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"
#include "exploration_sim_planner/util/MsgUtil.hpp"

#define DEBUG_MODE 1

CoveragePathPlannerNode::CoveragePathPlannerNode()
    : Node("coverage_path_planner"), connected_components_util_(10, 1) {
  RCLCPP_INFO(get_logger(),
              "Coverage Path Planner Node has started, configured with sector "
              "size %d and safe distance %d.",
              10, 1);

  // Subscribe to the map topic
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&CoveragePathPlannerNode::map_callback, this,
                std::placeholders::_1));

  // Create a publisher for the connectivity graph
  graph_pub_ = create_publisher<exploration_sim_msgs::msg::ConnectivityGraph>(
      "connectivity_graph", 10);

  labels_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("cell_labels", 10);

  zones_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("zones", 10);

  frontier_pub_ = create_publisher<exploration_sim_msgs::msg::FrontierClusters>(
      "frontier_clusters", 10);
}

void CoveragePathPlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Received a new map message.");

  // Create an OgmView object to allow interacting with the OGM
  auto ogm_view = std::make_shared<OgmView>(msg);

  RCLCPP_DEBUG(get_logger(), "Received OGM with dimensions: %dx%d",
               ogm_view->width(), ogm_view->height());

  // Label the cells in the map
  auto cell_labels = connected_components_util_.label_cells(*ogm_view);

  // publish the cell labels if anyone is subscribed to the topic
  if (labels_pub_->get_subscription_count() > 0 || DEBUG_MODE) {
    RCLCPP_DEBUG(get_logger(), "Publishing cell labels.");
    auto labels_msg = msg_util::matrix_to_occupancy_grid(cell_labels, msg);
    labels_pub_->publish(labels_msg);
  }

  // get a list of frontier cells
  auto frontier_cells =
      connected_components_util_.find_frontier_cells(cell_labels);

  // Run set union algorithm on the frontier cells
  auto frontier_clusters =
      connected_components_util_.cluster_frontiers(frontier_cells);

  auto frontier_viewpoints =
      connected_components_util_.sample_frontier_viewpoints(frontier_clusters,
                                                            cell_labels);

  // Publish the frontier clusters if anyone is subscribed to the topic
  if (frontier_pub_->get_subscription_count() > 0 || DEBUG_MODE) {
    RCLCPP_DEBUG(get_logger(), "Publishing frontier clusters.");
    auto frontier_msg = msg_util::frontier_clusters_to_msg(frontier_clusters,
                                                           frontier_viewpoints);
    frontier_msg.header = msg->header;
    frontier_msg.info = msg->info;
    frontier_pub_->publish(frontier_msg);
  }

  // Compute zones based on the cell labels
  auto zones = connected_components_util_.compute_zones(cell_labels);

  // publish the zones if anyone is subscribed to the topic
  if (zones_pub_->get_subscription_count() > 0 || DEBUG_MODE) {
    RCLCPP_DEBUG(get_logger(), "Publishing cell zones.");
    auto zones_msg = msg_util::matrix_to_occupancy_grid(zones, msg);
    zones_pub_->publish(zones_msg);
  }

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