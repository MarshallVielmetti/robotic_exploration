/**
 * @file MsgUtil.hpp
 *
 * @brief This file contains utility functions for converting between some
 * custom types defined in this package and custom ros messages
 */
#pragma once

#include <Eigen/Dense>

#include "exploration_sim_msgs/msg/connectivity_graph.hpp"
#include "exploration_sim_msgs/msg/edge.hpp"
#include "exploration_sim_msgs/msg/frontier_clusters.hpp"
#include "exploration_sim_msgs/msg/point2d.hpp"
#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace msg_util {

inline exploration_sim_msgs::msg::Point2d to_point2d(
    const Eigen::Vector2d& point) {
  exploration_sim_msgs::msg::Point2d msg;
  msg.x = point.x();
  msg.y = point.y();
  return msg;
}

inline exploration_sim_msgs::msg::Edge edge_to_msg(const Edge& edge) {
  exploration_sim_msgs::msg::Edge msg;
  msg.label = static_cast<uint8_t>(edge.type);
  msg.cost = edge.cost;
  return msg;
}

inline exploration_sim_msgs::msg::ConnectivityGraph graph_to_msg(
    const ConnectivityGraph& graph) {
  exploration_sim_msgs::msg::ConnectivityGraph msg;
  for (const auto& node : graph.nodes) {
    msg.nodes.push_back(to_point2d(node));
  }

  msg.rows = graph.edges.rows();
  msg.edges.resize(graph.edges.rows() * graph.edges.cols());
  msg.num_edges = msg.edges.size();

  for (uint32_t i = 0; i < graph.edges.rows(); i++) {
    for (uint32_t j = 0; j < graph.edges.cols(); j++) {
      msg.edges[i * graph.edges.cols() + j] = edge_to_msg(graph.edges(i, j));
      // if (graph.edges(i, j).type != EdgeType::INVALID) {
      //   msg.edges[i * graph.edges.cols() + j] = edge_to_msg(graph.edges(i,
      //   j));
      // }
    }
  }

  return msg;
}

inline nav_msgs::msg::OccupancyGrid matrix_to_occupancy_grid(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& matrix,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& ogm_reference_frame) {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header = ogm_reference_frame->header;

  msg.info.width = matrix.cols();
  msg.info.height = matrix.rows();
  msg.info.origin = ogm_reference_frame->info.origin;
  msg.info.resolution = ogm_reference_frame->info.resolution;

  msg.data.resize(msg.info.width * msg.info.height);

  for (uint32_t y = 0; y < matrix.rows(); y++) {
    for (uint32_t x = 0; x < matrix.cols(); x++) {
      msg.data[y * matrix.cols() + x] = static_cast<int8_t>(matrix(y, x));
    }
  }

  return msg;
}

inline nav_msgs::msg::OccupancyGrid matrix_to_occupancy_grid(
    const Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>& matrix,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& ogm_reference_frame) {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header = ogm_reference_frame->header;

  msg.info.width = matrix.cols();
  msg.info.height = matrix.rows();
  msg.info.origin = ogm_reference_frame->info.origin;
  msg.info.resolution = ogm_reference_frame->info.resolution;

  msg.data.resize(msg.info.width * msg.info.height);

  for (uint32_t y = 0; y < matrix.rows(); y++) {
    for (uint32_t x = 0; x < matrix.cols(); x++) {
      msg.data[y * matrix.cols() + x] = static_cast<int8_t>(matrix(y, x));
    }
  }

  return msg;
}

inline exploration_sim_msgs::msg::FrontierClusters frontier_clusters_to_msg(
    const std::vector<std::vector<Eigen::Vector2i>>& clusters) {
  exploration_sim_msgs::msg::FrontierClusters msg;

  for (const auto& cluster : clusters) {
    exploration_sim_msgs::msg::FrontierCluster frontier_cluster;
    for (const auto& point : cluster) {
      frontier_cluster.points.push_back(to_point2d(point.cast<double>()));
    }
    msg.clusters.push_back(frontier_cluster);
  }

  return msg;
}

}  // namespace msg_util