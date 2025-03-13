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
#include "exploration_sim_msgs/msg/point2d.hpp"
#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"

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
      if (graph.edges(i, j).type != EdgeType::INVALID) {
        msg.edges[i * graph.edges.cols() + j] = edge_to_msg(graph.edges(i, j));
      }
    }
  }

  return msg;
}
}  // namespace msg_util