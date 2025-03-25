/**
 * @file ShortTermPlannerNode.cpp
 * @brief This file implements the ShortTermPlannerNode class
 */

#include "exploration_sim_planner/nodes/ShortTermPlannerNode.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include "exploration_sim_planner/short_term_planner/ShortTermPathPlanner.hpp"

using namespace std::chrono_literals;

ShortTermPlannerNode::ShortTermPlannerNode() : Node("short_term_planner_node") {
  RCLCPP_INFO(this->get_logger(), "ShortTermPlannerNode node started");

  coverage_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "coverage_path", 10,
      std::bind(&ShortTermPlannerNode::coverage_path_callback, this,
                std::placeholders::_1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&ShortTermPlannerNode::map_callback, this,
                std::placeholders::_1));

  short_term_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("short_term_path", 10);

  committed_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("committed_path", 10);

  committed_path_timer_ = this->create_wall_timer(5s, [this]() {
    RCLCPP_INFO(this->get_logger(), "Replanning committed path");
    this->committed_path_ = current_candidate_;

    for (auto &pose : committed_path_.poses) {
      pose.header.stamp = rclcpp::Clock().now();
    }

    this->committed_path_.header.stamp = rclcpp::Clock().now();

    committed_path_pub_->publish(committed_path_);
  });

  safety_timer_ = this->create_wall_timer(0.1s, [this]() {
    if (committed_path_.poses.empty()) {
      return;
    }

    return;

    // Check if the current candidate path is safe
    if (ShortTermPathPlanner::check_path_safety(curr_esdf_, committed_path_)) {
      return;
    }

    // If not, replan
    RCLCPP_WARN(this->get_logger(), "Current candidate path is not safe");

    this->committed_path_ = current_candidate_;
    this->committed_path_.header.stamp = rclcpp::Clock().now();
    committed_path_pub_->publish(committed_path_);
  });
}

void ShortTermPlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store an OgmView -- used for transformations
  ogm_ = std::make_shared<OgmView>(msg);

  RCLCPP_DEBUG(this->get_logger(), "Received map");

  Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
      map_view(reinterpret_cast<const int8_t *>(msg->data.data()),
               msg->info.height, msg->info.width);

  curr_esdf_ = map_view.cast<double>();
}

void ShortTermPlannerNode::coverage_path_callback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  // Verify there is a valid map
  if (!ogm_) {
    RCLCPP_WARN(this->get_logger(),
                "Received coverage path before map, ignoring");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Received coverage path");

  // Every time a new coverage path is received, replan the short term path
  current_coverage_path_.clear();

  // Transform path to a vector of Eigen::Vector2d in map coordinates
  for (const auto &pose : msg->poses) {
    Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    auto map_point = ogm_->world_to_map(point);
    current_coverage_path_.push_back(map_point);
  }

  auto path = ShortTermPathPlanner::fit_smoothed_path(curr_esdf_,
                                                      current_coverage_path_);

  // Convert the path to a nav_msgs::Path message in
  auto path_msg = nav_msgs::msg::Path();
  // std::vector<geometry_msgs::msg::PoseStamped> path_msg(path.size());
  std::transform(path.begin(), path.end(), std::back_inserter(path_msg.poses),
                 [this, &msg](const Eigen::Vector2d &point) {
                   auto world_frame = ogm_->cell_to_world(point);
                   geometry_msgs::msg::PoseStamped pose;
                   pose.pose.position.x = world_frame.x();
                   pose.pose.position.y = world_frame.y();
                   pose.header = msg->header;
                   return pose;
                 });

  // Store the path for future reference
  // Publish the path
  path_msg.header = msg->header;
  current_candidate_ = path_msg;
  short_term_path_pub_->publish(path_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShortTermPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}