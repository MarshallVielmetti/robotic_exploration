/**
 * @file ShortTermPlannerNodel.hpp
 * @brief This file defines the ShortTermPlannerNode class
 */

#pragma once

#include <Eigen/Dense>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include "exploration_sim_planner/util/OgmView.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

class ShortTermPlannerNode : public rclcpp::Node {
 public:
  ShortTermPlannerNode();

 private:
  void coverage_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool is_safe(const nav_msgs::msg::Path &path);

 private:
  // members
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr coverage_path_sub_;
  // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
  //     robot_position_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr short_term_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr committed_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr backup_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr backup_target_pub_;

  // When this timer triggers, replace the committed path with the current
  // candidate
  rclcpp::TimerBase::SharedPtr committed_path_timer_;

  // At every iteration of this timer, verify the safety of the committed path
  // with new sensor data
  rclcpp::TimerBase::SharedPtr safety_timer_;

  Eigen::MatrixXd curr_esdf_;  // TODO This is not actually an ESDF rn
  std::vector<Eigen::Vector2d> current_coverage_path_;
  std::shared_ptr<OgmView> ogm_;

  nav_msgs::msg::Path committed_path_;
  nav_msgs::msg::Path current_candidate_;
  // Eigen::Vector2d current_robot_position_;
};