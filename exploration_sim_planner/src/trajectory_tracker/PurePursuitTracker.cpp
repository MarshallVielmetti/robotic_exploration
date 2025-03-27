/**
 * @file PurePursuitTracker.cpp
 * @brief This file implements the PurePursuitTracker class defined in
 * PurePursuitTracker.hpp
 */

#include "exploration_sim_planner/trajectory_tracker/PurePursuitTracker.hpp"

#include <limits>

#define WHEELBASE 1.0f

PurePursuitTracker::~PurePursuitTracker() {}

std::pair<double, Eigen::Vector2d> PurePursuitTracker::compute_steering_angle(
    const std::vector<Eigen::Vector2d> &path,
    const Eigen::Vector2d &current_position, double current_heading,
    double lookahead_distance) {
  // Find the target point
  auto target_point =
      compute_target_point(path, current_position, lookahead_distance);

  double dx = target_point.x() - current_position.x();
  double dy = target_point.y() - current_position.y();

  // double target_x_local =
  //     dx * cos(-current_heading) - dy * sin(-current_heading);
  double target_y_local =
      dx * sin(-current_heading) + dy * cos(-current_heading);

  double curvature =
      2.0 * target_y_local / (lookahead_distance * lookahead_distance);

  double steering_angle = atan(WHEELBASE * curvature);

  return std::make_pair(steering_angle, target_point);
}

Eigen::Vector2d PurePursuitTracker::compute_target_point(
    const std::vector<Eigen::Vector2d> &path,
    const Eigen::Vector2d &current_position, double lookahead_distance) {
  // Find the closest point on the path to the current position
  double min_distance = std::numeric_limits<double>::max();
  Eigen::Vector2d closest_point;
  size_t point_index = 0;
  for (size_t i = 0; i < path.size(); i++) {
    double distance = (path[i] - current_position).norm();
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = path[i];
      point_index = i;
    }
  }

  // Find the target point by sampling points further along the path until
  // the lookahead distance is reached
  double path_distance = 0.0;

  for (size_t i = point_index + 1; i < path.size(); i++) {
    path_distance += (path[i] - path[i - 1]).norm();
    if (path_distance > lookahead_distance) {
      return path[i];
    }
  }

  // If the lookahead distance is greater than the path length, return the last
  // point on the path
  return path.back();
}
