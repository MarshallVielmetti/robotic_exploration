/**
 * @file PurePursuitTracker.cpp
 * @brief This file implements the PurePursuitTracker class defined in
 * PurePursuitTracker.hpp
 */

#include "exploration_sim_planner/trajectory_tracker/PurePursuitTracker.hpp"

#include <limits>

PurePursuitTracker::~PurePursuitTracker() {}

double PurePursuitTracker::compute_steering_angle(
    const std::vector<Eigen::Vector2d> &path,
    const Eigen::Vector2d &current_position, double lookahead_distance) {
  // Find the target point
  auto target_point =
      compute_target_point(path, current_position, lookahead_distance);

  // Compute the steering angle
  auto diff = target_point - current_position;
  return std::atan2(diff.y(), diff.x());
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
  Eigen::Vector2d best_lookahead = closest_point;
  double best_distance = lookahead_distance;

  for (size_t i = point_index + 1; i < path.size(); i++) {
    auto cand = path[i];
    double diff = (cand - current_position).norm() - lookahead_distance;
    if (diff < best_distance) {
      best_distance = diff;
      best_lookahead = cand;
    }
  }

  return best_lookahead;
}
