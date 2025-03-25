/**
 * @file PurePursuitTracker.hpp
 * @brief This file defines the PurePursuitTracker class, which is used to
 * track a path using the pure pursuit algorithm
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "AbstractTrajectoryTracker.hpp"

class PurePursuitTracker : public AbstractTrajectoryTracker {
 public:
  virtual ~PurePursuitTracker();

  /**
   * @brief Compute the steering angle for the robot to follow the path
   *
   * @param path The path to follow
   * @param current_position The current position of the robot
   * @param lookahead_distance The distance ahead of the robot to look for the
   * target point
   * @return The steering angle in radians
   */
  static double compute_steering_angle(const std::vector<Eigen::Vector2d>& path,
                                       const Eigen::Vector2d& current_position,
                                       double lookahead_distance);

  /**
   * @brief Compute the target point for the robot to follow the path
   *
   * @param path The path to follow
   * @param current_position The current position of the robot
   * @param lookahead_distance The distance ahead of the robot to look for the
   * target point
   * @return The target point on the path
   */
  static Eigen::Vector2d compute_target_point(
      const std::vector<Eigen::Vector2d>& path,
      const Eigen::Vector2d& current_position, double lookahead_distance);
};