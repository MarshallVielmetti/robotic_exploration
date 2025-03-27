/**
 * @file MPCTrajectoryTracker.hpp
 * @brief This file declares the MPCTrajectoryTracker class
 */

#pragma once

#include <osqp/osqp.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "exploration_sim_planner/util/MpcController.hpp"

class MPCTrajectoryTracker {
 public:
  /**
   * @brief Construct a new MPCTrajectoryTracker object
   * @param wheelbase Wheelbase
   * @param dt Time step
   * @param v_max Maximum linear velocity
   * @param omega_max Maximum angular velocity
   */
  MPCTrajectoryTracker(double dt, double v_max, double omega_max);

  /**
   * @brief Computes control commands using Model Predictive Control to follow a
   * trajectory
   *
   * This method generates control commands to guide the robot along the
   * provided path using the current state and orientation as the starting
   * point. It applies MPC optimization techniques to determine optimal control
   * inputs.
   *
   * @param state Current robot state as a 2D vector (typically x,y position)
   * @param theta Current robot orientation in radians
   * @param path Reference trajectory to follow, provided as a vector of 2D
   * points
   *
   * @return Eigen::Vector2d Control commands (linear and angular velocity)
   */
  Eigen::Vector2d compute_control(const Eigen::Vector2d& state, double theta, std::vector<Eigen::Vector2d>& path);

  /**
   * @brief Set MPC parameters
   * @param params New MPC parameters
   */
  void set_parameters(const MpcController::MPCParameters& params);

  /**
   * @brief Get current MPC parameters
   * @return Current MPC parameters
   */
  const MpcController::MPCParameters& get_parameters() const;

 private:
  // MPC controller instance for reusing state between solver runs
  std::unique_ptr<MpcController> mpc_controller_;

  // MPC parameters
  MpcController::MPCParameters mpc_params_;

  double v_ref_ = 0.0;
};