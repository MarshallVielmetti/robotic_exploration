/**
 * @file MPCTrajectoryTracker.cpp
 * @brief This file defines the MPCTrajectoryTracker class
 */

#include "exploration_sim_planner/trajectory_tracker/MPCTrajectoryTracker.hpp"

#include <Eigen/Dense>
#include <memory>

#include "exploration_sim_planner/util/MpcController.hpp"

MPCTrajectoryTracker::MPCTrajectoryTracker(double dt, double v_max, double omega_max) {
  // Initialize MPC parameters
  mpc_params_ = MpcController::MPCParameters();

  // Create MpcController instance
  mpc_controller_ = std::make_unique<MpcController>(dt, v_max, omega_max, mpc_params_);
}

void MPCTrajectoryTracker::set_parameters(const MpcController::MPCParameters& params) {
  mpc_params_ = params;
  if (mpc_controller_) {
    mpc_controller_->set_parameters(params);
  }
}

const MpcController::MPCParameters& MPCTrajectoryTracker::get_parameters() const {
  return mpc_controller_ ? mpc_controller_->get_parameters() : mpc_params_;
}

Eigen::Vector2d MPCTrajectoryTracker::compute_control(const Eigen::Vector2d& state, double theta,
                                                      std::vector<Eigen::Vector2d>& path) {
  // Call MpcController's compute_control directly
  auto res = mpc_controller_->compute_control(state, theta, v_ref_, path);
  v_ref_ = res[0];
  return res;
}
