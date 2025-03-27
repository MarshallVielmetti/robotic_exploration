/**
 * @file mpc_test.cpp
 * @brief This file contains tests for the MPC controller
 */

#include <cstddef>
#include <iostream>

#include "exploration_sim_planner/util/MpcController.hpp"

int main() {
  MpcController::MPCParameters params;

  params.horizon = 10;
  params.Q_x = 10.0;
  params.Q_y = 10.0;
  params.Q_theta = 5.0;
  params.R_v = 1.0;
  params.R_omega = 1.0;

  size_t horizon = 10;
  double dt = 0.1;
  double v_max = 1.0;
  double omega_max = 1.0;

  MpcController mpc(dt, v_max, omega_max, params);

  Eigen::Vector2d state(0.0, 0.0);
  double theta = 0.0;

  // Create a simple reference trajectory
  std::vector<Eigen::Vector2d> path;
  for (size_t i = 0; i < 1000 * horizon; i++) {
    path.push_back(Eigen::Vector2d(0.5 * static_cast<double>(i + 1), 0.5 * static_cast<double>(i + 1)));
  }

  size_t current_idx = 0;
  for (size_t iter = 0; iter < 100; iter++) {
    auto subpath = std::vector<Eigen::Vector2d>(path.begin() + current_idx, path.begin() + current_idx + horizon);
    Eigen::Vector2d control = mpc.compute_control(state, theta, 0, subpath);

    std::cout << "Computed Control: v = " << control[0] << ", omega = " << control[1] << std::endl;

    state[0] += control[0] * std::cos(theta) * dt;
    state[1] += control[0] * std::sin(theta) * dt;
    theta += control[1] * dt;
    // Wrap theta to [-pi, pi]
    theta = std::fmod(theta + M_PI, 2 * M_PI) - M_PI;

    // Update the reference trajectory to start from the new state
    // Find the closest point in the path -- when distance
    // starts increasing, we've passed the point
    double min_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    for (size_t i = current_idx; i < path.size(); i++) {
      double dx = path[i][0] - state[0];
      double dy = path[i][1] - state[1];
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        min_idx = i;
      } else {
        break;
      }
    }

    current_idx = min_idx;

    std::cout << "New State: x = " << state[0] << ", y = " << state[1] << ", theta = " << theta << std::endl;
  }

  Eigen::Vector2d control = mpc.compute_control(state, theta, 0, path);

  std::cout << "Computed Control: v = " << control[0] << ", omega = " << control[1] << std::endl;

  // pretty pritn the solution vector
  // for (size_t i = 0; i < 30; i += 3) {
  //   std::cout << "X: " << mpc.solver_.getSolution()[i] << " Y: " << mpc.solver_.getSolution()[i + 1]
  //             << " Theta: " << mpc.solver_.getSolution()[i + 2] << std::endl;
  // }

  // for (size_t i = 30; i < 50; i += 2) {
  //   std::cout << "V: " << mpc.solver_.getSolution()[i] << " Omega: " << mpc.solver_.getSolution()[i + 1] <<
  //   std::endl;
  // }

  return 0;
}
