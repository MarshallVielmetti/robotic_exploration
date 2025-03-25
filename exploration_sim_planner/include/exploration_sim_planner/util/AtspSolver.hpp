/**
 * @file AtspSolver.hpp
 * @brief This file contains the definition of the ATSPSolver interface
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

class ATSPSolver {
 public:
  ATSPSolver() = default;
  virtual ~ATSPSolver() = default;

  virtual std::vector<uint32_t> solve(const Eigen::MatrixXd& cost_matrix) = 0;
};

class SimulatedAnnealingSolver : public ATSPSolver {
 public:
  SimulatedAnnealingSolver(double initial_temperature, double cooling_rate);

  std::vector<uint32_t> solve(const Eigen::MatrixXd& cost_matrix) override;

 private:
  static double calculate_cost(const std::vector<uint32_t>& tour,
                               const Eigen::MatrixXd& cost_matrix);

  static std::vector<uint32_t> generate_initial_tour(uint32_t num_nodes);

  static std::vector<uint32_t> get_neighbor(const std::vector<uint32_t>& tour);

  static double acceptance_probability(double curr_cost, double new_cost,
                                       double temperature);

 private:
  const double initial_temperature_;
  const double cooling_rate_;
};