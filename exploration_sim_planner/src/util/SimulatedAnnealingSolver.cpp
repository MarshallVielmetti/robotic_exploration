/**
 * @file SimulatedAnnealingSolver.cpp
 * @brief This file contains the implementation of the SimulatedAnnealingSolver
 * class
 */

#include <cmath>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <random>

#include "exploration_sim_planner/util/AtspSolver.hpp"

#define RAND_MAX_F 2147483647.f

std::vector<uint32_t> SimulatedAnnealingSolver::solve(
    const Eigen::MatrixXd &cost_matrix) {
  size_t n = cost_matrix.rows();
  auto current_solution = generate_initial_tour(n);

  double current_cost = calculate_cost(current_solution, cost_matrix);

  auto best_solution = current_solution;
  double best_cost = current_cost;

  double temperature = initial_temperature_;

  size_t num_iterations = 0;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  while (temperature > 1) {
    num_iterations++;
    std::vector<uint32_t> new_solution = get_neighbor(current_solution);

    double new_cost = calculate_cost(new_solution, cost_matrix);

    if (acceptance_probability(current_cost, new_cost, temperature) >
        dis(gen)) {
      current_solution = new_solution;
      current_cost = new_cost;
    }

    if (current_cost < best_cost) {
      best_solution = current_solution;
      best_cost = current_cost;
    }

    temperature *= cooling_rate_;
  }

  std::cout << "Simulated annealing found a solution with cost: " << best_cost
            << " after " << num_iterations << " iterations." << std::endl;

  return best_solution;
}

SimulatedAnnealingSolver::SimulatedAnnealingSolver(double initial_temperature,
                                                   double cooling_rate)
    : initial_temperature_(initial_temperature), cooling_rate_(cooling_rate) {}

double SimulatedAnnealingSolver::calculate_cost(
    const std::vector<uint32_t> &tour, const Eigen::MatrixXd &cost_matrix) {
  double cost = 0;

  for (size_t i = 0; i < tour.size() - 1; i++) {
    cost += cost_matrix(tour[i], tour[i + 1]);
  }

  // return cost -- SHOULD BE ZERO
  cost += cost_matrix(tour.back(), tour.front());
  return cost;
}

std::vector<uint32_t> SimulatedAnnealingSolver::generate_initial_tour(
    uint32_t num_nodes) {
  std::vector<uint32_t> initial_tour(num_nodes);

  // fill backwards so first node corresponds to the first node (which is the
  // last entry in the cost matrix)
  std::iota(initial_tour.rbegin(), initial_tour.rend(), 0);

  // don't shuffle the first node -- it is fixed
  std::shuffle(initial_tour.begin() + 1, initial_tour.end(),
               std::mt19937{std::random_device{}()});

  return initial_tour;
}

std::vector<uint32_t> SimulatedAnnealingSolver::get_neighbor(
    const std::vector<uint32_t> &tour) {
  std::vector<uint32_t> new_tour = tour;

  // neither can be the first node -- it is fixed
  size_t n = new_tour.size() - 1;
  uint32_t i = (rand() % n) + 1;
  uint32_t j = (rand() % n) + 1;

  std::swap(new_tour[i], new_tour[j]);
  return new_tour;
}

double SimulatedAnnealingSolver::acceptance_probability(double curr_cost,
                                                        double new_cost,
                                                        double temperature) {
  return (new_cost < curr_cost)
             ? 1.0
             : std::exp((curr_cost - new_cost) / temperature);
}
