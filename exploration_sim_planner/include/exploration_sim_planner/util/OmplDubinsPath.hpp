/**
 * @file OmplDubinsPath.hpp
 * @brief DubinsPath solver implemented with Ompl
 */

#pragma once

#include <ompl/base/spaces/DubinsStateSpace.h>

#include <Eigen/Dense>
#include <optional>
#include <vector>

class OmplDubinsPath {
 public:
  static std::optional<std::vector<Eigen::Vector3d>> compute_dubins_path(const Eigen::Vector3d& start,
                                                                         const Eigen::Vector3d& goal,
                                                                         double sample_spacing = 0.2) {
    ompl::base::DubinsStateSpace dubinsSpace(1.0);

    auto start_state = dubinsSpace.allocState();
    start_state->as<ompl::base::DubinsStateSpace::StateType>()->setXY(start.x(), start.y());
    start_state->as<ompl::base::DubinsStateSpace::StateType>()->setYaw(start.z());

    auto goal_state = dubinsSpace.allocState();
    goal_state->as<ompl::base::DubinsStateSpace::StateType>()->setXY(goal.x(), goal.y());
    goal_state->as<ompl::base::DubinsStateSpace::StateType>()->setYaw(goal.z());

    auto path = dubinsSpace.dubins(start_state, goal_state);

    auto sampled_point = dubinsSpace.allocState();
    double num_samples = path.length() / sample_spacing;

    if (path.length() > 99999) {
      return std::nullopt;
    }

    std::vector<Eigen::Vector3d> dubins_path;
    dubins_path.reserve(num_samples);

    for (double t = 0; t <= 1.0; t += 1.0 / num_samples) {
      dubinsSpace.interpolate(start_state, goal_state, t, sampled_point);
      auto temp = sampled_point->as<ompl::base::DubinsStateSpace::StateType>();
      dubins_path.emplace_back(temp->getX(), temp->getY(), temp->getYaw());
    }

    dubinsSpace.freeState(start_state);
    dubinsSpace.freeState(goal_state);
    dubinsSpace.freeState(sampled_point);

    return dubins_path;
  }
};