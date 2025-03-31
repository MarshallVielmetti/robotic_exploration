/**
 * @file DubinsPath.hpp
 * @brief Defines functions to compute the Dubins path between two states
 */

#pragma once

#include <Eigen/Dense>
#include <optional>

namespace DubinsPath {

enum class DubinsPathType { LSL, LSR, RSL, RSR, RLR, LRL };

struct DubinsPath {
  DubinsPathType type;
  double length;
  double radius;
  double length1;
  double length2;
  double length3;
  double angle1;
  double angle2;
  double angle3;
};

/**
 * @brief Compute the Dubins path between two states
 * @param start The start state
 * @param end The end state
 * @param radius The radius of the vehicle
 * @return The Dubins path
 */
std::optional<DubinsPath> compute_dubins_path(Eigen::Vector3d start, Eigen::Vector3d end, double radius);

/**
 * @brief Interpolates the Dubins path into a series of points
 * @param path The Dubins path
 * @param step_size The distance between points
 * @return A vector of points (and omega values) along the Dubins path
 */
std::vector<std::pair<Eigen::Vector3d, double>> compute_dubins_path_points(const DubinsPath& path, double step_size);

}  // namespace DubinsPath