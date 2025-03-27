/**
 * @file EsdfUtil.hpp
 *
 * @brief Utility functions for working with Euclidean Signed Distance Fields
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace EsdfUtil {

/**
 * @brief Convert an occupancy grid map (OGM) to an Euclidean Signed Distance
 * Field (ESDF)
 *
 * @param ogm Occupancy grid map where 0 is free space and 100 is obstacle
 * @param max_distance Maximum distance value to compute (limits computation)
 * @return Eigen::MatrixXd ESDF matrix where values represent distance to
 * nearest obstacle
 */
Eigen::MatrixXd computeEsdf(const Eigen::MatrixXd& ogm,
                            double max_distance = 50.0);

/**
 * @brief Checks if a point is within safe distance from obstacles
 *
 * @param esdf The Euclidean signed distance field
 * @param point The point to check
 * @param safety_distance Minimum required distance from obstacles
 * @return bool True if the point is safe
 */
bool isSafe(const Eigen::MatrixXd& esdf, const Eigen::Vector2d& point,
            double safety_distance);

/**
 * @struct Cell
 * @brief Structure to represent a cell in the wavefront expansion algorithm
 */
struct Cell {
  int x;
  int y;
  double distance;

  // Operator for priority queue (min heap)
  bool operator>(const Cell& other) const { return distance > other.distance; }
};

}  // namespace EsdfUtil
