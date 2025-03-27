/**
 * @file EsdfUtil.cpp
 *
 * @brief Implementation of Euclidean Signed Distance Field utilities
 */

#include "exploration_sim_planner/util/EsdfUtil.hpp"

#include <limits>
#include <queue>
#include <vector>

namespace EsdfUtil {

Eigen::MatrixXd computeEsdf(const Eigen::MatrixXd& ogm, double max_distance) {
  const int rows = ogm.rows();
  const int cols = ogm.cols();

  // Initialize ESDF with large values for free space
  // and zero for obstacles
  Eigen::MatrixXd esdf(rows, cols);
  Eigen::MatrixXi visited = Eigen::MatrixXi::Zero(rows, cols);

  // Priority queue for wavefront expansion
  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> queue;

  // Initialize ESDF and queue with obstacle cells
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (ogm(y, x) == 100) {  // Obstacle cell
        esdf(y, x) = 0;
        visited(y, x) = 1;

        // Add neighbors to the queue
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;

            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows &&
                ogm(ny, nx) != 100) {
              double dist = std::sqrt(dx * dx + dy * dy);
              queue.push({nx, ny, dist});
            }
          }
        }
      } else {
        esdf(y, x) = std::numeric_limits<double>::infinity();
      }
    }
  }

  // 8-connected neighborhood offsets for wavefront expansion
  const std::vector<std::pair<int, int>> offsets = {
      {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

  // Process queue in order of increasing distance
  while (!queue.empty()) {
    Cell current = queue.top();
    queue.pop();

    // Skip if this cell has already been processed with a better distance
    if (visited(current.y, current.x)) {
      continue;
    }

    // Mark as visited and set distance
    visited(current.y, current.x) = 1;
    esdf(current.y, current.x) = current.distance;

    // Don't expand beyond max_distance
    if (current.distance >= max_distance) {
      continue;
    }

    // Expand to neighbors
    for (const auto& offset : offsets) {
      int nx = current.x + offset.first;
      int ny = current.y + offset.second;

      if (nx >= 0 && nx < cols && ny >= 0 && ny < rows && !visited(ny, nx)) {
        // Calculate Euclidean distance from new cell to closest obstacle
        // by considering distance from current cell plus the step
        double step_dist = std::sqrt(offset.first * offset.first +
                                     offset.second * offset.second);
        double new_dist = current.distance + step_dist;

        // If this new path gives a better distance estimate, update the queue
        if (new_dist < esdf(ny, nx)) {
          esdf(ny, nx) = new_dist;
          queue.push({nx, ny, new_dist});
        }
      }
    }
  }

  // Fill any remaining unvisited cells with max_distance
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (!visited(y, x)) {
        esdf(y, x) = max_distance;
      }
    }
  }

  return esdf;
}

bool isSafe(const Eigen::MatrixXd& esdf, const Eigen::Vector2d& point,
            double safety_distance) {
  int x = static_cast<int>(std::round(point.x()));
  int y = static_cast<int>(std::round(point.y()));

  if (x < 0 || y < 0 || x >= esdf.cols() || y >= esdf.rows()) {
    return false;  // Out of bounds is not safe
  }

  return esdf(y, x) >= safety_distance;
}

}  // namespace EsdfUtil
