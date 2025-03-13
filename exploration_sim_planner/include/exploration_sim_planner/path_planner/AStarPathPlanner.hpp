#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "exploration_sim_planner/path_planner/AbstractPathPlanner.hpp"
#include "exploration_sim_planner/util/OgmView.hpp"

namespace std {
template <>
struct hash<Eigen::Vector2i> {
  std::size_t operator()(const Eigen::Vector2i &v) const {
    std::size_t h1 = std::hash<int>()(v.x());
    std::size_t h2 = std::hash<int>()(v.y());
    return h1 ^ (h2 << 1);
  }
};
}  // namespace std

/**
 * @brief Utility structures for the A* path planning algorithm.
 *
 * The namespace provides structures that allow efficient node management for
 * the planner.
 */

namespace PathPlannerUtil {
/**
 * @brief Represents a node in the priority queue used by the A* algorithm.
 *
 * Each node stores a cell position, the cost to reach that cell from the start
 * (g), and a pointer to its parent node to allow path reconstruction once the
 * goal is reached.
 *
 * @param cell A 2D integer vector representing the cell's coordinates.
 * @param g The accumulated cost from the starting cell to the current cell.
 * @param parent Pointer to the parent node in the path.
 */
struct PqNode {
 public:
  PqNode(Eigen::Vector2i cell, double g, std::shared_ptr<PqNode> parent)
      : cell{cell}, g{g}, parent{parent} {}

  Eigen::Vector2i cell;
  double g;  // cost to reach the node

  std::shared_ptr<PqNode> parent;
};

/**
 * @brief Comparator functor for nodes in the priority queue.
 *
 * This functor compares two nodes based on the sum of the cost to reach the
 * node (g) and an heuristic estimate (Euclidean distance) from the node to the
 * goal cell. The node with the lower sum is given higher priority, facilitating
 * efficient A* search.
 *
 * @param goal_cell The target cell used to compute the heuristic.
 */
struct PqNodeCompare {
  PqNodeCompare(Eigen::Vector2i goal_cell) : goal_cell{goal_cell} {}

  bool operator()(const std::shared_ptr<PqNode> &lhs,
                  const std::shared_ptr<PqNode> &rhs) const {
    auto lhs_cost = lhs->g + (lhs->cell - goal_cell).norm();
    auto rhs_cost = rhs->g + (rhs->cell - goal_cell).norm();
    return lhs_cost > rhs_cost;
  }

 private:
  Eigen::Vector2i goal_cell;
};

};  // namespace PathPlannerUtil

/**
 * @class AStarPathPlanner
 * @brief Implements the A* algorithm for path planning in occupancy grid maps.
 *
 * Usage:
 *   Created an instance of AStarPathPlanner and invoke the operator() to
 * compute the path, supplying the goal pose, map, and current pose.
 *
 */
class AStarPathPlanner : public AbstractPathPlanner {
 public:
  AStarPathPlanner() = default;
  virtual ~AStarPathPlanner() = default;

  nav_msgs::msg::Path operator()(
      const geometry_msgs::msg::PoseStamped::SharedPtr goal,
      const nav_msgs::msg::OccupancyGrid::SharedPtr map,
      const geometry_msgs::msg::PoseStamped::SharedPtr pose) override;

 private:
  std::vector<Eigen::Vector2i> reconstruct_cell_path(
      std::shared_ptr<PathPlannerUtil::PqNode> end_node);

  std::vector<Eigen::Vector2i> exec_astar(Eigen::Vector2i start_cell,
                                          Eigen::Vector2i goal_cell);

  nav_msgs::msg::Path cell_to_world_path(
      const std::vector<Eigen::Vector2i> &cell_path);

  std::vector<Eigen::Vector2i> get_valid_neighbors(const Eigen::Vector2i &cell);

 private:
  std::shared_ptr<OgmView> ogm_;
};