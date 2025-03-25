#include "exploration_sim_planner/path_planner/AStarPathPlanner.hpp"

#include <memory>
#include <queue>
#include <rclcpp/clock.hpp>
#include <unordered_map>

#include "Eigen/src/Core/Matrix.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp

nav_msgs::msg::Path AStarPathPlanner::operator()(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal,
    const nav_msgs::msg::OccupancyGrid::SharedPtr map,
    const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
  // Create an ogm view object to interact with the occupancy grid map
  ogm_ = std::make_shared<OgmView>(map);

  auto start_cell = ogm_->world_to_cell(
      Eigen::Vector2d(pose->pose.position.x, pose->pose.position.y));

  auto goal_cell = ogm_->world_to_cell(
      Eigen::Vector2d(goal->pose.position.x, goal->pose.position.y));

  // Create a path message to store the planned path
  auto cell_path = exec_astar(start_cell, goal_cell);

  // convert cell path to world path
  auto path = cell_to_world_path(cell_path);

  return path;
}

std::vector<Eigen::Vector2i> AStarPathPlanner::exec_astar(
    Eigen::Vector2i start_cell, Eigen::Vector2i goal_cell) {
  auto comparator = PathPlannerUtil::PqNodeCompare(goal_cell);

  std::priority_queue<std::shared_ptr<PathPlannerUtil::PqNode>,
                      std::vector<std::shared_ptr<PathPlannerUtil::PqNode>>,
                      PathPlannerUtil::PqNodeCompare>
      open_set(comparator);

  std::unordered_map<Eigen::Vector2i, std::shared_ptr<PathPlannerUtil::PqNode>>
      closed_set;

  // Add the start node to the open set
  open_set.push(
      std::make_shared<PathPlannerUtil::PqNode>(start_cell, 0, nullptr));

  // A* path planning algorithm implementation
  while (!open_set.empty()) {
    auto curr = open_set.top();
    open_set.pop();

    if (curr->cell == goal_cell) {
      // goal reached
      return reconstruct_cell_path(curr);
    }

    // if this cell has already been visited with a lower cost, then just
    // continue
    if (closed_set.contains(curr->cell) &&
        closed_set[curr->cell]->g <= curr->g) {
      continue;
    }

    closed_set[curr->cell] = curr;

    auto valid_neighbors = get_valid_neighbors(curr->cell);

    for (auto &neighbor : valid_neighbors) {
      Eigen::Vector2d delta = (neighbor - curr->cell).cast<double>();

      // account for diagonal moves
      double g = curr->g + (delta.norm() > 1 ? 1.5 : 1);

      // check if the neighbor cell is already in the closed set
      if (closed_set.contains(neighbor) && closed_set[neighbor]->g <= g) {
        continue;
      }

      // add to the open set
      open_set.push(
          std::make_shared<PathPlannerUtil::PqNode>(neighbor, g, curr));
    }
  }

  // if got here, then failed to find a path
  return {};
}

std::vector<Eigen::Vector2i> AStarPathPlanner::reconstruct_cell_path(
    std::shared_ptr<PathPlannerUtil::PqNode> end_node) {
  std::vector<Eigen::Vector2i> cell_path;

  auto curr = end_node;
  while (curr) {
    cell_path.push_back(curr->cell);
    curr = curr->parent;
  }

  std::reverse(cell_path.begin(), cell_path.end());

  return cell_path;
}

std::vector<Eigen::Vector2i> AStarPathPlanner::get_valid_neighbors(
    const Eigen::Vector2i &cell) {
  std::vector<Eigen::Vector2i> neighbors;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }

      // check if the neighbor cell is occupied
      try {
        if (ogm_->get(cell.x() + dx, cell.y() + dy) != 0) {
          continue;
        }
      } catch (const std::out_of_range &e) {
        continue;
      }

      neighbors.push_back(cell + Eigen::Vector2i(dx, dy));
    }
  }

  return neighbors;
}

nav_msgs::msg::Path AStarPathPlanner::cell_to_world_path(
    const std::vector<Eigen::Vector2i> &cell_path) {
  nav_msgs::msg::Path path;

  auto now = rclcpp::Clock().now();
  std::string frame_id = "diff_drive/odom";

  for (const auto &cell : cell_path) {
    geometry_msgs::msg::PoseStamped pose;
    auto world_point = ogm_->cell_to_world(cell);
    pose.pose.position.x = world_point.x();
    pose.pose.position.y = world_point.y();
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.frame_id = frame_id;
    pose.header.stamp = now;

    path.poses.push_back(pose);
  }

  path.header.frame_id = frame_id;
  path.header.stamp = now;

  return path;
}
