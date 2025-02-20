#include "exploration_sim_planner/path_planner/AStarPathPlanner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <queue>

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

std::vector<Eigen::Vector2i>
AStarPathPlanner::exec_astar(Eigen::Vector2i start_cell,
                             Eigen::Vector2i goal_cell) {

  std::vector<std::vector<Node>> nodes(ogm_->width(),
                                       std::vector<Node>(ogm_->height()));

  auto node_provider = NodeProvider(ogm_.get(), goal_cell);
  // A* path planning algorithm implementation

  std::priority_queue<Node *, std::vector<Node *>, NodePtrCompare> open_set;

  // Add the start node to the open set
  open_set.push(node_provider.get_node(start_cell, 0));

  //
  while (!open_set.empty()) {
  }
}
