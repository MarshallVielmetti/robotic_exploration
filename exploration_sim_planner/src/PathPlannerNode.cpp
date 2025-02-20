#include "exploration_sim_planner/PathPlannerNode.hpp"

#include "exploration_sim_planner/path_planner/AStarPathPlanner.hpp"

PathPlannerNode::PathPlannerNode() : Node("path_planner") {
  // Create a publisher for the path
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

  // Create a subscriber for the map
  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&PathPlannerNode::map_callback, this, std::placeholders::_1));

  // Subscribe to the goal pose
  // TODO: This should be an array of goal poses, and it should generate the
  // optimal aTSP solution?
  goal_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "goal", 10,
          std::bind(&PathPlannerNode::goal_callback, this,
                    std::placeholders::_1));

  // Subscribe to the robot pose
  pose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "robot_pose", 10,
          std::bind(&PathPlannerNode::pose_callback, this,
                    std::placeholders::_1));

  path_planner = std::make_shared<AStarPathPlanner>();
}

void PathPlannerNode::trigger_replanning() {
  // If any of the current state is not set, do not replan
  if (!current_goal_ || !current_map_ || !current_pose_) {
    return;
  }

  auto path = (*path_planner)(current_goal_, current_map_, current_pose_);
  path_publisher_->publish(path);
}

void PathPlannerNode::goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  this->current_goal_ = msg;
  trigger_replanning();
}

void PathPlannerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  this->current_map_ = msg;
}

void PathPlannerNode::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  this->current_pose_ = msg;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}