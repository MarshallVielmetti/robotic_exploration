#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>


class AbstractPathPlanner {
public:
  AbstractPathPlanner() = default;
  virtual ~AbstractPathPlanner() = default;


  virtual nav_msgs::msg::Path operator()(const geometry_msgs::msg::PoseStamped::SharedPtr goal,
                        const nav_msgs::msg::OccupancyGrid::SharedPtr map,
                        const geometry_msgs::msg::PoseStamped::SharedPtr pose) = 0;

};