/**
 * @file frontier_display.cpp
 * @brief Implementation of the FrontierDisplay class
 */

#include <exploration_sim_rviz/frontier_display.hpp>
#include <rviz_common/logging.hpp>

namespace exploration_sim_rviz
{
  void FrontierDisplay::processMessage(const exploration_sim_msgs::msg::FrontierClusters::ConstSharedPtr msg)
  {
    // Display the number of frontiers
    RVIZ_COMMON_LOG_INFO_STREAM("Received " << msg->clusters.size() << " frontiers");
  }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(exploration_sim_rviz::FrontierDisplay, rviz_common::Display)
