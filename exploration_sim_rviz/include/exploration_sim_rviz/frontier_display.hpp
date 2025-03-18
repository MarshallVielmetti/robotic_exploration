/**
 * @file frontier_display.hpp
 */

#ifndef EXPLORATION_SIM_RVIZ__FRONTIER_DISPLAY_HPP_ 
#define EXPLORATION_SIM_RVIZ__FRONTIER_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <exploration_sim_msgs/msg/frontier_clusters.hpp>

namespace exploration_sim_rviz
{

class FrontierDisplay
  : public rviz_common::MessageFilterDisplay<exploration_sim_msgs::msg::FrontierClusters>
{
  Q_OBJECT

protected:
  void processMessage(const exploration_sim_msgs::msg::FrontierClusters::ConstSharedPtr msg) override;
};
}  // namespace exploration_sim_rviz

#endif // EXPLORATION_SIM_RVIZ__FRONTIER_DISPLAY_HPP_
