/**
 * @file frontier_display.hpp
 */

#ifndef EXPLORATION_SIM_RVIZ__FRONTIER_DISPLAY_HPP_
#define EXPLORATION_SIM_RVIZ__FRONTIER_DISPLAY_HPP_

#include <OgreColourValue.h>
#include <exploration_sim_msgs/msg/frontier_clusters.hpp>
#include <memory>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace exploration_sim_rviz {

class FrontierDisplay : public rviz_common::MessageFilterDisplay<
                            exploration_sim_msgs::msg::FrontierClusters> {
  Q_OBJECT

public:
  ~FrontierDisplay() override;

protected:
  void processMessage(
      const exploration_sim_msgs::msg::FrontierClusters::ConstSharedPtr msg)
      override;
  void onInitialize() override;
  void reset() override;

private:
  using ShapePtr = std::unique_ptr<rviz_rendering::Shape>;
  std::vector<std::vector<ShapePtr>> cluster_markers_;

  void clearMarkers();
  Ogre::ColourValue getClusterColor(int cluster_id);
};
} // namespace exploration_sim_rviz

#endif // EXPLORATION_SIM_RVIZ__FRONTIER_DISPLAY_HPP_
