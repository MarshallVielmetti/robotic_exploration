/**
 * @file connectivity_graph_display.hpp
 */

#ifndef EXPLORATION_SIM_RVIZ__CONNECTIVITY_GRAPH_DISPLAY_HPP_
#define EXPLORATION_SIM_RVIZ__CONNECTIVITY_GRAPH_DISPLAY_HPP_

#include "exploration_sim_msgs/msg/connectivity_graph.hpp"
#include <OgreColourValue.h>
#include <cwchar>
#include <exploration_sim_msgs/msg/connectivity_graph.hpp>
#include <memory>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace exploration_sim_rviz {

class ConnectivityGraphDisplay
    : public rviz_common::MessageFilterDisplay<
          exploration_sim_msgs::msg::ConnectivityGraph> {
  Q_OBJECT

public:
  ConnectivityGraphDisplay();
  ~ConnectivityGraphDisplay() override;

protected:
  void processMessage(
      const exploration_sim_msgs::msg::ConnectivityGraph::ConstSharedPtr)
      override;

  void onInitialize() override;
  void reset() override;

private:
  using ShapePtr = std::unique_ptr<rviz_rendering::Shape>;
  using LinePtr = std::unique_ptr<rviz_rendering::BillboardLine>;

  std::vector<ShapePtr> node_markers_;
  std::vector<LinePtr> edge_lines_;

  void clearMarkers();

  // Properties
  rviz_common::properties::FloatProperty *node_size_property_;
  rviz_common::properties::ColorProperty *node_color_property_;
  rviz_common::properties::FloatProperty *line_width_property_;
  rviz_common::properties::ColorProperty *free_color_property_;
  rviz_common::properties::ColorProperty *unknown_color_property_;
  rviz_common::properties::ColorProperty *portal_color_property_;
};

} // namespace exploration_sim_rviz

#endif