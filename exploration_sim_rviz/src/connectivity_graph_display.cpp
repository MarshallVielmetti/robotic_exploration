/**
 * @file connectivity_graph_display.cpp
 * @brief Implementation of the ConnectivityGraphDisplay class
 */

#include "exploration_sim_rviz/connectivity_graph_display.hpp"
#include <OgrePrerequisites.h>
#include <memory>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

namespace exploration_sim_rviz {

ConnectivityGraphDisplay::ConnectivityGraphDisplay() {
  node_size_property_ = new rviz_common::properties::FloatProperty(
      "Node Size", 0.2f, "Size of each node in meters", this);
  node_size_property_->setMin(0.01);

  node_color_property_ = new rviz_common::properties::ColorProperty(
      "Node Color", QColor(0, 255, 0), "Color for nodes", this);

  line_width_property_ = new rviz_common::properties::FloatProperty(
      "Line Width", 0.02f, "Width of connection lines in meters", this);
  line_width_property_->setMin(0.001);

  free_color_property_ = new rviz_common::properties::ColorProperty(
      "Line Color", QColor(255, 0, 0), "Color for connection lines", this);

  unknown_color_property_ = new rviz_common::properties::ColorProperty(
      "Line Color", QColor(0, 255, 0), "Color for connection lines", this);

  portal_color_property_ = new rviz_common::properties::ColorProperty(
      "Line Color", QColor(0, 0, 255), "Color for connection lines", this);

  toggle_path_coloring_ = new rviz_common::properties::BoolProperty(
      "Toggle Path WEight Coloring", false, "Toggle path coloring", this);
}

ConnectivityGraphDisplay::~ConnectivityGraphDisplay() { clearMarkers(); }

void ConnectivityGraphDisplay::onInitialize() { MFDClass::onInitialize(); }

void ConnectivityGraphDisplay::clearMarkers() {
  node_markers_.clear();
  edge_lines_.clear();
}

void ConnectivityGraphDisplay::reset() {
  MFDClass::reset();
  clearMarkers();
}

void ConnectivityGraphDisplay::processMessage(
    const exploration_sim_msgs::msg::ConnectivityGraph::ConstSharedPtr msg) {

  // Clear previous visualization
  clearMarkers();

  if (!msg) {
    return;
  }

  // RVIZ_COMMON_LOG_INFO_STREAM("Received connectivity graph with "
  //                             << msg->nodes.size() << " nodes and "
  //                             << msg->connectivity_matrix.size()
  //                             << " connections");

  // Get property values
  float node_size = node_size_property_->getFloat();
  QColor node_qcolor = node_color_property_->getColor();
  Ogre::ColourValue node_color(node_qcolor.redF(), node_qcolor.greenF(),
                               node_qcolor.blueF());

  float line_width = line_width_property_->getFloat();

  QColor free_line_qcolor = free_color_property_->getColor();
  Ogre::ColourValue free_line_color(free_line_qcolor.redF(),
                                    free_line_qcolor.greenF(),
                                    free_line_qcolor.blueF(), 1.0);

  QColor unknown_line_qcolor = unknown_color_property_->getColor();
  Ogre::ColourValue unknown_line_color(unknown_line_qcolor.redF(),
                                       unknown_line_qcolor.greenF(),
                                       unknown_line_qcolor.blueF(), 1.0);

  QColor portal_line_qcolor = portal_color_property_->getColor();
  Ogre::ColourValue portal_line_color(portal_line_qcolor.redF(),
                                      portal_line_qcolor.greenF(),
                                      portal_line_qcolor.blueF(), 1.0);

  bool is_weight_colored = toggle_path_coloring_->getBool();

  const float z_offset = 0.025f; // Place slightly above the ground

  // Create markers for nodes
  for (size_t i = 0; i < msg->nodes.size(); ++i) {
    const auto &node = msg->nodes[i];

    // Create sphere for the node
    auto shape = std::make_unique<rviz_rendering::Shape>(
        rviz_rendering::Shape::Sphere, context_->getSceneManager(),
        scene_node_);

    // Set position and size
    shape->setPosition(Ogre::Vector3(node.x, node.y, z_offset));
    shape->setScale(Ogre::Vector3(node_size, node_size, node_size));
    shape->setColor(node_color);

    node_markers_.push_back(std::move(shape));
  }

  size_t len = msg->edges.size();
  size_t rows = msg->rows;
  size_t cols = len / rows;

  float max_weight = 0.0;

  if (is_weight_colored) {
    // find the max weight of all edges
    for (size_t row = 0; row < rows; row++) {
      for (size_t col = row + 1; col < cols; col++) {
        const auto &connection = msg->edges[row * cols + col];
        if (connection.cost > max_weight) {
          max_weight = connection.cost;
        }
      }
    }
  }

  for (size_t row = 0; row < rows; row++) {
    for (size_t col = row + 1; col < cols; col++) {
      const auto &connection = msg->edges[row * cols + col];

      if (connection.label == 0)
        continue;

      const auto &from_node = msg->nodes[row];
      const auto &to_node = msg->nodes[col];

      auto line = std::make_unique<rviz_rendering::BillboardLine>(
          context_->getSceneManager(), scene_node_);

      line->setLineWidth(line_width);

      line->addPoint(Ogre::Vector3(from_node.x, from_node.y, z_offset));
      line->addPoint(Ogre::Vector3(to_node.x, to_node.y, z_offset));

      if (is_weight_colored) {
        line->setColor(connection.cost / max_weight, 0.0,
                       1.0 - connection.cost / max_weight, 1.0);
      } else {
        if (connection.label == 1) {
          line->setColor(portal_line_qcolor.redF(), portal_line_qcolor.greenF(),
                         portal_line_qcolor.blueF(), 1.0);
        } else if (connection.label == 2) {
          line->setColor(unknown_line_qcolor.redF(),
                         unknown_line_qcolor.greenF(),
                         unknown_line_qcolor.blueF(), 1.0);
        } else if (connection.label == 3) {
          line->setColor(portal_line_qcolor.redF(), portal_line_qcolor.greenF(),
                         portal_line_qcolor.blueF(), 1.0);
        }
      }

      edge_lines_.push_back(std::move(line));
    }
  }
}

} // namespace exploration_sim_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(exploration_sim_rviz::ConnectivityGraphDisplay,
                       rviz_common::Display)