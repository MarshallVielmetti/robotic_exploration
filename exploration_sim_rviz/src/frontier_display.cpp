/**
 * @file frontier_display.cpp
 * @brief Implementation of the FrontierDisplay class
 */

#include <OgreColourValue.h>
#include <exploration_sim_rviz/frontier_display.hpp>
#include <rviz_common/logging.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace exploration_sim_rviz {

FrontierDisplay::~FrontierDisplay() { clearMarkers(); }

void FrontierDisplay::onInitialize() { MFDClass::onInitialize(); }

void FrontierDisplay::clearMarkers() {
  for (auto &cluster : cluster_markers_) {
    for (auto &marker : cluster) {
      marker.reset();
    }
  }

  for (auto &viewpoints : viewpoint_markers_) {
    for (auto &marker : viewpoints) {
      marker.reset();
    }
  }

  cluster_markers_.clear();
  viewpoint_markers_.clear();
}

void FrontierDisplay::reset() {
  MFDClass::reset();
  clearMarkers();
}

Ogre::ColourValue FrontierDisplay::getClusterColor(int cluster_index) {
  float h = 0.0f; // red in HSL
  float s = 1.0f; // Full saturation
  float l = 0.5f; // medium lightness

  // rotate hue using cluster idx and golden ratio
  h += static_cast<float>(cluster_index) * 0.618033988749895f;
  h = std::fmod(h, 1.0f);

  // convert HSL to RGB
  float c = (1.0f - std::abs(2.0f * l - 1.0f)) * s;
  float x = c * (1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f));
  float m = l - c / 2.0f;

  float r, g, b;
  if (h < 1.0f / 6.0f) {
    r = c;
    g = x;
    b = 0;
  } else if (h < 2.0f / 6.0f) {
    r = x;
    g = c;
    b = 0;
  } else if (h < 3.0f / 6.0f) {
    r = 0;
    g = c;
    b = x;
  } else if (h < 4.0f / 6.0f) {
    r = 0;
    g = x;
    b = c;
  } else if (h < 5.0f / 6.0f) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }

  return Ogre::ColourValue(r + m, g + m, b + m, 1.0f);
}

void FrontierDisplay::processMessage(
    const exploration_sim_msgs::msg::FrontierClusters::ConstSharedPtr msg) {
  // Display the number of frontiers
  RVIZ_COMMON_LOG_INFO_STREAM("Received " << msg->clusters.size()
                                          << " frontiers");

  // Clear all previous markers
  clearMarkers();

  if (!msg)
    return;

  float cell_size = msg->info.resolution;
  auto origin = msg->info.origin.position;

  // small z offset to overlay it above the OGM
  const float z_offset = 0.01f;

  // process each cluster
  cluster_markers_.resize(msg->clusters.size());
  viewpoint_markers_.resize(msg->clusters.size());
  for (size_t i = 0; i < msg->clusters.size(); ++i) {
    const auto &cluster = msg->clusters[i];

    Ogre::ColourValue color = getClusterColor(i);

    // create a marker for each frontier in the cluster
    cluster_markers_[i].resize(cluster.points.size());
    for (size_t j = 0; j < cluster.points.size(); ++j) {
      const auto &frontier = cluster.points[j];

      // create a square marker
      cluster_markers_[i][j] = std::make_unique<rviz_rendering::Shape>(
          rviz_rendering::Shape::Cube, context_->getSceneManager(),
          scene_node_);

      float x = frontier.x * cell_size + origin.x + cell_size / 2.0f;
      float y = frontier.y * cell_size + origin.y + cell_size / 2.0f;

      // set the position and scale of the marker
      cluster_markers_[i][j]->setPosition(Ogre::Vector3(x, y, z_offset));
      cluster_markers_[i][j]->setScale(
          Ogre::Vector3(cell_size, cell_size, 0.001f));

      // set the color of the marker
      cluster_markers_[i][j]->setColor(color);
    }

    // display the viewpoints
    viewpoint_markers_[i].resize(cluster.viewpoints.size());
    for (size_t j = 0; j < cluster.viewpoints.size(); j++) {
      const auto &vp = cluster.viewpoints[j];

      // create a small sphere marker
      viewpoint_markers_[i][j] = std::make_unique<rviz_rendering::Shape>(
          rviz_rendering::Shape::Sphere, context_->getSceneManager(),
          scene_node_);

      float x = vp.x * cell_size + origin.x;
      float y = vp.y * cell_size + origin.y;

      // set the position and scale of the marker
      viewpoint_markers_[i][j]->setPosition(Ogre::Vector3(x, y, z_offset));
      viewpoint_markers_[i][j]->setScale(Ogre::Vector3(cell_size / 4.0f));

      // set the color of the marker
      viewpoint_markers_[i][j]->setColor(color);
    }
  }
}

} // namespace exploration_sim_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(exploration_sim_rviz::FrontierDisplay,
                       rviz_common::Display)
