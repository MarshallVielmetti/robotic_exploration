#pragma once

#include "Eigen/Dense"
#include "nav_msgs/msg/occupancy_grid.hpp"

/**
 * @class OgmView
 * @brief A class to view and interact with an occupancy grid map (OGM).
 *
 * This class provides methods to access and manipulate the occupancy grid map.
 * It allows querying the occupancy status of specific cells and converting
 * world coordinates to grid cell coordinates.
 */
class OgmView {
 public:
  /**
   * @brief Constructor for OgmView.
   *
   * @param ogm A shared pointer to the occupancy grid map.
   */
  OgmView(nav_msgs::msg::OccupancyGrid::SharedPtr ogm)
      : ogm_(ogm),
        resolution_(ogm->info.resolution),
        width_(ogm->info.width),
        height_(ogm->info.height) {
    origin_ << ogm->info.origin.position.x, ogm->info.origin.position.y;
  }

  /**
   * @brief Get the occupancy value at a specific grid cell.
   *
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   * @return The occupancy value at the specified cell.
   * @throws std::out_of_range if the cell coordinates are out of range.
   */
  int8_t get(const uint32_t x, const uint32_t y) const {
    if (x >= width_ || y >= height_) {
      throw std::out_of_range("Index out of range");
    }

    // The occupancy grid is stored in row-major order
    return ogm_->data[y * width_ + x];
  }

  /**
   * @brief Get the occupancy value at a specific point in the world.
   *
   * @param point The world coordinates of the point.
   * @return The occupancy value at the specified point.
   */
  int8_t get_point(const Eigen::Vector2d &point) {
    const Eigen::Vector2i cell = world_to_cell(point);
    return get(cell.x(), cell.y());
  }

  /**
   * @brief Get the world coordinates of a specific grid cell.
   *
   * @param cell The grid cell coordinates.
   * @return The world coordinates of the specified cell.
   */
  Eigen::Vector2d cell_to_world(const Eigen::Vector2i &cell) {
    return origin_ + cell.cast<double>() * resolution_;
  }

  /**
   * @brief Convert world coordinates to grid cell coordinates.
   *
   * @param point The world coordinates to be converted.
   * @return The grid cell coordinates corresponding to the world coordinates.
   */
  Eigen::Vector2i world_to_cell(const Eigen::Vector2d &point) {
    if (point.x() < origin_.x() || point.y() < origin_.y()) {
      throw std::out_of_range("Point outside the occupancy grid");
    }

    // floors the value to the nearest integer
    const Eigen::Vector2i cell = ((point - origin_) / resolution_).cast<int>();

    return cell;
  }

  int8_t operator()(const uint32_t x, const uint32_t y) const {
    return get(x, y);
  }

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }

 private:
  nav_msgs::msg::OccupancyGrid::SharedPtr ogm_;

  double resolution_;
  uint32_t width_;
  uint32_t height_;

  Eigen::Vector2d
      origin_;  // point represented by the (0,0) cell in the occupancy grid
};