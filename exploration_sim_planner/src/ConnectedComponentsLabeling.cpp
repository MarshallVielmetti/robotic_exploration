/*
 * ConnectedComponentsLabeling.cpp
 * This class implements the ConnectedComponentsLabeling class defined in
 * ConnectedComponentsLabeling.hpp.
 */

#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"

#include <unordered_map>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "exploration_sim_planner/util/UnionFind.hpp"

#define OGM_OCCUPIED 1
#define OGM_FREE 0
#define OGM_UNKNOWN -1

ConnectedComponentsLabeling::ConnectedComponentsLabeling(uint32_t sector_size,
                                                         uint32_t safe_distance)
    : sector_size_(sector_size), safe_distance_{safe_distance} {}

Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>
ConnectedComponentsLabeling::label_cells(const OgmView& ogm) {
  Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic> cell_labels(
      ogm.height(), ogm.width());

  for (uint32_t y = 0; y < ogm.height(); y++) {
    for (uint32_t x = 0; x < ogm.width(); x++) {
      cell_labels(y, x) = get_cell_label(ogm, x, y);
    }
  }

  return cell_labels;
}

CellLabel ConnectedComponentsLabeling::get_cell_label(const OgmView& ogm,
                                                      uint32_t x, uint32_t y) {
  if (ogm.get(x, y) == OGM_UNKNOWN) {
    return CellLabel::UNKNOWN;
  } else if (ogm.get(x, y) == OGM_OCCUPIED) {
    return CellLabel::OCCUPIED;
  } else if (is_safe_free(ogm, x, y)) {
    return CellLabel::SAFE_FREE;
  } else {
    return CellLabel::UNSAFE_FREE;
  }
}

bool ConnectedComponentsLabeling::is_safe_free(const OgmView& ogm, uint32_t x,
                                               uint32_t y) {
  // assumed the cell itself is free at this point
  // just need to check if it falls within the safe distance of an occupied cell

  // check if the cell is within the safe distance of an occupied cell
  for (int32_t dy = -safe_distance_; dy <= static_cast<int32_t>(safe_distance_);
       dy++) {
    for (int32_t dx = -safe_distance_;
         dx <= static_cast<int32_t>(safe_distance_); dx++) {
      if (dx == 0 && dy == 0) {
        continue;
      }

      if (x + dx < 0 || x + dx >= ogm.width() || y + dy < 0 ||
          y + dy >= ogm.height()) {
        continue;
      }

      if (ogm.get(x + dx, y + dy) == OGM_OCCUPIED) {
        return false;
      }
    }
  }
}

Eigen::Matrix2i ConnectedComponentsLabeling::compute_zones(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>&
        cell_labels) {
  Eigen::Matrix2i zones(cell_labels.rows(), cell_labels.cols());
  zones.setZero();

  // id of the next zone to be assigned
  uint32_t cur_zone_id = 0;

  // loop through every sector in the map
  for (uint32_t y = 0; y < cell_labels.rows(); y += sector_size_) {
    for (uint32_t x = 0; x < cell_labels.cols(); x += sector_size_) {
      compute_sector_zones(cell_labels, zones, x, y, cur_zone_id);
    }
  }

  return zones;
}

void ConnectedComponentsLabeling::compute_sector_zones(
    const Eigen::Matrix2i& cell_labels, Eigen::Matrix2i& zones, uint32_t x0,
    uint32_t y0, uint32_t& zone_id) {
  // loop through every cell in the sector
  // for each cell, check the type of the cell to the left and below
  // if they are the same, assign the same zone id
  // if they are different, assign a new zone id

  // Uses UnionFind.hpp
  UnionFind uf(sector_size_ * sector_size_);

  // if they are the same but have different zone ids, merge the zones
  for (uint32_t dy = 0; dy < sector_size_ && y0 + dy < cell_labels.rows();
       dy++) {
    for (uint32_t dx = 0; dx < sector_size_ && x0 + dx < cell_labels.cols();
         dx++) {
      uint32_t x = x0 + dx;
      uint32_t y = y0 + dy;

      // first cell in the sector
      if (dx == 0 && dy == 0) {
        continue;
      }

      // check both neighbors regardless -- if they are the same, the union
      // operation will either combine two existing, uncombined sets or do
      // nothing

      // check the cell to the left (requires dx > 0)
      if (dx > 0 && cell_labels(y, x) == cell_labels(y, x - 1)) {
        uf.unite(dy * sector_size_ + dx, dy * sector_size_ + dx - 1);
      }

      // check the cell below (requires dy > 0)
      if (dy > 0 && cell_labels(y, x) == cell_labels(y - 1, x)) {
        uf.unite(dy * sector_size_ + dx, (dy - 1) * sector_size_ + dx);
      }
    }
  }

  // assign zone ids between zone_id and zone_id + num_sets
  std::unordered_map<uint32_t, uint32_t> zone_remapping;

  for (uint32_t dy = 0; dy < sector_size_ && y0 + dy < cell_labels.rows();
       dy++) {
    for (uint32_t dx = 0; dx < sector_size_ && x0 + dx < cell_labels.cols();
         dx++) {
      uint32_t x = x0 + dx;
      uint32_t y = y0 + dy;

      uint32_t set_id = uf.find(dy * sector_size_ + dx);

      if (zone_remapping.find(set_id) == zone_remapping.end()) {
        zone_remapping[set_id] = zone_id++;
      }

      zones(y, x) = zone_remapping[set_id];
    }
  }
}

std::vector<Eigen::Vector2d> ConnectedComponentsLabeling::find_centers(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
    const Eigen::Matrix2i& zones) {
  // map from zone id to the working value for center
  std::unordered_map<uint32_t, Eigen::Vector2d> zone_centers;

  // map from zone id to the number of points that have been aggregated
  // this is used to calculate the average center position
  std::unordered_map<uint32_t, uint32_t> zone_counts;

  for (uint32_t y = 0; y < cell_labels.rows(); y++) {
    for (uint32_t x = 0; x < cell_labels.cols(); x++) {
      uint32_t zone_id = zones(y, x);
      CellLabel cell_label = cell_labels(y, x);

      // only care about zones that are not occupied & safe
      if (cell_label == CellLabel::OCCUPIED ||
          cell_label == CellLabel::UNSAFE_FREE) {
        continue;
      }

      if (zone_centers.find(zone_id) == zone_centers.end()) {
        zone_centers[zone_id] = Eigen::Vector2d::Zero();
        zone_counts[zone_id] = 0;
      }

      zone_centers[zone_id] +=
          Eigen::Vector2d(x, y) + Eigen::Vector2d(0.5, 0.5);
      zone_counts[zone_id]++;
    }
  }

  std::vector<Eigen::Vector2d> centers;
  centers.reserve(zone_centers.size());

  for (const auto& [zone_id, count] : zone_counts) {
    Eigen::Vector2d center = zone_centers[zone_id] / count;
    centers.push_back(center);
  }

  return centers;
}

ConnectivityGraph
ConnectedComponentsLabeling::compute_incremental_connectivity_graph(
    const Eigen::Matrix2i& cell_labels,
    const std::vector<Eigen::Vector2d>& centers) {
  ConnectivityGraph graph;

  return graph;
}
