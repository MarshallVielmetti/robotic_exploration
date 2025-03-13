/*
 * ConnectedComponentsLabeling.cpp
 * This class implements the ConnectedComponentsLabeling class defined in
 * ConnectedComponentsLabeling.hpp.
 */

#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"

#include <cstdint>
#include <iterator>
#include <queue>
#include <unordered_map>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "exploration_sim_planner/path_planner/AStarPathPlanner.hpp"
#include "exploration_sim_planner/util/UnionFind.hpp"

#define OGM_OCCUPIED 1
#define OGM_FREE 0
#define OGM_UNKNOWN -1

/**
 * Definition of terms used
 * - Cell: A single grid cell in the occupancy grid map
 * - Sector: A square region of the map that is sector_size x sector_size cells
 * - Zone: A connected component of safe-free or unknown cells
 * - Center: The center of a zone, used for path planning
 * - Edge: A connection between two centers, used for path planning, weighted
 * proportionally to the A* distance between the centers
 */

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

  return true;
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

  // TODO - verify the center is located inside the zone, and rectify if not

  return centers;
}

ConnectivityGraph
ConnectedComponentsLabeling::compute_incremental_connectivity_graph(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
    const std::vector<Eigen::Vector2d>& centers) {
  ConnectivityGraph graph;

  graph.nodes = centers;

  // compute the cell that each center (double) is in by flooring the values
  std::vector<Eigen::Vector2i> center_cells;

  std::transform(
      centers.begin(), centers.end(), std::back_inserter(center_cells),
      [](const Eigen::Vector2d& center) {
        return Eigen::Vector2i(std::floor(center.x()), std::floor(center.y()));
      });

  // loop through every pair of centers
  for (uint32_t i = 0; i < centers.size(); i++) {
    for (uint32_t j = i + 1; j < centers.size(); j++) {
      // the two centers can only be connected if they are in the same or
      // adjacent sectors (don't include diagonals)
      int32_t x_diff =
          std::abs(center_cells[i].x() / static_cast<int32_t>(sector_size_) -
                   center_cells[j].x() / static_cast<int32_t>(sector_size_));

      int32_t y_diff =
          std::abs(center_cells[i].y() / static_cast<int32_t>(sector_size_) -
                   center_cells[j].y() / static_cast<int32_t>(sector_size_));

      // if the centers are not in the same or adjacent sectors, skip
      if (x_diff > 1 || y_diff > 1 || x_diff + y_diff > 1) {
        continue;
      }

      // Get cell labels
      CellLabel i_label = cell_labels(center_cells[i].y(), center_cells[i].x());
      CellLabel j_label = cell_labels(center_cells[j].y(), center_cells[j].x());

      /*
       * Three cases: both safe-free (free edges)
       * both unknown (unknown edges)
       * one of each, and in the same sector (portal edges)
       */

      if ((i_label == CellLabel::SAFE_FREE &&
           j_label == CellLabel::SAFE_FREE) ||
          (i_label == CellLabel::UNKNOWN && j_label == CellLabel::UNKNOWN)) {
        // check if the two centers are connected by a path
        auto path_cost =
            restricted_astar(cell_labels, center_cells[i], center_cells[j]);

        // no valid path found
        if (!path_cost.has_value()) {
          continue;
        }

        EdgeType type = (i_label == CellLabel::SAFE_FREE) ? EdgeType::FREE
                                                          : EdgeType::UNKNOWN;

        graph.add_edge(i, j, type, path_cost.value());
      }

      // if they are different, specifically one is free and one is safe (else
      // case) then only check for path if they are in the same sector
      // this edge would be a "portal path"
      else if (x_diff == 0 && y_diff == 0) {
        // check if the two centers are connected by a path
        auto path_cost =
            restricted_astar(cell_labels, center_cells[i], center_cells[j]);

        if (!path_cost.has_value()) {
          continue;
        }

        // add the edge
        graph.add_edge(i, j, EdgeType::PORTAL, path_cost.value());
      }  // end if
    }  // end inner for
  }  // end outer for

  /*
   * Remove any isolated subcomponents before returning
   */

  filter_isolated_subcomponents(graph);

  return graph;
}

std::optional<double> ConnectedComponentsLabeling::restricted_astar(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
    const Eigen::Vector2i& start, const Eigen::Vector2i& goal) {
  // check if the start and goal are the same
  if (start == goal) {
    return 0.0;
  }

  // helper function to check if a cell is valid to be considered in the
  // exploration process -- restricted astar
  std::function<bool(const Eigen::Vector2i&)> is_valid_cell =
      [&](const Eigen::Vector2i& cell) {
        // check that the cell is the same type as the start OR goal
        if (cell_labels(cell.y(), cell.x()) !=
                cell_labels(start.y(), start.x()) &&
            cell_labels(cell.y(), cell.x()) !=
                cell_labels(goal.y(), goal.x())) {
          return false;
        }

        // check that the cell is in the map
        if (cell.x() < 0 || cell.x() >= cell_labels.cols() || cell.y() < 0 ||
            cell.y() >= cell_labels.rows()) {
          return false;
        }

        // check that the cell is in the same sector as the start or goal
        int start_sector_x = start.x() / sector_size_;
        int start_sector_y = start.y() / sector_size_;

        int goal_sector_x = goal.x() / sector_size_;
        int goal_sector_y = goal.y() / sector_size_;

        int cell_sector_x = cell.x() / sector_size_;
        int cell_sector_y = cell.y() / sector_size_;

        // verify the cell is in the same sector as the start or goal cells
        if ((start_sector_x != cell_sector_x ||
             start_sector_y != cell_sector_y) &&
            (goal_sector_x != cell_sector_x ||
             goal_sector_y != cell_sector_y)) {
          return false;
        }

        return true;
      };

  PathPlannerUtil::PqNodeCompare comparator(goal);

  std::priority_queue<std::shared_ptr<PathPlannerUtil::PqNode>,
                      std::vector<std::shared_ptr<PathPlannerUtil::PqNode>>,
                      PathPlannerUtil::PqNodeCompare>
      open_set(comparator);

  std::unordered_map<Eigen::Vector2i, std::shared_ptr<PathPlannerUtil::PqNode>>
      closed_set;

  open_set.push(std::make_shared<PathPlannerUtil::PqNode>(start, 0, nullptr));

  while (!open_set.empty()) {
    auto curr = open_set.top();
    open_set.pop();

    // check if goal reached, if so return cost
    if (curr->cell == goal) {
      return curr->g;
    }

    // if this cell has already been visited with a lower cost, then just
    // continue
    if (closed_set.contains(curr->cell) &&
        closed_set[curr->cell]->g <= curr->g) {
      continue;
    }

    closed_set[curr->cell] = curr;

    auto valid_neighbors = ConnectedComponentsLabeling::get_valid_neighbors(
        curr->cell, is_valid_cell);

    for (auto& neighbor : valid_neighbors) {
      Eigen::Vector2d delta = (neighbor - curr->cell).cast<double>();

      double g = curr->g + 1;

      // check if the neighbor cell is already in the closed set
      if (closed_set.contains(neighbor) && closed_set[neighbor]->g <= g) {
        continue;
      }

      // add to the open set
      open_set.push(
          std::make_shared<PathPlannerUtil::PqNode>(neighbor, g, curr));
    }
  }

  return std::nullopt;
};

std::vector<Eigen::Vector2i> ConnectedComponentsLabeling::get_valid_neighbors(
    const Eigen::Vector2i& cell,
    std::function<bool(const Eigen::Vector2i&)>& is_valid_cell) {
  // vector to store return values
  std::vector<Eigen::Vector2i> neighbors;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      // only consider 4 direct non-diagonal neighboring cells
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) {
        continue;
      }

      // check if neighboring cell is valid
      if (!is_valid_cell(cell + Eigen::Vector2i(dx, dy))) {
        continue;
      }

      neighbors.push_back(cell + Eigen::Vector2i(dx, dy));
    }
  }

  return neighbors;
}

void ConnectedComponentsLabeling::remove_isolated_subcomponents(
    ConnectivityGraph& graph) {}