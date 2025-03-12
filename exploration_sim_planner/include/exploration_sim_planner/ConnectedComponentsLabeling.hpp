/*
 * ConnectedComponentsLabeling.hpp
 *
 * This class implements the connected components labeling algorithm
 * presented in the FALCON paper.

  * First, cells in the map are labeled as either safe-free, unsafe-free,
  * occupied, or unknown.
  * Unsafe-free cells are known free cells that violate a minimum distance
  * requirement.

  * Neighboring cells are connected IFF they are both safe-free or both unknown.
  * Then, the initial centers of these connected cells are found. because
  * convexity is not guaranteed, a center that falls outside of the zone is
  * recomputed as the nearest point on the boundary of the zone from the
  * calculated center.
 */

#pragma once

#include <Eigen/Dense>
#include <set>

#include "exploration_sim_planner/util/OgmView.hpp"

enum class CellLabel {
  UNKNOWN = 0,
  OCCUPIED = 1,
  SAFE_FREE = 2,
  UNSAFE_FREE = 3
};

struct ConnectivityGraph {
  std::vector<Eigen::Vector2d> nodes;             // centers of zones
  std::set<std::pair<uint32_t, uint32_t>> edges;  // edges between zones
};

class ConnectedComponentsLabeling {
 public:
  ConnectedComponentsLabeling(uint32_t sector_size, uint32_t safe_distance);

  // Label the cells in the map as safe-free, unsafe-free, occupied, or unknown
  Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic> label_cells(
      const OgmView& ogm);

  // Decompose the map into zones based on sectors and cell labels
  Eigen::Matrix2i compute_zones(
      const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>&
          cell_labels);

  // Find the centers of the connected components
  std::vector<Eigen::Vector2d> find_centers(
      const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>&
          cell_labels,
      const Eigen::Matrix2i& zones);

  ConnectivityGraph compute_incremental_connectivity_graph(
      const Eigen::Matrix2i& cell_labels,
      const std::vector<Eigen::Vector2d>& centers);

 private:
  // Get the label of a cell
  CellLabel get_cell_label(const OgmView& ogm, uint32_t x, uint32_t y);

  // Check if a cell is safe-free
  bool is_safe_free(const OgmView& ogm, uint32_t x, uint32_t y);

  // just does the zone assignment for a sector
  void compute_sector_zones(const Eigen::Matrix2i& cell_labels,
                            Eigen::Matrix2i& zones, uint32_t x, uint32_t y,
                            uint32_t& zone_id);

 private:
  // The size of the sectors in the map
  uint32_t sector_size_;
  uint32_t safe_distance_;
};