/*
 * ConnectedComponentsLabeling.cpp
 * This class implements the ConnectedComponentsLabeling class defined in
 * ConnectedComponentsLabeling.hpp.
 */

#include "exploration_sim_planner/ConnectedComponentsLabeling.hpp"

#include <algorithm>
#include <cstdint>
#include <execution>
#include <iterator>
#include <queue>
#include <unordered_map>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "exploration_sim_planner/path_planner/AStarPathPlanner.hpp"
#include "exploration_sim_planner/util/UnionFind.hpp"

#define OGM_OCCUPIED 100
#define OGM_FREE 0
#define OGM_UNKNOWN -1

#define MIN_FRONTIER_SIZE 3
#define MIN_FRONTIER_SPLIT_THRESHOLD 10
#define CLUSTER_EIGENVALUE_SPLIT_THRESHOLD 0.5

#define SENSOR_RANGE 10
#define RAYCAST_RESOLUTION 0.1

#define N_VIEW_MAX 8

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

  std::vector<Eigen::Vector2i> frontier_cells;

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

std::vector<Eigen::Vector2i> ConnectedComponentsLabeling::find_frontier_cells(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>&
        cell_labels) {
  std::vector<Eigen::Vector2i> frontier_cells;

  for (uint32_t y = 0; y < cell_labels.rows(); y++) {
    for (uint32_t x = 0; x < cell_labels.cols(); x++) {
      if (is_frontier(cell_labels, x, y)) {
        frontier_cells.push_back(Eigen::Vector2i(x, y));
      }
    }
  }

  return frontier_cells;
}

std::vector<std::vector<Eigen::Vector2i>>
ConnectedComponentsLabeling::cluster_frontiers(
    const std::vector<Eigen::Vector2i>& frontier_cells) {
  // Uses the UnionFind datastructure
  UnionFind uf(frontier_cells.size());

  for (uint32_t i = 0; i < frontier_cells.size(); i++) {
    for (uint32_t j = i + 1; j < frontier_cells.size(); j++) {
      // check if the two cells are adjacent
      auto diff = frontier_cells[i] - frontier_cells[j];

      // allow uniting along diagonals
      if (std::abs(diff.x()) <= 1 && std::abs(diff.y()) <= 1) {
        uf.unite(i, j);
      }
    }
  }

  // map from cluster id to index in list
  std::unordered_map<uint32_t, uint32_t> cluster_idxs;
  std::vector<std::vector<Eigen::Vector2i>> clusters;

  for (uint32_t i = 0; i < frontier_cells.size(); i++) {
    uint32_t set_id = uf.find(i);

    // encountered a new set id
    if (!cluster_idxs.contains(set_id)) {
      cluster_idxs[set_id] = clusters.size();
      clusters.push_back(std::vector<Eigen::Vector2i>());
    }

    clusters[cluster_idxs[set_id]].push_back(frontier_cells[i]);
  }

  /* Filter the map
   *
   * If the cluster is too small, remove it
   *
   * If the cluster is too large, break it in half using PCA and create two new
   * clusters, which are appended to the back of the map, so they can be further
   * broken down if necessary
   */

  for (uint32_t i = 0; i < clusters.size();) {
    // if the cluster is too small, remove it, swap with the back and pop
    if (clusters[i].size() < MIN_FRONTIER_SIZE) {
      std::iter_swap(clusters.begin() + i, std::prev(clusters.end()));
      clusters.pop_back();
      continue;
    }

    // TODO REMOVE
    // i++; continue;

    // if the cluster is too large, split it
    PCAResult pca = cluster_pca(clusters[i]);

    // If the maximum eigenvalue of the PCA is less than the split threshold,
    // or its too small to split, then continue and increment i
    if (pca.eigenvalues.maxCoeff() < CLUSTER_EIGENVALUE_SPLIT_THRESHOLD ||
        clusters[i].size() < MIN_FRONTIER_SPLIT_THRESHOLD) {
      i++;
      continue;
    }

    // If the maximum eigenvalue of the PCA is greater than the split threshold,
    // split the cluster along the first principal component
    auto [cluster1, cluster2] = split_cluster(clusters[i], pca);
    clusters[i] = cluster1;
    clusters.push_back(cluster2);

    // don't increment i here -- need to recursively evaluate cluster1, now
    // located at i
  }

  // because we modified the clusters vector in place, just return it
  return clusters;
}

bool ConnectedComponentsLabeling::is_frontier(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
    uint32_t x, uint32_t y) {
  // frontier cell is an unknown cell that borders a free cell
  if (cell_labels(y, x) != CellLabel::UNKNOWN) {
    return false;
  }

  for (int32_t dy = -1; dy <= 1; dy++) {
    for (int32_t dx = -1; dx <= 1; dx++) {
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) {
        continue;
      }

      if (x + dx < 0 || x + dx >= cell_labels.cols() || y + dy < 0 ||
          y + dy >= cell_labels.rows()) {
        continue;
      }

      if (cell_labels(y + dy, x + dx) == CellLabel::SAFE_FREE ||
          cell_labels(y + dy, x + dx) == CellLabel::UNSAFE_FREE) {
        return true;
      }
    }
  }

  return false;
}

PCAResult ConnectedComponentsLabeling::cluster_pca(
    std::vector<Eigen::Vector2i>& cluster) {
  // convert the cluster to a matrix
  Eigen::MatrixXd data(cluster.size(), 2);  // nx2 matrix of data points

  for (size_t i = 0; i < cluster.size(); i++) {
    data.row(i) = cluster[i].cast<double>();
  }

  // Step 1 : Mean Center the Data
  Eigen::Vector2d mean = data.colwise().mean();  // 2x1 vector
  data.rowwise() -= mean.transpose();

  // Step 2 : Compute the Covariance Matrix Q
  // This can be estimated as X^T X / (n-1) (resulting matrix is 2x2)
  uint32_t n = data.rows();
  Eigen::MatrixXd Q = data.transpose() * data / static_cast<double>(n - 1);

  // Step 3 : Perform Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeFullU);

  // Step 4 : Compute the Eigenvectors and Eigenvalues
  Eigen::MatrixXd eigenvectors = svd.matrixU();
  Eigen::VectorXd eigenvalues = svd.singularValues();

  // Step 5 : Apply the transformation
  // It is given by X_new = X V, where X is the mean centered data
  Eigen::MatrixXd transformed_data = data * eigenvectors;

  return PCAResult(eigenvalues, eigenvectors, transformed_data);
}

std::pair<std::vector<Eigen::Vector2i>, std::vector<Eigen::Vector2i>>
ConnectedComponentsLabeling::split_cluster(
    const std::vector<Eigen::Vector2i>& cluster, const PCAResult& pca) {
  // we have the transformed data, so we can simply check whether the value of
  // the first component is greater than 0 or less than 0 to determine which
  // cluster the point belongs to

  std::vector<Eigen::Vector2i> cluster1, cluster2;

  // need to find the median value along the first principle axis
  Eigen::VectorXd principle_axis = pca.transformed_data.col(0);

  std::sort(principle_axis.data(),
            principle_axis.data() + principle_axis.size());

  size_t n = principle_axis.size();
  double median;
  if (n % 2 == 0) {
    median = (principle_axis(n / 2 - 1) + principle_axis(n / 2)) / 2.f;
  } else {
    median = principle_axis(n / 2);
  }

  for (size_t i = 0; i < cluster.size(); i++) {
    if (pca.transformed_data(i, 0) >= median) {
      cluster1.push_back(cluster[i]);
    } else {
      cluster2.push_back(cluster[i]);
    }
  }

  return std::make_pair(cluster1, cluster2);
}

std::vector<std::vector<Eigen::Vector2d>>
ConnectedComponentsLabeling::sample_frontier_viewpoints(
    const std::vector<std::vector<Eigen::Vector2i>>& frontier_clusters,
    const Eigen::MatrixX<CellLabel>& cell_labels) {
  std::vector<std::vector<Eigen::Vector2d>> viewpoints;

  // applies a parallel transformation from frontier_clusters to viewpoints
  std::transform(std::execution::par, frontier_clusters.begin(),
                 frontier_clusters.end(), viewpoints.begin(),
                 [&cell_labels](const std::vector<Eigen::Vector2i>& cluster) {
                   // sample a viewpoint for each frontier cluster
                   auto vp =
                       sample_one_frontier_viewpoints(cluster, cell_labels);

                   // filter the viewpoints
                   filter_viewpoints(vp, cluster, cell_labels);
                   return vp;
                 });

  return viewpoints;
}

std::vector<Eigen::Vector2d>
ConnectedComponentsLabeling::sample_one_frontier_viewpoints(
    const std::vector<Eigen::Vector2i>& frontier_cluster,
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>&
        cell_labels) {
  // find the center of the cluster
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  for (const auto& point : frontier_cluster) {
    center += point.cast<double>();
  }
  center /= frontier_cluster.size();

  // use cylindrical coordinates to sample points around the center lying in
  // free space

  std::vector<Eigen::Vector2d> viewpoints;

  for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 4) {
    for (double r = 1; r < SENSOR_RANGE; r++) {
      Eigen::Vector2d point =
          center + Eigen::Vector2d{r * cos(theta), r * sin(theta)};

      Eigen::Vector2i point_i = point.cast<int>();

      if (point_i.x() < 0 || point_i.x() >= cell_labels.cols() ||
          point_i.y() < 0 || point_i.y() >= cell_labels.rows()) {
        continue;
      }

      if (cell_labels(point_i.y(), point_i.x()) == CellLabel::SAFE_FREE) {
        viewpoints.push_back(point_i);
      }
    }
  }

  return viewpoints;
}

void ConnectedComponentsLabeling::filter_viewpoints(
    std::vector<Eigen::Vector2d>& viewpoints,
    const std::vector<Eigen::Vector2i>& frontier_cells,
    const Eigen::MatrixX<CellLabel>& cell_labels) {
  std::vector<double> coverage(viewpoints.size(), 0.0);

  // In parallel, calculates the coverage of each viewpoint
  std::transform(
      std::execution::seq, viewpoints.begin(), viewpoints.end(),
      coverage.begin(),
      [&cell_labels, &frontier_cells](const Eigen::Vector2i& viewpoint) {
        return calculate_coverage(viewpoint, frontier_cells, cell_labels);
      });

  // now using the coverage, filter out the viewpoints

  // sort the viewpoints by coverage
  std::vector<size_t> indices(viewpoints.size());
  std::iota(indices.begin(), indices.end(), 0);

  std::sort(indices.begin(), indices.end(), [&coverage](size_t i1, size_t i2) {
    return coverage[i1] > coverage[i2];
  });

  // compute the mean and standard deviation of the coverage, and return
  // at most N_view cells, all of which are at least in the top 75% of coverage

  double mean = std::accumulate(coverage.begin(), coverage.end(), 0.0) /
                static_cast<double>(coverage.size());

  double variance = 0.0;
  for (size_t i = 0; i < coverage.size(); i++) {
    variance += (coverage[i] - mean) * (coverage[i] - mean);
  }

  variance /= coverage.size();

  double std_dev = sqrt(variance);

  std::vector<Eigen::Vector2d> filtered_viewpoints(N_VIEW_MAX);

  // compute the threshold
  double threshold = mean - 0.75 * std_dev;

  // Take first N_VIEW_MAX viewpoints that have coverage greater than threshold
  uint32_t n_selected = 0;
  for (size_t idx : indices) {
    if (n_selected >= N_VIEW_MAX) {
      break;
    }

    if (coverage[idx] >= threshold) {
      filtered_viewpoints[n_selected++] = viewpoints[idx];
    }
  }

  // resize the viewpoints vector if less than N_VIEW_MAX selected
  if (n_selected < N_VIEW_MAX) {
    filtered_viewpoints.resize(n_selected);
  }

  // overwrite the passed viewpoints vector
  viewpoints = std::move(filtered_viewpoints);
}

// checks if line from viewpoint to every frontier cell is free and within
// sensor range
double ConnectedComponentsLabeling::calculate_coverage(
    const Eigen::Vector2d& viewpoint,
    const std::vector<Eigen::Vector2i>& frontier_cells,
    const Eigen::MatrixX<CellLabel>& cell_labels) {
  double coverage = 0.0f;

  std::for_each(
      std::execution::seq, frontier_cells.begin(), frontier_cells.end(),
      [&coverage, &viewpoint, &cell_labels](const Eigen::Vector2i& cell) {
        Eigen::Vector2d diff = cell.cast<double>() - viewpoint;
        double distance = diff.norm();

        if (distance > SENSOR_RANGE) {
          return;
        }

        // check if the line of sight is occluded -- if so, return;
        // TODO -- definitely a better algorithm...could probably also improve
        // the sampling process to sample along a ray until it becomes occluded
        // or smthing
        for (double i = 1; i < distance; i += RAYCAST_RESOLUTION) {
          Eigen::Vector2i point =
              (viewpoint + (diff * i / distance)).cast<int>();

          if (cell_labels(point.y(), point.x()) != CellLabel::SAFE_FREE) {
            return;
          }
        }

        // TODO -- maybe weigh distance to points?
        coverage += 1.0;
      });

  return coverage;
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

Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>
ConnectedComponentsLabeling::compute_zones(
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>&
        cell_labels) {
  // Initialize matrix to store the zone information
  Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> zones(
      cell_labels.rows(), cell_labels.cols());
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
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
    Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>& zones, uint32_t x0,
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
    const Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>& zones) {
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
  ConnectivityGraph graph(centers);

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

  filter_isolated_subcomponents(graph, cell_labels);

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
  std::function<bool(const Eigen::Vector2i)> is_valid_cell =
      [&](const Eigen::Vector2i cell) {
        // check that the cell is in the map
        if (cell.x() < 0 || cell.x() >= cell_labels.cols() || cell.y() < 0 ||
            cell.y() >= cell_labels.rows()) {
          return false;
        }

        // check that the cell is the same type as the start OR goal
        if (cell_labels(cell.y(), cell.x()) !=
                cell_labels(start.y(), start.x()) &&
            cell_labels(cell.y(), cell.x()) !=
                cell_labels(goal.y(), goal.x())) {
          return false;
        }

        // check that the cell is in the same sector as the start or goal
        int start_sector_x = start.x() / sector_size_;
        int start_sector_y = start.y() / sector_size_;

        int goal_sector_x = goal.x() / sector_size_;
        int goal_sector_y = goal.y() / sector_size_;

        int cell_sector_x = cell.x() / sector_size_;
        int cell_sector_y = cell.y() / sector_size_;

        bool in_start_sector = (start_sector_x == cell_sector_x &&
                                start_sector_y == cell_sector_y);

        bool in_goal_sector =
            (goal_sector_x == cell_sector_x && goal_sector_y == cell_sector_y);

        // verify the cell is in the same sector as the start or goal cells
        return in_start_sector || in_goal_sector;
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
    std::function<bool(const Eigen::Vector2i)>& is_valid_cell) {
  // vector to store return values
  std::vector<Eigen::Vector2i> neighbors;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      // only consider 4 direct non-diagonal neighboring cells
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) {
        continue;
      }

      auto candidate = cell + Eigen::Vector2i(dx, dy);

      // check if neighboring cell is valid
      if (!is_valid_cell(candidate)) {
        continue;
      }

      neighbors.push_back(candidate);
    }
  }

  return neighbors;
}

void ConnectedComponentsLabeling::filter_isolated_subcomponents(
    ConnectivityGraph& graph,
    const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>&
        cell_labels) {
  // Going to re-use the UnionFind class to determine the number of connected
  // components in the graph. Any connected components which contain only
  // unknown nodes will be eliminated

  UnionFind uf(graph.nodes.size());
  for (uint32_t i = 0; i < graph.nodes.size(); i++) {
    for (uint32_t j = i + 1; j < graph.nodes.size(); j++) {
      if (graph.edges(i, j).type != EdgeType::INVALID) {
        uf.unite(i, j);
      }
    }
  }

  // if there is only one connected component, then there is nothing to do
  if (uf.get_num_sets() == 1) {
    return;
  }

  // Build a map from representative to the list of node indices in that
  // component.
  std::unordered_map<uint32_t, std::vector<uint32_t>> comp_nodes;
  for (uint32_t i = 0; i < graph.nodes.size(); i++) {
    uint32_t rep = uf.find(i);
    comp_nodes[rep].push_back(i);
  }

  // Identify component to keep--the only component that contains a known node.
  uint32_t keep_rep = 0;
  bool found = false;

  for (const auto& [rep, nodes] : comp_nodes) {
    for (uint32_t idx : nodes) {
      // Convert the center cell to coordinates
      Eigen::Vector2i cell(static_cast<int>(std::floor(graph.nodes[idx].x())),
                           static_cast<int>(std::floor(graph.nodes[idx].y())));

      if (cell_labels(cell.y(), cell.x()) != CellLabel::UNKNOWN) {
        keep_rep = rep;
        found = true;
        break;
      }
    }
  }

  if (!found) {
    return;
  }

  // build a new graph that includesa only nodes in the component to keep
  std::unordered_map<uint32_t, uint32_t> index_map;
  std::vector<Eigen::Vector2d> new_centers;

  // add valid nodes to the new graph
  for (uint32_t i = 0; i < graph.nodes.size(); i++) {
    if (uf.find(i) == keep_rep) {
      index_map[i] = new_centers.size();
      new_centers.push_back(graph.nodes[i]);
    }
  }

  ConnectivityGraph new_graph(new_centers);

  // add valid edges to new graph
  for (uint32_t i = 0; i < graph.nodes.size(); i++) {
    // only consider nodes in the component to keep
    if (uf.find(i) != keep_rep) {
      continue;
    }

    for (uint32_t j = i + 1; j < graph.nodes.size(); j++) {
      // only consider nodes in the component to keep
      if (uf.find(j) != keep_rep) {
        continue;
      }

      // if the edge exists and is between two nodes that are kept, copy it to
      // new graph using the translation map
      if (graph.edges(i, j).type != EdgeType::INVALID) {
        uint32_t new_i = index_map[i];
        uint32_t new_j = index_map[j];
        new_graph.add_edge(new_i, new_j, graph.edges(i, j).type,
                           graph.edges(i, j).cost);
      }
    }
  }

  // replace the original graph with the modified one
  graph = std::move(new_graph);
}