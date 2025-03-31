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
#include <functional>
#include <optional>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "exploration_sim_planner/util/EigenUtil.hpp"
#include "exploration_sim_planner/util/OgmView.hpp"

enum class CellLabel { UNKNOWN = 0, OCCUPIED = 1, SAFE_FREE = 2, UNSAFE_FREE = 3 };

enum class EdgeType { INVALID = 0, FREE = 1, UNKNOWN = 2, PORTAL = 3 };

struct Edge {
  EdgeType type;
  double cost;
};

enum class LocationType { VIEWPOINT_CENTER, UNKNOWN_ZONE_CENTER, ROBOT_POSITION };
struct TargetPosition {
  Eigen::Vector2d position;  // location of the target
  LocationType type;         // type of the location
  uint32_t zone_id;          // id of the zone the target belongs to
};

struct ConnectivityGraph {
  ConnectivityGraph(std::vector<Eigen::Vector2d> the_nodes) : nodes(the_nodes) {
    edges = Eigen::MatrixX<Edge>(nodes.size(), nodes.size());
    edges.setConstant(Edge{EdgeType::INVALID, 0.0});
  }

  std::vector<Eigen::Vector2d> nodes;  // centers of zones

  // connectivity graph
  Eigen::MatrixX<Edge> edges;

  void add_edge(uint32_t i, uint32_t j, EdgeType type, double cost) {
    Edge edge(type, cost);
    edges(i, j) = edge;
    edges(j, i) = edge;
  }
};

struct PCAResult {
  Eigen::VectorXd eigenvalues;
  Eigen::MatrixXd eigenvectors;
  Eigen::MatrixXd transformed_data;

  PCAResult(Eigen::VectorXd& evals, Eigen::MatrixXd& evecs, Eigen::MatrixXd& data)
      : eigenvalues(evals), eigenvectors(evecs), transformed_data(data) {}
};

/**
 * @brief Implements labeling and analysis of connected components in an
 * occupancy grid.
 *
 * The ConnectedComponentsLabeling class offers methods to analyze an occupancy
 * grid map by labeling its connected components, computing regions (zones), and
 * determining centers within those regions. It also provides functionality to
 * derive an incremental connectivity graph based on the labeled regions, using
 * variants of region identification and path planning algorithms.
 *
 * Key functionalities:
 * - Label each connected region in an occupancy grid using cell labels.
 * - Compute a compact representation of zones from the cell label matrix.
 * - Find the geometric centers of each connected component.
 * - Construct an incremental connectivity graph from the labeled grid and
 * corresponding centers.
 *
 * Additionally, the class encapsulates utility methods to determine cell
 * properties (e.g., safe-free status) and compute paths under given constraints
 * using a restricted A* algorithm. Parameters such as sector size and safe
 * distance delimit the operational area within the grid.
 *
 * Usage example:
 * @code
 * uint32_t sectorSize = 10;
 * uint32_t safeDistance = 2;
 * ConnectedComponentsLabeling ccl(sectorSize, safeDistance);
 *
 * // Assume 'ogm' is an occupancy grid map view.
 * Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic> cellLabels =
 * ccl.label_cells(ogm);
 *
 * Eigen::Matrix2i zones = ccl.compute_zones(cellLabels);
 *
 * std::vector<Eigen::Vector2d> centers = ccl.find_centers(cellLabels, zones);
 *
 * ConnectivityGraph graph =
 * ccl.compute_incremental_connectivity_graph(cellLabels, centers);
 * @endcode
 *
 */
class ConnectedComponentsLabeling {
 public:
  ConnectedComponentsLabeling(uint32_t sector_size, uint32_t safe_distance);

  /**
   * @brief Labels connected components in an occupancy grid map.
   *
   * This function performs connected components labeling on a given occupancy
   * grid map (ogm). It analyzes the provided grid view and assigns a unique
   * label to each distinct connected component. The resulting Eigen matrix
   * contains these labels, where each element corresponds to the cell's
   * component identifier.
   *
   * @param ogm The occupancy grid map view containing cell data used for
   * connected components analysis.
   * @return An Eigen dynamic matrix where each element is a CellLabel
   * representing the connected component identifier of the corresponding cell
   * in the occupancy grid.
   */
  Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic> label_cells(const OgmView& ogm);

  /**
   * @brief Identifies and extracts frontier cells from a labeled occupancy
   * grid.
   *
   * Frontier cells are typically located at the boundary between explored free
   * space and unexplored areas. These cells are essential for exploration
   * planning as they represent potential directions for further exploration.
   *
   * @param cell_labels A matrix where each cell is labeled according to its
   * state (e.g., free, occupied, unknown, frontier)
   * @return A vector of 2D coordinates representing the positions of all
   * identified frontier cells in the grid
   */
  std::vector<Eigen::Vector2i> find_frontier_cells(
      const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels);

  /**
   * @brief Clusters frontier cells into connected components.
   *
   * This function takes a list of frontier cells and groups them into clusters
   * where each cluster represents a connected component of frontier cells.
   *
   * @param frontier_cells Vector of 2D points representing individual frontier
   * cells
   *
   * @return Vector of clusters, where each cluster is a vector of connected
   * frontier cells
   */
  std::vector<std::vector<Eigen::Vector2i>> cluster_frontiers(const std::vector<Eigen::Vector2i>& frontier_cells);

  std::vector<std::vector<Eigen::Vector2d>> sample_frontier_viewpoints(
      const std::vector<std::vector<Eigen::Vector2i>>& frontier_cells, const Eigen::MatrixX<CellLabel>& cell_labels);

  /**
   * @brief Computes zones based on connected components labeling of cell
   * labels.
   *
   * This function analyzes the given matrix of cell labels to identify and
   * compute zones (or connected regions) within an exploration grid. It
   * processes the input matrix to determine connected components and returns a
   * 2x2 matrix that encodes the resulting zone information.
   *
   * @param cell_labels A dynamic Eigen matrix where each element represents a
   * label assigned to a cell. The matrix dimensions correspond to the grid
   * layout for which zones are computed.
   *
   * @return Eigen::Matrix2i A fixed-size 2x2 integer matrix representing the
   * computed zone data. The specific interpretation of the zone data is
   * determined by the underlying connected components labeling algorithm.
   */
  Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> compute_zones(
      const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels);

  /**
   * @brief Finds the centers of connected components in the provided cell label
   * matrix.
   *
   * This function analyzes the input matrix of cell labels to detect connected
   * components and computes the center (as an Eigen::Vector2d) for each
   * connected component identified. The zones parameter provides additional
   * contextual information that may be used to restrict or influence the search
   * for connected components.
   *
   * @param cell_labels A dynamic Eigen matrix containing labels for cells.
   * @param zones A 2x2 integer matrix defining the zones applicable for
   * processing.
   * @return A vector of Eigen::Vector2d objects, where each vector represents
   * the center of a detected connected component.
   */
  std::vector<Eigen::Vector2d> find_centers(const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
                                            const Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>& zones);

  /**
   * @brief Computes the incremental connectivity graph.
   *
   * This function builds a connectivity graph incrementally based on the
   * provided cell labels and centers. Each cell label in the matrix represents
   * a discrete section or region, while the centers vector provides spatial
   * coordinates for corresponding cells. The resulting connectivity graph
   * encapsulates the connectivity relationships among these regions.
   *
   * @param cell_labels A dynamic matrix containing CellLabel values that
   * represent labeled regions in the map.
   * @param centers A vector of 2D points (Eigen::Vector2d) specifying the
   * centers of each corresponding cell -- must only contain centers of interest
   * @return ConnectivityGraph The computed graph where nodes represent regions
   * and edges represent connectivity associations between them.
   */
  ConnectivityGraph compute_incremental_connectivity_graph(
      const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
      const std::vector<Eigen::Vector2d>& centers);

  // std::vector<std::vector<Eigen::Vector2d>> find_viewpoint_representatives(
  //     const Eigen::MatrixX<CellLabel>& cell_labels,
  //     const Eigen::MatrixX<uint32_t>& zones,
  //     const std::vector<std::vector<Eigen::Vector2d>>& frontier_viewpoints);

  std::pair<std::vector<TargetPosition>, Eigen::MatrixXd> compute_atsp_cost_matrix(
      const ConnectivityGraph& graph, const std::vector<std::vector<Eigen::Vector2d>>& viewpoint_representatives,
      const Eigen::MatrixX<CellLabel>& cell_labels, const Eigen::MatrixX<uint32_t>& zones,
      const Eigen::Vector2d& current_position);

  /**
   * @brief Finds representative viewpoints for each connected component of frontier viewpoints.
   *
   * This function analyzes the given zones matrix and associates each connected component with its corresponding
   * frontier viewpoints. A connected component here typically represents a contiguous region in the zones grid.
   * For each such region, the function selects a set of representative viewpoints from the provided list of frontier
   * viewpoints.
   *
   * @param zones A matrix where each element represents a zone identifier, which is used to determine connected
   * regions.
   * @param frontier_viewpoints A vector of vectors of 2D points. Each inner vector contains the frontier viewpoints
   * associated with a specific zone or connected component.
   * @return A vector of vectors of 2D points, where each inner vector contains representative viewpoints for a
   * connected component.
   */
  std::vector<std::vector<Eigen::Vector2d>> find_viewpoint_representatives(
      const Eigen::MatrixX<uint32_t>& zones, const std::vector<std::vector<Eigen::Vector2d>>& frontier_viewpoints);

 private:
  // Get the label of a cell
  CellLabel get_cell_label(const OgmView& ogm, uint32_t x, uint32_t y);

  // Check if a cell is safe-free
  bool is_safe_free(const OgmView& ogm, uint32_t x, uint32_t y);

  /**
   * @brief Computes sector zones based on connected component labeling.
   *
   * This function analyzes the provided cell_labels matrix to identify and
   * label distinct connected regions (zones) within a specific area referenced
   * by the coordinates (x, y). The detected zones are recorded in the zones
   * matrix, and the zone_id is updated with a unique identifier for each new
   * zone discovered.
   *
   * @param cell_labels The input matrix containing cell labels
   * @param zones The output matrix modified to include computed zone labels
   * @param x The x coordinate used as the reference point for computing zones.
   * @param y The y coordinate used as the reference point for computing zones.
   * @param zone_id A reference to an unsigned integer that holds the current
   * zone identifier. It is incremented when new zones are assigned.
   */
  void compute_sector_zones(const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
                            Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>& zones, uint32_t x, uint32_t y,
                            uint32_t& zone_id);

  /**
   * @brief Computes a path cost using A* with restricted exploration.
   *
   * This function performs a variant of the A* search algorithm on a grid
   * defined by cell labels. The search is constrained by certain criteria
   * implied by the cell labels in the input matrix. Specifically, if both start
   * and goal cells are safe-free, the algorithm will only consider safe-free
   * cells during exploration. If both are unknown, it will only consider
   * unknown cells. In either case, it will only consider cells within the the
   * same sectors as the points.
   *
   * @param cell_labels A dynamic matrix where each element represents the label
   * of a cell. The labels provide the exploration constraints for the
   * algorithm.
   * @param start The starting cell coordinates in the grid.
   * @param goal The target cell coordinates in the grid.
   * @return std::optional<double> The cost of the computed path if one exists;
   *         otherwise, an empty optional indicating no valid path was found.
   */
  std::optional<double> restricted_astar(const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels,
                                         const Eigen::Vector2i& start, const Eigen::Vector2i& goal);

  std::vector<Eigen::Vector2i> get_valid_neighbors(const Eigen::Vector2i& cell,
                                                   std::function<bool(const Eigen::Vector2i)>& is_valid_cell);

  bool is_frontier(const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels, uint32_t x, uint32_t y);

  PCAResult cluster_pca(std::vector<Eigen::Vector2i>& cluster);

  std::pair<std::vector<Eigen::Vector2i>, std::vector<Eigen::Vector2i>> split_cluster(
      const std::vector<Eigen::Vector2i>& cluster, const PCAResult& pca);

  /**
   * @brief Samples viewpoints for one frontier cluster
   *
   * @param frontier_cluster A vector of grid cell coordinates (Eigen::Vector2i)
   * that represents the frontier cluster for which viewpoints should be
   * sampled.
   *
   * @param cell_labels A matrix (Eigen::MatrixX) of CellLabel containing the
   * labels for each cell.
   *
   * @return A vector of 2D points (Eigen::Vector2d) representing the sampled
   * viewpoints
   */
  static std::vector<Eigen::Vector2d> sample_one_frontier_viewpoints(
      const std::vector<Eigen::Vector2i>& frontier_cluster, const Eigen::MatrixX<CellLabel>& cell_labels);

  /**
   * @brief Filters and refines a list of viewpoint positions based on frontier
   * cells and their labels.
   *
   * This function processes the provided vector of viewpoints and prunes or
   * adjusts it according to associated frontier cell positions and their
   * corresponding cell labels as represented in the cell_labels matrix. The
   * filtering criteria and maneuvers are determined by the connectivity or
   * labeling information in cell_labels, which typically represents segmented
   * regions or connected components in the grid.
   *
   * @param[out] viewpoints A modifiable vector of Eigen::Vector2d representing
   * candidate viewpoints. The function filters these viewpoints based on the
   * provided frontier cells and the connectivity information. After execution,
   * this vector will contain only the viewpoints that meet the filtering
   * criteria.
   * @param[in] frontier_cells A vector of Eigen::Vector2i representing the
   * positions of frontier cells. These are used in conjunction with the
   * connectivity matrix to determine valid viewpoints.
   * @param[in] cell_labels A matrix of CellLabel elements where each element
   * corresponds to a cell's label in the grid. This matrix provides the
   * necessary connectivity or segment information that is used to filter the
   * viewpoints.
   */
  static void filter_viewpoints(std::vector<Eigen::Vector2d>& viewpoints,
                                const std::vector<Eigen::Vector2i>& frontier_cells,
                                const Eigen::MatrixX<CellLabel>& cell_labels);

  /**
   * @brief Calculates the coverage metric from a given viewpoint.
   *
   * This function computes a coverage value based on the location of the
   * viewpoint, the list of frontier cells identified in the environment, and
   * the matrix of cell labels that describe the state or connectivity of each
   * cell. The coverage value can be used to determine the extent of the
   * explored area or the effectiveness of the exploration strategy.
   *
   * @param viewpoint A 2D vector representing the coordinates of the viewpoint.
   * @param frontier_cells A vector of 2D integer vectors, each representing the
   * grid coordinates of a frontier cell.
   * @param cell_labels A matrix of CellLabel objects that categorizes each cell
   * within the grid.
   * @return A double value representing the calculated coverage.
   */
  static double calculate_coverage(const Eigen::Vector2d& viewpoint, const std::vector<Eigen::Vector2i>& frontier_cells,
                                   const Eigen::MatrixX<CellLabel>& cell_labels);

  /**
   * @brief Computes rectified viewpoint representatives for frontier cluster
   *
   * Viewpoint centers are computed as the average position of all viewpoint
   * representatives within the zone (for that specific frontier)
   */
  // Eigen::Vector2d compute_viewpoint_representative();

  /**
   * @brief Removes isolated subcomponents from the connectivity graph.
   *
   * Removes any subcomponents from the graph which consist only of isolated
   * unknown zones. Such subcomponents represent unacccessible areas such as
   * those located internal to large obstacles.
   *
   * @param graph The connectivity graph to be processed.
   */
  void filter_isolated_subcomponents(ConnectivityGraph& graph,
                                     const Eigen::Matrix<CellLabel, Eigen::Dynamic, Eigen::Dynamic>& cell_labels);

  double grid_astar(Eigen::Vector2d& start, Eigen::Vector2d& goal, const Eigen::MatrixX<CellLabel>& cell_labels);

  double graph_astar(uint32_t start_node, uint32_t end_node, const ConnectivityGraph& graph);

 private:
  // The size of the sectors in the map
  uint32_t sector_size_;
  uint32_t safe_distance_;
};