/**
 * @file ShortTermPathPlanner.hpp
 *
 * @brief This file defines the ShortTermPathPlanner class, which is used to
 * generate short-term paths for exploration planning.
 *
 * This class is responsible for generating smooth paths that connect
 * approximately the first 5 nodes in the global path.
 *
 * It first generates an A* shortest path from the current robot position
 * through the series of waypoints.
 *
 * A cubic b-spline is then fit to the path to smooth it out, while still
 * accounting for safety
 */

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <vector>

#include "exploration_sim_planner/util/BSplineUtil.hpp"
#include "exploration_sim_planner/util/OgmView.hpp"
#include "nav_msgs/msg/path.hpp"

class ShortTermPathPlanner {
 public:
  /**
   * @brief Publicly accessible method to generate a smoothed path
   * between as series of waypoints
   *
   * Every parameter should be in the map frame -- i.e. 0, 0 is the bottom left
   * hand corner
   * @param esdf The Euclidean signed distance field representing the
   * environment
   * @param global_path The global path to be smoothed
   * @param current_position The current position of the robot
   *
   * @return A vector of 2D points sampled along the smoothed path
   */
  static std::vector<Eigen::Vector2d> fit_smoothed_path(const Eigen::MatrixXd& esdf,
                                                        const std::vector<Eigen::Vector2d>& global_path);

  static std::optional<std::vector<Eigen::Vector3d>> plan_backup(const std::vector<Eigen::Vector2d>& smoothed_path,
                                                                 const std::vector<Eigen::Vector2d>& coverage_path,
                                                                 const std::shared_ptr<OgmView>& ogm);

  static bool check_dubins_path_safety(const std::shared_ptr<OgmView>& ogm, const std::vector<Eigen::Vector3d>& path);
  static bool check_path_safety(const Eigen::MatrixXd& esdf, const nav_msgs::msg::Path& path);

  /**
   * @brief Finds the first free node in the path after the first unknown node
   */
  static std::optional<Eigen::Vector2d> find_next_free_waypoint(const std::shared_ptr<OgmView>& ogm,
                                                                const std::vector<Eigen::Vector2d>& coverage_path);

  static std::optional<std::vector<Eigen::Vector3d>> compute_backup_dubins(
      const std::vector<Eigen::Vector2d>& smoothed_path, const Eigen::Vector2d& backup_point,
      const std::shared_ptr<OgmView>& ogm);

 private:
  /**
   * @brief Fits an astar path to the global path using the ESDF
   *
   * @param esdf The Euclidean signed distance field representing the
   * environment
   * @param global_path The waypoints to fit an A* path between
   * @param current_position The current position of the robot
   * @return A vector of 2D points sampled along the A* path
   */
  static std::optional<std::vector<Eigen::Vector2i>> fit_astar(const Eigen::MatrixXd& esdf,
                                                               const std::vector<Eigen::Vector2d>& global_path);

  static BSplineUtil::CubicBSpline fit_cubic_bspline(const Eigen::MatrixXd& esdf,
                                                     const std::vector<Eigen::Vector2i>& astar_path);

  static std::vector<Eigen::Vector2d> shorten_path(const std::vector<Eigen::Vector2d>& global_path, double max_length);
};