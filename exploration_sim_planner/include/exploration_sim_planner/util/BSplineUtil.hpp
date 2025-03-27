/**
 * @file BSplineUtil.hpp
 *
 * @brief Utility functions for working with B-splines
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace BSplineUtil {
struct CubicBSpline {
  std::vector<Eigen::Vector2d> control_points;
  double knot_interval;

  /**
   * @brief Evaluate the cubic B-spline at parameter t
   *
   * @param t Parameter value between 0 and 1
   * @return Eigen::Vector2d Point on the spline
   */
  Eigen::Vector2d evaluate(double t) const;

  /**
   * @brief Sample n evenly spaced points along the spline
   *
   * @param n Number of points to sample
   * @return std::vector<Eigen::Vector2d> Sampled points
   */
  std::vector<Eigen::Vector2d> sample(size_t n) const;

  /**
   * @brief Calculate the total arc length of the B-spline
   * 
   * @return double Approximate arc length of the spline
   */
  double arcLength() const;
};

/**
 * @brief Fit a cubic B-spline to a set of points
 *
 * @param points The points to fit the spline to
 * @param smoothness Smoothness parameter (higher means smoother curve)
 * @return CubicBSpline The fitted B-spline
 */
CubicBSpline fitCubicBSpline(const std::vector<Eigen::Vector2d>& points,
                             double smoothness = 0.5);

/**
 * @brief Calculate the basis functions for cubic B-spline
 *
 * @param u Parameter between 0 and 1
 * @param i Index of the control point
 * @param degree Degree of the B-spline (3 for cubic)
 * @param knots Knot vector
 * @return double Value of the basis function
 */
double basisFunction(double u, int i, int degree,
                     const std::vector<double>& knots);

/**
 * @brief Check if a B-spline path collides with obstacles in the ESDF
 *
 * @param esdf Euclidean signed distance field
 * @param spline B-spline to check
 * @param safety_margin Safety margin to maintain from obstacles (in grid cells)
 * @return bool True if path is safe, false otherwise
 */
bool checkSplineSafety(const Eigen::MatrixXd& esdf, const CubicBSpline& spline,
                       double safety_margin = 2.0);

/**
 * @brief Optimize a B-spline using gradient descent to maximize safety and minimize length
 * 
 * @param spline The initial B-spline to optimize
 * @param esdf The Euclidean signed distance field representing obstacles
 * @param safety_weight Weight for the safety term in the cost function
 * @param length_weight Weight for the path length term in the cost function
 * @param learning_rate Learning rate for gradient descent
 * @param max_iterations Maximum number of iterations for optimization
 * @return CubicBSpline The optimized B-spline
 */
CubicBSpline optimizeSpline(
    const CubicBSpline& spline, 
    const Eigen::MatrixXd& esdf, 
    double safety_weight = 0.7,
    double length_weight = 0.3,
    double learning_rate = 0.05,
    int max_iterations = 100);

/**
 * @brief Evaluate the ESDF value at a continuous point using bilinear interpolation
 * 
 * @param esdf The Euclidean signed distance field
 * @param point The point to evaluate (can be non-integer)
 * @return double The interpolated ESDF value
 */
double evaluateEsdfInterpolated(const Eigen::MatrixXd& esdf, const Eigen::Vector2d& point);

/**
 * @brief Calculate the gradient of the ESDF at a point
 * 
 * @param esdf The Euclidean signed distance field
 * @param point The point to evaluate the gradient at
 * @return Eigen::Vector2d The gradient vector
 */
Eigen::Vector2d calculateEsdfGradient(const Eigen::MatrixXd& esdf, const Eigen::Vector2d& point);

}  // namespace BSplineUtil