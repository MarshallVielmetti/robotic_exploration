/**
 * @file BSplineUtil.cpp
 *
 * @brief Implementation of B-spline utility functions
 */

#include "exploration_sim_planner/util/BSplineUtil.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace BSplineUtil {

Eigen::Vector2d CubicBSpline::evaluate(double t) const {
  if (control_points.size() < 4) {
    // Need at least 4 points for cubic spline
    return control_points.empty() ? Eigen::Vector2d(0, 0)
                                  : control_points.front();
  }

  // Clamp t to [0, 1]
  t = std::max(0.0, std::min(1.0, t));

  // Convert to spline parameter domain
  double u = t * (control_points.size() - 3);
  int i = std::min(static_cast<int>(std::floor(u)),
                   static_cast<int>(control_points.size() - 4));
  u -= i;

  // De Boor's algorithm for cubic B-spline evaluation
  double u2 = u * u;
  double u3 = u2 * u;

  // Cubic basis functions
  double b0 = (1 - u) * (1 - u) * (1 - u) / 6.0;
  double b1 = (3 * u3 - 6 * u2 + 4) / 6.0;
  double b2 = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6.0;
  double b3 = u3 / 6.0;

  // Evaluate the spline at parameter t
  return b0 * control_points[i] + b1 * control_points[i + 1] +
         b2 * control_points[i + 2] + b3 * control_points[i + 3];
}

std::vector<Eigen::Vector2d> CubicBSpline::sample(size_t n) const {
  std::vector<Eigen::Vector2d> samples;
  samples.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    double t = static_cast<double>(i) / (n - 1);
    samples.push_back(evaluate(t));
  }

  return samples;
}

double CubicBSpline::arcLength() const {
  // Approximate arc length by sampling points and summing segment lengths
  const size_t num_samples = 100;
  auto points = this->sample(num_samples);

  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }

  return length;
}

CubicBSpline fitCubicBSpline(const std::vector<Eigen::Vector2d>& points,
                             double smoothness) {
  CubicBSpline spline;

  if (points.size() < 2) {
    return spline;
  }

  // For a simple implementation, use the input points directly as control
  // points with some endpoint adjustments for closed curve behavior
  spline.control_points = points;
  spline.knot_interval = 1.0;

  // Apply smoothing if requested by adjusting control points
  if (smoothness > 0 && points.size() > 4) {
    std::vector<Eigen::Vector2d> smoothed_points;
    smoothed_points.push_back(points.front());  // Keep first point

    for (size_t i = 1; i < points.size() - 1; ++i) {
      Eigen::Vector2d prev = points[i - 1];
      Eigen::Vector2d curr = points[i];
      Eigen::Vector2d next = points[i + 1];

      // Weighted average based on smoothness parameter
      Eigen::Vector2d smoothed =
          (1 - smoothness) * curr + smoothness * 0.5 * (prev + next);
      smoothed_points.push_back(smoothed);
    }

    smoothed_points.push_back(points.back());  // Keep last point
    spline.control_points = smoothed_points;
  }

  return spline;
}

bool checkSplineSafety(const Eigen::MatrixXd& esdf, const CubicBSpline& spline,
                       double safety_margin) {
  // Sample the spline at a reasonable resolution
  const size_t num_samples = 100;
  std::vector<Eigen::Vector2d> samples = spline.sample(num_samples);

  // Check each sampled point against the ESDF
  for (const auto& point : samples) {
    int x = static_cast<int>(std::round(point.x()));
    int y = static_cast<int>(std::round(point.y()));

    // Check if point is within grid bounds
    if (x < 0 || y < 0 || x >= esdf.cols() || y >= esdf.rows()) {
      return false;  // Out of bounds
    }

    // Check if point is too close to obstacles
    if (esdf(y, x) < safety_margin) {
      return false;  // Too close to obstacle
    }
  }

  return true;  // Path is safe
}

double evaluateEsdfInterpolated(const Eigen::MatrixXd& esdf,
                                const Eigen::Vector2d& point) {
  // Ensure the point is within bounds
  if (point.x() < 0 || point.y() < 0 || point.x() >= esdf.cols() - 1 ||
      point.y() >= esdf.rows() - 1) {
    // Return very low value for points outside the map
    return -std::numeric_limits<double>::infinity();
  }

  // Get cell coordinates and fractional parts for bilinear interpolation
  int x0 = static_cast<int>(std::floor(point.x()));
  int y0 = static_cast<int>(std::floor(point.y()));
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  double dx = point.x() - x0;
  double dy = point.y() - y0;

  // Bilinear interpolation
  double v00 = esdf(y0, x0);
  double v01 = esdf(y0, x1);
  double v10 = esdf(y1, x0);
  double v11 = esdf(y1, x1);

  // Interpolate along x
  double v0 = v00 * (1 - dx) + v01 * dx;
  double v1 = v10 * (1 - dx) + v11 * dx;

  // Interpolate along y
  return v0 * (1 - dy) + v1 * dy;
}

Eigen::Vector2d calculateEsdfGradient(const Eigen::MatrixXd& esdf,
                                      const Eigen::Vector2d& point) {
  // Calculate gradient using central differences
  const double eps = 0.01;

  // Evaluate ESDF at neighboring points
  double fx_plus =
      evaluateEsdfInterpolated(esdf, point + Eigen::Vector2d(eps, 0));
  double fx_minus =
      evaluateEsdfInterpolated(esdf, point - Eigen::Vector2d(eps, 0));
  double fy_plus =
      evaluateEsdfInterpolated(esdf, point + Eigen::Vector2d(0, eps));
  double fy_minus =
      evaluateEsdfInterpolated(esdf, point - Eigen::Vector2d(0, eps));

  // Calculate gradient components
  double dx = (fx_plus - fx_minus) / (2 * eps);
  double dy = (fy_plus - fy_minus) / (2 * eps);

  return Eigen::Vector2d(dx, dy);
}

CubicBSpline optimizeSpline(const CubicBSpline& initial_spline,
                            const Eigen::MatrixXd& esdf, double safety_weight,
                            double length_weight, double learning_rate,
                            int max_iterations) {
  // Create a copy of the spline to optimize
  CubicBSpline spline = initial_spline;

  // Minimum safety distance threshold
  const double safety_threshold = 3.0;

  // Number of points to sample along the spline for evaluation
  const size_t num_eval_points = 50;

  // Initial path length for normalization
  double initial_length = spline.arcLength();

  // Keep first and last control points fixed
  size_t first_movable = 1;
  size_t last_movable = spline.control_points.size() - 2;

  // Perform gradient descent iterations
  for (int iter = 0; iter < max_iterations; ++iter) {
    // Sample points along the current spline
    std::vector<Eigen::Vector2d> sample_points = spline.sample(num_eval_points);

    // Calculate gradients for each control point
    std::vector<Eigen::Vector2d> control_point_gradients(
        spline.control_points.size(), Eigen::Vector2d::Zero());

    // Evaluate safety cost and gradient
    // double safety_cost = 0.0;
    for (const auto& point : sample_points) {
      double esdf_value = evaluateEsdfInterpolated(esdf, point);

      // Penalize points that are too close to obstacles
      if (esdf_value < safety_threshold) {
        // safety_cost += safety_threshold - esdf_value;

        // Calculate the gradient of the ESDF
        Eigen::Vector2d esdf_grad = calculateEsdfGradient(esdf, point);

        // For each sampled point that's too close to an obstacle,
        // update the gradients of control points that influence it
        double t = static_cast<double>(&point - &sample_points[0]) /
                   (num_eval_points - 1);

        // Find the active control points that influence this sample
        double u = t * (spline.control_points.size() - 3);
        int i = std::min(static_cast<int>(std::floor(u)),
                         static_cast<int>(spline.control_points.size() - 4));
        u -= i;

        // Cubic basis functions
        double u2 = u * u;
        double u3 = u2 * u;
        double b0 = (1 - u) * (1 - u) * (1 - u) / 6.0;
        double b1 = (3 * u3 - 6 * u2 + 4) / 6.0;
        double b2 = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6.0;
        double b3 = u3 / 6.0;

        // Update gradients for the 4 control points that influence this sample
        control_point_gradients[i] += esdf_grad * b0;
        control_point_gradients[i + 1] += esdf_grad * b1;
        control_point_gradients[i + 2] += esdf_grad * b2;
        control_point_gradients[i + 3] += esdf_grad * b3;
      }
    }

    // Evaluate path length and compute length gradient
    // double current_length = spline.arcLength();
    // double length_cost = current_length / initial_length;

    // For each control point (except endpoints), compute length gradient
    for (size_t i = first_movable; i <= last_movable; ++i) {
      // Approximate length gradient by the direction to neighboring control
      // points
      Eigen::Vector2d length_grad = Eigen::Vector2d::Zero();

      if (i > 0) {
        length_grad += (spline.control_points[i] - spline.control_points[i - 1])
                           .normalized();
      }

      if (i < spline.control_points.size() - 1) {
        length_grad += (spline.control_points[i] - spline.control_points[i + 1])
                           .normalized();
      }

      // Add length gradient to total
      control_point_gradients[i] += length_weight * length_grad;
    }

    // Update control points using the computed gradients
    bool any_updates = false;
    for (size_t i = first_movable; i <= last_movable; ++i) {
      // Don't update first or last control point
      if (i == 0 || i == spline.control_points.size() - 1) {
        continue;
      }

      // Apply safety gradient with higher weight
      Eigen::Vector2d update = safety_weight * control_point_gradients[i];

      // Only apply update if it's significant
      if (update.norm() > 1e-6) {
        spline.control_points[i] += learning_rate * update;
        any_updates = true;
      }
    }

    // Early stopping if no significant updates
    if (!any_updates) {
      break;
    }

    // Check if the optimized spline is safe
    if (checkSplineSafety(esdf, spline, safety_threshold)) {
      // If we've reached a safe configuration, we can stop
      break;
    }

    // Adaptive learning rate
    learning_rate *= 0.98;  // Reduce learning rate slightly each iteration
  }

  return spline;
}

}  // namespace BSplineUtil
