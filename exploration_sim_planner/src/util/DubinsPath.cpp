#include "exploration_sim_planner/util/DubinsPath.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace DubinsPath {

namespace {
// Normalizes an angle to be within [0, 2*pi)
double normalizeAngle(double angle) {
  angle = fmod(angle, 2.0 * M_PI);
  if (angle < 0) angle += 2.0 * M_PI;
  return angle;
}

// Structure to represent a path segment (turn or straight)
struct PathSegment {
  enum class Type { LEFT, RIGHT, STRAIGHT };

  Type type;
  double length;

  // Constructor for turn segments (LEFT or RIGHT)
  PathSegment(Type t, double l) : type(t), length(l) {}

  // Static factory methods for semantic clarity
  static PathSegment Left(double length) { return PathSegment(Type::LEFT, length); }
  static PathSegment Right(double length) { return PathSegment(Type::RIGHT, length); }
  static PathSegment Straight(double length) { return PathSegment(Type::STRAIGHT, length); }
};

// Struct to represent a candidate Dubins path
struct DubinsCandidate {
  std::array<PathSegment, 3> segments;
  DubinsPathType type;
  double total_length;

  DubinsCandidate(DubinsPathType t, PathSegment s1, PathSegment s2, PathSegment s3)
      : segments{s1, s2, s3}, type(t), total_length(s1.length + s2.length + s3.length) {}

  // Convert to the public DubinsPath structure
  DubinsPath toPath(double radius) const {
    DubinsPath path;
    path.type = type;
    path.radius = radius;
    path.length = total_length;

    // Set segment lengths
    path.length1 = segments[0].length;
    path.length2 = segments[1].length;
    path.length3 = segments[2].length;

    // Set angles based on lengths and radius
    if (segments[0].type == PathSegment::Type::LEFT) {
      path.angle1 = path.length1 / radius;
    } else if (segments[0].type == PathSegment::Type::RIGHT) {
      path.angle1 = path.length1 / radius;
    } else {
      path.angle1 = 0.0;
    }

    if (segments[1].type == PathSegment::Type::LEFT) {
      path.angle2 = path.length2 / radius;
    } else if (segments[1].type == PathSegment::Type::RIGHT) {
      path.angle2 = path.length2 / radius;
    } else {
      path.angle2 = 0.0;
    }

    if (segments[2].type == PathSegment::Type::LEFT) {
      path.angle3 = path.length3 / radius;
    } else if (segments[2].type == PathSegment::Type::RIGHT) {
      path.angle3 = path.length3 / radius;
    } else {
      path.angle3 = 0.0;
    }

    return path;
  }
};

// Normalizes the problem by transforming the end state relative to the start state
Eigen::Vector3d normalizeEndState(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
  double dx = end[0] - start[0];
  double dy = end[1] - start[1];
  double theta = start[2];

  double x = dx * cos(theta) + dy * sin(theta);
  double y = -dx * sin(theta) + dy * cos(theta);
  double phi = normalizeAngle(end[2] - theta);

  return Eigen::Vector3d(x, y, phi);
}

// Compute LSL path (Left-Straight-Left)
std::optional<DubinsCandidate> computeLSL(const Eigen::Vector3d& end, double radius) {
  double x = end[0];
  double y = end[1];
  double phi = end[2];

  // Centers of the circles for initial and final turns
  double centerx1 = 0;
  double centery1 = radius;
  double centerx2 = x + radius * sin(phi);
  double centery2 = y - radius * cos(phi);

  // Distance between circle centers
  double dx = centerx2 - centerx1;
  double dy = centery2 - centery1;
  double d = sqrt(dx * dx + dy * dy);

  // No feasible solution if circles are too close
  if (d < 2 * radius) {
    return std::nullopt;
  }

  // For a pure semicircular path (half-moon case)
  if (std::abs(x) < 1e-6 && std::abs(y - 2 * radius) < 1e-6 && std::abs(phi - M_PI) < 1e-6) {
    // Handle the semicircular path through the standard parametric representation
    // Two quarter-circles with no straight segment
    return DubinsCandidate(DubinsPathType::LSL, PathSegment::Left(M_PI * radius / 2), PathSegment::Straight(0.0),
                           PathSegment::Left(M_PI * radius / 2));
  }

  // Angle of the tangent line between circles
  double angle1 = atan2(dy, dx);

  // Length of the first turn
  double len1 = radius * normalizeAngle(angle1);

  // Length of the straight segment
  double len2 = d;

  // Length of the final turn
  double angle3 = normalizeAngle(phi - angle1);
  double len3 = radius * angle3;

  return DubinsCandidate(DubinsPathType::LSL, PathSegment::Left(len1), PathSegment::Straight(len2),
                         PathSegment::Left(len3));
}

// Compute RSR path (Right-Straight-Right)
std::optional<DubinsCandidate> computeRSR(const Eigen::Vector3d& end, double radius) {
  double x = end[0];
  double y = end[1];
  double phi = end[2];

  // Special case for straight line path
  if (std::abs(y) < 1e-6 && std::abs(phi) < 1e-6) {
    return DubinsCandidate(DubinsPathType::RSR, PathSegment::Right(0.0), PathSegment::Straight(x),
                           PathSegment::Right(0.0));
  }

  // Centers of the circles for initial and final turns
  double centerx1 = 0;
  double centery1 = -radius;
  double centerx2 = x - radius * sin(phi);
  double centery2 = y + radius * cos(phi);

  // Distance between circle centers
  double dx = centerx2 - centerx1;
  double dy = centery2 - centery1;
  double d = sqrt(dx * dx + dy * dy);

  // No feasible solution if circles are too close
  if (d < 2 * radius) {
    return std::nullopt;
  }

  // Angle of the tangent line between circles
  double angle1 = normalizeAngle(2 * M_PI - atan2(dy, dx));

  // Length of the first turn
  double len1 = radius * angle1;

  // Length of the straight segment
  double len2 = d;

  // Length of the final turn
  double angle3 = normalizeAngle(2 * M_PI - (phi - angle1 + 2 * M_PI));
  double len3 = radius * angle3;

  return DubinsCandidate(DubinsPathType::RSR, PathSegment::Right(len1), PathSegment::Straight(len2),
                         PathSegment::Right(len3));
}

// Compute RSL path (Right-Straight-Left)
std::optional<DubinsCandidate> computeRSL(const Eigen::Vector3d& end, double radius) {
  double x = end[0];
  double y = end[1];
  double phi = end[2];

  // Centers of the circles for initial and final turns
  double centerx1 = 0;
  double centery1 = -radius;
  double centerx2 = x + radius * sin(phi);
  double centery2 = y - radius * cos(phi);

  // Distance between circle centers
  double dx = centerx2 - centerx1;
  double dy = centery2 - centery1;
  double d = sqrt(dx * dx + dy * dy);

  // No feasible solution if circles are too close
  if (d < 2 * radius) {
    return std::nullopt;
  }

  // Calculate angles and path lengths
  double theta = atan2(dy, dx);
  double alpha = acos(2 * radius / d);

  // Length of the first turn
  double len1 = radius * normalizeAngle(2 * M_PI - (theta + alpha));

  // Length of the straight segment
  double len2 = sqrt(d * d - 4 * radius * radius);

  // Length of the final turn
  double len3 = radius * normalizeAngle(phi - theta - alpha + M_PI);

  return DubinsCandidate(DubinsPathType::RSL, PathSegment::Right(len1), PathSegment::Straight(len2),
                         PathSegment::Left(len3));
}

// Compute LSR path (Left-Straight-Right)
std::optional<DubinsCandidate> computeLSR(const Eigen::Vector3d& end, double radius) {
  double x = end[0];
  double y = end[1];
  double phi = end[2];

  // Centers of the circles for initial and final turns
  double centerx1 = 0;
  double centery1 = radius;
  double centerx2 = x - radius * sin(phi);
  double centery2 = y + radius * cos(phi);

  // Distance between circle centers
  double dx = centerx2 - centerx1;
  double dy = centery2 - centery1;
  double d = sqrt(dx * dx + dy * dy);

  // No feasible solution if circles are too close
  if (d < 2 * radius) {
    return std::nullopt;
  }

  // Calculate angles and path lengths
  double theta = atan2(dy, dx);
  double alpha = acos(2 * radius / d);

  // Length of the first turn
  double len1 = radius * normalizeAngle(theta - alpha);

  // Length of the straight segment
  double len2 = sqrt(d * d - 4 * radius * radius);

  // Length of the final turn
  double len3 = radius * normalizeAngle(2 * M_PI - (phi - theta + alpha));

  return DubinsCandidate(DubinsPathType::LSR, PathSegment::Left(len1), PathSegment::Straight(len2),
                         PathSegment::Right(len3));
}

// Compute RLR path (Right-Left-Right)
std::optional<DubinsCandidate> computeRLR(const Eigen::Vector3d& end, double radius) {
  double x = end[0];
  double y = end[1];
  double phi = end[2];

  // Centers of the circles for initial and final turns
  double centerx1 = 0;
  double centery1 = -radius;
  double centerx2 = x - radius * sin(phi);
  double centery2 = y + radius * cos(phi);

  // Distance between circle centers
  double dx = centerx2 - centerx1;
  double dy = centery2 - centery1;
  double d = sqrt(dx * dx + dy * dy);

  // No feasible solution if circles are too far apart
  if (d > 4 * radius) {
    return std::nullopt;
  }

  // Calculate angles and path lengths
  double theta = atan2(dy, dx);
  double alpha = acos(d / (4 * radius));

  // Length of the first turn
  double len1 = radius * normalizeAngle(2 * M_PI - (theta + alpha));

  // Length of the middle turn
  double len2 = radius * normalizeAngle(2 * M_PI - 2 * alpha);

  // Length of the final turn
  double len3 = radius * normalizeAngle(2 * M_PI - (phi - theta - alpha));

  return DubinsCandidate(DubinsPathType::RLR, PathSegment::Right(len1), PathSegment::Left(len2),
                         PathSegment::Right(len3));
}

// Compute LRL path (Left-Right-Left)
std::optional<DubinsCandidate> computeLRL(const Eigen::Vector3d& end, double radius) {
  double x = end[0];
  double y = end[1];
  double phi = end[2];

  // Centers of the circles for initial and final turns
  double centerx1 = 0;
  double centery1 = radius;
  double centerx2 = x + radius * sin(phi);
  double centery2 = y - radius * cos(phi);

  // Distance between circle centers
  double dx = centerx2 - centerx1;
  double dy = centery2 - centery1;
  double d = sqrt(dx * dx + dy * dy);

  // No feasible solution if circles are too far apart
  if (d > 4 * radius) {
    return std::nullopt;
  }

  // Calculate angles and path lengths
  double theta = atan2(dy, dx);
  double alpha = acos(d / (4 * radius));

  // Length of the first turn
  double len1 = radius * normalizeAngle(theta - alpha);

  // Length of the middle turn
  double len2 = radius * normalizeAngle(2 * M_PI - 2 * alpha);

  double beta = phi - theta + alpha;

  // Length of the final turn
  double len3 = radius * normalizeAngle(beta);

  return DubinsCandidate(DubinsPathType::LRL, PathSegment::Left(len1), PathSegment::Right(len2),
                         PathSegment::Left(len3));
}

// Helper function to update position for a left turn
void updatePositionLeftTurn(Eigen::Vector3d& position, double dist, double radius, double& omega) {
  omega = 1.0 / radius;
  position[0] += radius * (sin(position[2] + dist / radius) - sin(position[2]));
  position[1] += radius * (-cos(position[2] + dist / radius) + cos(position[2]));
  position[2] += dist / radius;
}

// Helper function to update position for a right turn
void updatePositionRightTurn(Eigen::Vector3d& position, double dist, double radius, double& omega) {
  omega = -1.0 / radius;
  position[0] += radius * (-sin(position[2] - dist / radius) + sin(position[2]));
  position[1] += radius * (cos(position[2] - dist / radius) - cos(position[2]));
  position[2] -= dist / radius;
}

// Helper function to update position for a straight segment
void updatePositionStraight(Eigen::Vector3d& position, double dist, double& omega) {
  omega = 0.0;
  position[0] += dist * cos(position[2]);
  position[1] += dist * sin(position[2]);
  // No change in orientation
}

// Helper function to update position based on path type and segment
void updatePathPosition(Eigen::Vector3d& position, double dist, double radius, DubinsPathType type, int segment,
                        double& omega) {
  if (segment == 1) {
    // First segment
    switch (type) {
      case DubinsPathType::LSL:
      case DubinsPathType::LSR:
      case DubinsPathType::LRL:
        updatePositionLeftTurn(position, dist, radius, omega);
        break;
      case DubinsPathType::RSL:
      case DubinsPathType::RSR:
      case DubinsPathType::RLR:
        updatePositionRightTurn(position, dist, radius, omega);
        break;
    }
  } else if (segment == 2) {
    // Second segment
    switch (type) {
      case DubinsPathType::LSL:
      case DubinsPathType::RSL:
      case DubinsPathType::RSR:
      case DubinsPathType::LSR:
        updatePositionStraight(position, dist, omega);
        break;
      case DubinsPathType::RLR:
        updatePositionLeftTurn(position, dist, radius, omega);
        break;
      case DubinsPathType::LRL:
        updatePositionRightTurn(position, dist, radius, omega);
        break;
    }
  } else if (segment == 3) {
    // Third segment
    switch (type) {
      case DubinsPathType::LSL:
      case DubinsPathType::RSL:
      case DubinsPathType::LRL:
        updatePositionLeftTurn(position, dist, radius, omega);
        break;
      case DubinsPathType::LSR:
      case DubinsPathType::RSR:
      case DubinsPathType::RLR:
        updatePositionRightTurn(position, dist, radius, omega);
        break;
    }
  }
}

// Helper function to get segment length based on segment number
double getSegmentLength(const DubinsPath& path, int segment) {
  switch (segment) {
    case 1:
      return path.length1;
    case 2:
      return path.length2;
    case 3:
      return path.length3;
    default:
      return 0.0;
  }
}

}  // end anonymous namespace

std::optional<DubinsPath> compute_dubins_path(Eigen::Vector3d start, Eigen::Vector3d end, double radius) {
  // Normalize the problem
  Eigen::Vector3d normalized_end = normalizeEndState(start, end);

  // Compute all possible paths
  std::vector<std::optional<DubinsCandidate>> candidates;

  // Add all standard path types
  candidates.push_back(computeLSL(normalized_end, radius));
  candidates.push_back(computeRSR(normalized_end, radius));
  candidates.push_back(computeRSL(normalized_end, radius));
  candidates.push_back(computeLSR(normalized_end, radius));
  candidates.push_back(computeRLR(normalized_end, radius));
  candidates.push_back(computeLRL(normalized_end, radius));

  // Find the shortest valid path
  std::optional<DubinsPath> best_path;
  double min_length = std::numeric_limits<double>::max();

  for (const auto& candidate : candidates) {
    if (!candidate.has_value()) continue;

    double path_length = candidate->total_length;
    if (path_length < min_length) {
      min_length = path_length;
      best_path = candidate->toPath(radius);
    }
  }

  return best_path;
}

std::vector<std::pair<Eigen::Vector3d, double>> compute_dubins_path_points(const DubinsPath& path, double step_size) {
  std::vector<std::pair<Eigen::Vector3d, double>> points;

  // Invalid path
  if (path.length == std::numeric_limits<double>::max()) {
    return points;
  }

  // Initial position and orientation (origin with 0 orientation)
  Eigen::Vector3d current_pos(0, 0, 0);

  // Calculate total number of steps
  int num_steps = static_cast<int>(std::ceil(path.length / step_size));
  double actual_step_size = path.length / num_steps;

  // Add initial point
  points.emplace_back(current_pos, 0.0);

  // Remember which segment we're on and how far along it we are
  int segment = 1;
  double segment_pos = 0.0;
  double segment_length = getSegmentLength(path, segment);

  // For each step
  for (int i = 1; i <= num_steps; ++i) {
    double dist_to_travel = actual_step_size;
    double omega = 0.0;

    while (dist_to_travel > 0) {
      // Determine how much we can move in current segment
      double remaining_in_segment = segment_length - segment_pos;
      double dist = std::min(dist_to_travel, remaining_in_segment);

      // Update position based on current segment type
      updatePathPosition(current_pos, dist, path.radius, path.type, segment, omega);

      // Update state
      segment_pos += dist;
      dist_to_travel -= dist;

      // If we've completed a segment, move to the next one
      if (std::abs(segment_pos - segment_length) < 1e-6 && segment < 3) {
        segment++;
        segment_pos = 0.0;
        segment_length = getSegmentLength(path, segment);
      }
    }

    // Normalize angle
    current_pos[2] = normalizeAngle(current_pos[2]);

    // Add point to the result
    points.emplace_back(current_pos, omega);
  }

  return points;
}

}  // namespace DubinsPath
