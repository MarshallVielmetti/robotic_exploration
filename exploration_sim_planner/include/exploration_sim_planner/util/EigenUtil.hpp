/**
 * @file EigenUtil.hpp
 * @brief Some utility function for Eigen types
 */

#pragma once

#include <Eigen/Dense>

namespace std {
template <>
struct hash<Eigen::Vector2i> {
  std::size_t operator()(const Eigen::Vector2i &v) const {
    std::size_t h1 = std::hash<int>()(v.x());
    std::size_t h2 = std::hash<int>()(v.y());
    return h1 ^ (h2 << 1);
  }
};

template <>
struct hash<Eigen::Vector2d> {
  std::size_t operator()(const Eigen::Vector2d &v) const {
    std::size_t h1 = std::hash<double>()(v.x());
    std::size_t h2 = std::hash<double>()(v.y());
    return h1 ^ (h2 << 1);
  }
};

}  // namespace std