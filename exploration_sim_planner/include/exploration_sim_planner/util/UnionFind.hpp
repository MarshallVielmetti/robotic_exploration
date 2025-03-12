/**
 * @file UnionFind.hpp
 *
 * @brief A class to represent the UnionFind algorithm
 *
 * This is a header only class that implements the UnionFind algorithm
 * this is used by the ConnectedComponentsLabeling class to find the different
 * connected components inside each sector
 */

#pragma once

#include <cstdint>
#include <vector>

class UnionFind {
  std::vector<uint32_t> parent;
  uint32_t num_sets;

 public:
  UnionFind(uint32_t n) : parent(n) {
    // Initialize each parent to be itself
    for (uint32_t i = 0; i < n; i++) {
      parent[i] = i;
    }

    num_sets = n;
  }

  // Recursively find the representative of the set that contains i
  // Flatten the tree by setting the parent of each node to the root node
  uint32_t find(uint32_t i) {
    if (parent[i] == i) {
      return i;
    }

    parent[i] = find(parent[i]);
    return parent[i];
  }

  void unite(int i, int j) {
    uint32_t i_rep = find(i);
    uint32_t j_rep = find(j);

    if (i_rep == j_rep) {
      return;
    }

    num_sets--;

    parent[i_rep] = j_rep;
  }

  uint32_t get_num_sets() const { return num_sets; }
};