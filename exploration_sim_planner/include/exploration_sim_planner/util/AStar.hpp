/**
 * @file AStar.hpp
 * @brief Generic A* implementation so I don't have to implement it for a
 * fiftieth time
 */

#pragma once

#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

template <typename State, typename Cost>
class AStar {
 public:
  // using Node = std::pair<State, Cost>;

  using Node = struct {
    State state;
    Cost cost;  // depending on context, this represents the total f score or
                // the cost to travel to neighbor
  };

  using GetNeighbors = std::function<std::vector<Node>(const State&)>;
  using Heuristic = std::function<Cost(const State&, const State&)>;
  using NodeComparator = std::function<bool(const Node&, const Node&)>;

  std::optional<std::pair<std::vector<State>, Cost>> find_path(
      const State& start, const State& goal, GetNeighbors get_neighbors,
      Heuristic heuristic, NodeComparator node_comparator) {
    std::priority_queue<Node, std::vector<Node>, NodeComparator> open_set(
        node_comparator);

    std::unordered_map<State, State> came_from;
    std::unordered_map<State, Cost> g_score;

    g_score[start] = 0;

    open_set.emplace(start, 0);

    while (!open_set.empty()) {
      State current = open_set.top().state;

      if (current == goal) {
        return std::make_pair(reconstruct_path(came_from, current),
                              g_score[goal]);
      }

      open_set.pop();

      for (const auto& neighbor : get_neighbors(current)) {
        Cost tentative_g_score = g_score[current] + neighbor.cost;

        // If the node hasn't been visisted, or the tentative g score is lower
        // than previously seen
        if (!g_score.contains(neighbor.state) ||
            tentative_g_score < g_score[neighbor.state]) {
          // update the came from and g score
          came_from[neighbor.state] = current;
          g_score[neighbor.state] = tentative_g_score;

          // Compute the total heuristic cost and add to the open set
          Cost f_score = tentative_g_score + heuristic(neighbor.state, goal);
          open_set.emplace(neighbor.state, f_score);
        }
      }
    }

    // failed to find path
    return std::nullopt;
  }

 private:
  std::vector<State> reconstruct_path(
      const std::unordered_map<State, State>& came_from, const State& current) {
    std::vector<State> path;
    State node = current;

    while (came_from.find(node) != came_from.end()) {
      path.push_back(node);
      node = came_from.at(node);
    }

    std::reverse(path.begin(), path.end());

    return path;
  }
};