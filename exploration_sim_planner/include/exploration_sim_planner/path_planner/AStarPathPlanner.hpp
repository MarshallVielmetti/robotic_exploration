#pragma once

#include "exploration_sim_planner/path_planner/AbstractPathPlanner.hpp"
#include "exploration_sim_planner/util/OgmView.hpp"
#include <Eigen/Dense>

#include <memory>
#include <vector>

class AStarPathPlanner : public AbstractPathPlanner {
public:
  AStarPathPlanner() = default;
  virtual ~AStarPathPlanner() = default;

  nav_msgs::msg::Path
  operator()(const geometry_msgs::msg::PoseStamped::SharedPtr goal,
             const nav_msgs::msg::OccupancyGrid::SharedPtr map,
             const geometry_msgs::msg::PoseStamped::SharedPtr pose) override;

private:
  class Node {
  public:
    Node(OgmView *ogm, Eigen::Vector2i cell, double g, double h, Node *parent)
        : g{g}, h{h}, parent{parent}, ogm{ogm} {}

    double f() const { return g + h; }
    std::vector<std::unique_ptr<Node>> neighbors() const {
      std::vector<std::unique_ptr<Node>> neighbors;

      // loop through all the neighbors of the current cell
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {

          if (dx == 0 && dy == 0) {
            continue;
          }

          Eigen::Vector2i new_cell = cell + Eigen::Vector2i{dx, dy};
          int8_t occupancy; // from ogm message definition, 1 is occupied, 0 is
                            // unoccupied, -1 is uknown

          try {
            occupancy = ogm->get(new_cell.x(), new_cell.y());
          } catch (const std::out_of_range &e) {
            occupancy = -1;
          }

          // only add the cell if it is definitely unoccupied.
          if (occupancy != 0) {
            continue;
          }

          // add the valid neighbor cell to the list of neighbors
          neighbors.emplace_back(ogm, new_cell, g + 1, 0, this);
        }
      }

      return neighbors;
    }

    double g;
    double h;
    Eigen::Vector2i cell;
    Node *parent;

    bool operator<(const Node &other) const { return f() > other.f(); }

  private:
    OgmView *ogm;
  };

  class NodeProvider {
  public:
    NodeProvider(OgmView *ogm, Eigen::Vector2i goal_cell)
        : ogm_(ogm), goal_cell_(goal_cell) {

      nodes_ = std::vector<std::vector<Node>>(
          ogm_->width(), std::vector<Node>(ogm_->height()));
    }

    std::unique_ptr<Node> get_node(Eigen::Vector2i cell, double g,
                                   Node *parent) {
      return std::make_unique<Node>(ogm_, cell, g, h, parent);
    }

  private:
    OgmView *ogm_;
    Eigen::Vector2i goal_cell_;

    std::vector<std::vector<Node>> nodes_;
  };

  // compare two node* for a priority queue using builtin comparison
  struct NodePtrCompare {
    bool operator()(const Node *lhs, const Node *rhs) const {
      return *lhs < *rhs;
    }
  }

  std::vector<Eigen::Vector2i>
  exec_astar(Eigen::Vector2i start_cell, Eigen::Vector2i goal_cell);

  nav_msgs::msg::Path
  cell_to_world_path(const std::vector<Eigen::Vector2i> &cell_path);

private:
  std::shared_ptr<OgmView> ogm_;
};