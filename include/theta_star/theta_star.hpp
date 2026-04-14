#ifndef THETA_STAR_THETA_STAR_HPP_
#define THETA_STAR_THETA_STAR_HPP_

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <functional>
#include "theta_star/grid_map.hpp"

const double INF_COST = DBL_MAX;
const int UNKNOWN_COST = 255;
const int OCCUPIED_COST = 254;
const int MAX_NON_OBSTACLE_COST = 252;

struct coordsM
{
  int x, y;
};

struct coordsW
{
  double x, y;
};

struct tree_node
{
  int x, y;
  double g = INF_COST;
  double h = INF_COST;
  const tree_node * parent_id = nullptr;
  bool is_in_queue = false;
  double f = INF_COST;
};

struct comp
{
  bool operator()(const tree_node * p1, const tree_node * p2)
  {
    return (p1->f) > (p2->f);
  }
};

namespace theta_star
{

struct PlannerConfig
{
  double w_traversal_cost = 2.0;
  double w_euc_cost = 1.0;
  int how_many_corners = 8;
  bool allow_unknown = true;
  int terminal_checking_interval = 5000;
};

class ThetaStar
{
public:
  coordsM src_{}, dst_{};
  GridMap * costmap_{};

  double w_traversal_cost_;
  double w_euc_cost_;
  double w_heuristic_cost_;
  int how_many_corners_;
  bool allow_unknown_;
  int size_x_, size_y_;
  int terminal_checking_interval_;

  int nodes_opened = 0;

  ThetaStar();

  ~ThetaStar();

  void setCostmap(GridMap * costmap);

  void setStartAndGoal(int start_x, int start_y, int goal_x, int goal_y);

  void setStartAndGoalWorld(double start_wx, double start_wy,
                             double goal_wx, double goal_wy);

  bool generatePath(std::vector<coordsW> & raw_path,
                    std::function<bool()> cancel_checker = []() { return false; });

  inline bool isSafe(const int & cx, const int & cy) const
  {
    return (costmap_->getCost(cx, cy) == NO_INFORMATION && allow_unknown_) ||
           costmap_->getCost(cx, cy) <= MAX_NON_OBSTACLE_COST;
  }

  bool isUnsafeToPlan() const
  {
    return !(isSafe(src_.x, src_.y)) || !(isSafe(dst_.x, dst_.y));
  }

  void clearStart();

  void setConfig(const PlannerConfig & config);

protected:
  std::vector<tree_node *> node_position_;
  std::vector<tree_node> nodes_data_;
  std::priority_queue<tree_node *, std::vector<tree_node *>, comp> queue_;
  int index_generated_;

  const coordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  tree_node * exp_node;

  void resetParent(tree_node * curr_data);

  void setNeighbors(const tree_node * curr_data);

  bool losCheck(
    const int & x0, const int & y0, const int & x1, const int & y1,
    double & sl_cost) const;

  void backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const;

  bool isSafe(const int & cx, const int & cy, double & cost) const
  {
    double curr_cost = getCost(cx, cy);
    if ((costmap_->getCost(cx, cy) == NO_INFORMATION && allow_unknown_) ||
      curr_cost <= MAX_NON_OBSTACLE_COST)
    {
      if (costmap_->getCost(cx, cy) == NO_INFORMATION) {
        curr_cost = OCCUPIED_COST - 1;
      }
      cost += w_traversal_cost_ * curr_cost * curr_cost / MAX_NON_OBSTACLE_COST /
        MAX_NON_OBSTACLE_COST;
      return true;
    } else {
      return false;
    }
  }

  inline double getCost(const int & cx, const int & cy) const
  {
    return 26 + 0.9 * costmap_->getCost(cx, cy);
  }

  inline double getTraversalCost(const int & cx, const int & cy)
  {
    double curr_cost = getCost(cx, cy);
    return w_traversal_cost_ * curr_cost * curr_cost / MAX_NON_OBSTACLE_COST /
           MAX_NON_OBSTACLE_COST;
  }

  inline double getEuclideanCost(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return w_euc_cost_ * std::hypot(ax - bx, ay - by);
  }

  inline double getHCost(const int & cx, const int & cy)
  {
    return w_heuristic_cost_ * std::hypot(cx - dst_.x, cy - dst_.y);
  }

  inline bool withinLimits(const int & cx, const int & cy) const
  {
    return cx >= 0 && cx < size_x_ && cy >= 0 && cy < size_y_;
  }

  inline bool isGoal(const tree_node & this_node) const
  {
    return this_node.x == dst_.x && this_node.y == dst_.y;
  }

  void initializePosn(int size_inc = 0);

  inline void addIndex(const int & cx, const int & cy, tree_node * node_this)
  {
    node_position_[size_x_ * cy + cx] = node_this;
  }

  inline tree_node * getIndex(const int & cx, const int & cy)
  {
    return node_position_[size_x_ * cy + cx];
  }

  void addToNodesData(const int & id_this)
  {
    if (static_cast<int>(nodes_data_.size()) <= id_this) {
      nodes_data_.push_back({});
    } else {
      nodes_data_[id_this] = {};
    }
  }

  void resetContainers();

  void clearQueue()
  {
    queue_ = std::priority_queue<tree_node *, std::vector<tree_node *>, comp>();
  }
};

}  // namespace theta_star

#endif  // THETA_STAR_THETA_STAR_HPP_
