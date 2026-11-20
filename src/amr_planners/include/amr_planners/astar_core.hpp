// Copyright 2026 Munavir Zaman — Apache-2.0
// Grid A* on a nav2 Costmap2D. ROS-free so it can be unit tested in isolation.
#ifndef AMR_PLANNERS__ASTAR_CORE_HPP_
#define AMR_PLANNERS__ASTAR_CORE_HPP_

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace amr_planners
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct AStarParams
{
  double cost_weight{3.0};          // step cost = length * (1 + cost_weight * cost/252)
  double heuristic_weight{1.0};     // >1 => weighted A* (faster, no longer optimal)
  double unknown_cost_ratio{0.5};   // penalty used for NO_INFORMATION cells when allowed
  bool allow_unknown{true};
  unsigned char lethal_threshold{253};  // cost >= this is not traversable (inscribed inflated)
  int max_iterations{2000000};
};

struct AStarStats
{
  int expansions{0};
  bool success{false};
  bool aborted{false};
};

struct SmootherParams
{
  double w_data{0.2};
  double w_smooth{0.3};
  int iterations{200};
  double tolerance{1e-6};
  int cost_tolerance{128};          // a smoothed point may move into cells up to this cost
  unsigned char lethal_threshold{253};
};

class GridAStar
{
public:
  using Cell = std::pair<unsigned int, unsigned int>;

  explicit GridAStar(const AStarParams & params = AStarParams{})
  : params_(params) {}

  void setParams(const AStarParams & params) {params_ = params;}
  const AStarParams & params() const {return params_;}

  bool isTraversable(const nav2_costmap_2d::Costmap2D & cm, unsigned int mx, unsigned int my) const;

  /// 8-connected, cost-aware A*. Returns the cell path start..goal, or empty on failure.
  /// should_abort is polled every 1024 expansions (cancel / timeout).
  std::vector<Cell> search(
    const nav2_costmap_2d::Costmap2D & cm, const Cell & start, const Cell & goal,
    AStarStats * stats = nullptr,
    const std::function<bool()> & should_abort = nullptr) const;

  /// Nearest traversable cell to `goal` within `tolerance_m` (Euclidean). False if none.
  bool findNearestTraversable(
    const nav2_costmap_2d::Costmap2D & cm, const Cell & goal,
    double tolerance_m, Cell & out) const;

private:
  double cellPenalty(unsigned char cost) const;
  AStarParams params_;
};

/// Gradient-descent path smoother (data term + smoothness term) that refuses to
/// move a point into a higher-cost / lethal cell. End points are fixed.
std::vector<Point2D> smoothPath(
  const nav2_costmap_2d::Costmap2D & cm, const std::vector<Point2D> & path,
  const SmootherParams & params);

double pathLength(const std::vector<Point2D> & path);

}  // namespace amr_planners

#endif  // AMR_PLANNERS__ASTAR_CORE_HPP_
