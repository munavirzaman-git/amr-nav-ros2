// Copyright 2026 Munavir Zaman — Apache-2.0
#include "amr_planners/astar_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

#include "nav2_costmap_2d/cost_values.hpp"

namespace amr_planners
{
namespace
{
constexpr double kSqrt2 = 1.41421356237309504880;

struct OpenEntry
{
  double f;
  double g;
  std::size_t index;
  // min-heap on f; ties broken towards larger g (deeper nodes) to reduce expansions
  bool operator>(const OpenEntry & o) const
  {
    return f > o.f || (f == o.f && g < o.g);
  }
};
}  // namespace

bool GridAStar::isTraversable(
  const nav2_costmap_2d::Costmap2D & cm, unsigned int mx, unsigned int my) const
{
  const unsigned char c = cm.getCost(mx, my);
  if (c == nav2_costmap_2d::NO_INFORMATION) {
    return params_.allow_unknown;
  }
  return c < params_.lethal_threshold;
}

double GridAStar::cellPenalty(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return 1.0 + params_.cost_weight * params_.unknown_cost_ratio;
  }
  const double c = std::min(static_cast<double>(cost), 252.0);
  return 1.0 + params_.cost_weight * (c / 252.0);
}

std::vector<GridAStar::Cell> GridAStar::search(
  const nav2_costmap_2d::Costmap2D & cm, const Cell & start, const Cell & goal,
  AStarStats * stats, const std::function<bool()> & should_abort) const
{
  std::vector<Cell> path;
  AStarStats local_stats;
  AStarStats & st = stats ? *stats : local_stats;
  st = AStarStats{};

  const unsigned int nx = cm.getSizeInCellsX();
  const unsigned int ny = cm.getSizeInCellsY();
  if (start.first >= nx || start.second >= ny || goal.first >= nx || goal.second >= ny) {
    return path;
  }

  const std::size_t n = static_cast<std::size_t>(nx) * ny;
  auto to_index = [nx](unsigned int x, unsigned int y) {
      return static_cast<std::size_t>(y) * nx + x;
    };
  const std::size_t s_idx = to_index(start.first, start.second);
  const std::size_t g_idx = to_index(goal.first, goal.second);
  constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

  std::vector<double> g(n, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(n, kNone);
  std::vector<std::uint8_t> closed(n, 0);

  // Octile distance (in cells) is admissible because every step penalty is >= 1.
  auto heuristic = [&](unsigned int x, unsigned int y) {
      const double dx = std::fabs(static_cast<double>(x) - goal.first);
      const double dy = std::fabs(static_cast<double>(y) - goal.second);
      return params_.heuristic_weight * ((dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy));
    };

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;
  g[s_idx] = 0.0;
  open.push({heuristic(start.first, start.second), 0.0, s_idx});

  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    const OpenEntry cur = open.top();
    open.pop();
    if (closed[cur.index]) {
      continue;
    }
    closed[cur.index] = 1;
    ++st.expansions;

    if (cur.index == g_idx) {
      found = true;
      break;
    }
    if (params_.max_iterations > 0 && st.expansions >= params_.max_iterations) {
      break;
    }
    if (should_abort && (st.expansions & 1023) == 0 && should_abort()) {
      st.aborted = true;
      break;
    }

    const unsigned int cx = static_cast<unsigned int>(cur.index % nx);
    const unsigned int cy = static_cast<unsigned int>(cur.index / nx);

    for (int k = 0; k < 8; ++k) {
      const long ax = static_cast<long>(cx) + kDx[k];  // NOLINT
      const long ay = static_cast<long>(cy) + kDy[k];  // NOLINT
      if (ax < 0 || ay < 0 || ax >= static_cast<long>(nx) || ay >= static_cast<long>(ny)) {  // NOLINT
        continue;
      }
      const unsigned int ux = static_cast<unsigned int>(ax);
      const unsigned int uy = static_cast<unsigned int>(ay);
      const std::size_t ni = to_index(ux, uy);
      if (closed[ni] || !isTraversable(cm, ux, uy)) {
        continue;
      }
      const bool diagonal = (kDx[k] != 0 && kDy[k] != 0);
      if (diagonal &&
        (!isTraversable(cm, ux, cy) || !isTraversable(cm, cx, uy)))
      {
        continue;  // no corner cutting
      }
      const double step = diagonal ? kSqrt2 : 1.0;
      const double ng = g[cur.index] + step * cellPenalty(cm.getCost(ux, uy));
      if (ng < g[ni]) {
        g[ni] = ng;
        parent[ni] = cur.index;
        open.push({ng + heuristic(ux, uy), ng, ni});
      }
    }
  }

  st.success = found;
  if (!found) {
    return path;
  }
  for (std::size_t i = g_idx; ; i = parent[i]) {
    path.emplace_back(static_cast<unsigned int>(i % nx), static_cast<unsigned int>(i / nx));
    if (i == s_idx || parent[i] == kNone) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool GridAStar::findNearestTraversable(
  const nav2_costmap_2d::Costmap2D & cm, const Cell & goal, double tolerance_m, Cell & out) const
{
  if (goal.first < cm.getSizeInCellsX() && goal.second < cm.getSizeInCellsY() &&
    isTraversable(cm, goal.first, goal.second))
  {
    out = goal;
    return true;
  }
  const int r = static_cast<int>(std::ceil(tolerance_m / cm.getResolution()));
  const long nx = cm.getSizeInCellsX();  // NOLINT
  const long ny = cm.getSizeInCellsY();  // NOLINT
  long best = std::numeric_limits<long>::max();  // NOLINT
  bool ok = false;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const long d2 = static_cast<long>(dx) * dx + static_cast<long>(dy) * dy;  // NOLINT
      if (d2 > static_cast<long>(r) * r || d2 >= best) {  // NOLINT
        continue;
      }
      const long x = static_cast<long>(goal.first) + dx;  // NOLINT
      const long y = static_cast<long>(goal.second) + dy;  // NOLINT
      if (x < 0 || y < 0 || x >= nx || y >= ny) {
        continue;
      }
      if (isTraversable(cm, static_cast<unsigned int>(x), static_cast<unsigned int>(y))) {
        best = d2;
        out = {static_cast<unsigned int>(x), static_cast<unsigned int>(y)};
        ok = true;
      }
    }
  }
  return ok;
}

std::vector<Point2D> smoothPath(
  const nav2_costmap_2d::Costmap2D & cm, const std::vector<Point2D> & path,
  const SmootherParams & p)
{
  if (path.size() < 3 || p.iterations <= 0) {
    return path;
  }
  auto cost_at = [&cm](const Point2D & q, unsigned char & c) {
      unsigned int mx, my;
      if (!cm.worldToMap(q.x, q.y, mx, my)) {
        return false;
      }
      c = cm.getCost(mx, my);
      return true;
    };

  std::vector<unsigned char> original_cost(path.size(), 0);
  for (std::size_t i = 0; i < path.size(); ++i) {
    unsigned char c = 0;
    original_cost[i] = cost_at(path[i], c) ? c : 0;
  }

  std::vector<Point2D> out = path;
  for (int it = 0; it < p.iterations; ++it) {
    double change = 0.0;
    for (std::size_t i = 1; i + 1 < out.size(); ++i) {
      const Point2D prev = out[i];
      out[i].x += p.w_data * (path[i].x - out[i].x) +
        p.w_smooth * (out[i - 1].x + out[i + 1].x - 2.0 * out[i].x);
      out[i].y += p.w_data * (path[i].y - out[i].y) +
        p.w_smooth * (out[i - 1].y + out[i + 1].y - 2.0 * out[i].y);

      unsigned char c = 0;
      const int allowed = std::max(static_cast<int>(original_cost[i]), p.cost_tolerance);
      if (!cost_at(out[i], c) || c == nav2_costmap_2d::NO_INFORMATION ||
        c >= p.lethal_threshold || static_cast<int>(c) > allowed)
      {
        out[i] = prev;  // reject the move
        continue;
      }
      change += std::fabs(out[i].x - prev.x) + std::fabs(out[i].y - prev.y);
    }
    if (change < p.tolerance) {
      break;
    }
  }
  return out;
}

double pathLength(const std::vector<Point2D> & path)
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

}  // namespace amr_planners
