// Copyright 2026 Munavir Zaman — Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "amr_planners/astar_core.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using amr_planners::GridAStar;
using nav2_costmap_2d::Costmap2D;

namespace
{
Costmap2D makeMap(unsigned int n = 50)
{
  return Costmap2D(n, n, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
}
void wall(Costmap2D & cm, unsigned int x, unsigned int y0, unsigned int y1)
{
  for (unsigned int y = y0; y <= y1; ++y) {cm.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);}
}
}  // namespace

TEST(AStarCore, StraightLineInFreeSpace)
{
  auto cm = makeMap();
  GridAStar a;
  auto path = a.search(cm, {5, 10}, {40, 10});
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front(), GridAStar::Cell(5, 10));
  EXPECT_EQ(path.back(), GridAStar::Cell(40, 10));
  EXPECT_EQ(path.size(), 36u);   // optimal: 35 straight steps
}

TEST(AStarCore, GoesAroundWall)
{
  auto cm = makeMap();
  wall(cm, 25, 0, 40);   // gap at the top (y 41..49)
  GridAStar a;
  amr_planners::AStarStats st;
  auto path = a.search(cm, {10, 10}, {40, 10}, &st);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(st.success);
  for (const auto & c : path) {
    EXPECT_LT(cm.getCost(c.first, c.second), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
}

TEST(AStarCore, NoPathWhenEnclosed)
{
  auto cm = makeMap();
  wall(cm, 25, 0, 49);
  GridAStar a;
  EXPECT_TRUE(a.search(cm, {10, 10}, {40, 10}).empty());
}

TEST(AStarCore, NoCornerCutting)
{
  auto cm = makeMap(10);
  cm.setCost(5, 4, nav2_costmap_2d::LETHAL_OBSTACLE);
  cm.setCost(4, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  GridAStar a;
  auto path = a.search(cm, {4, 4}, {5, 5});
  ASSERT_FALSE(path.empty());
  EXPECT_GT(path.size(), 2u);   // diagonal squeeze between the two obstacles is forbidden
}

TEST(AStarCore, CostAwarenessAvoidsHighCostCorridor)
{
  auto cm = makeMap();
  for (unsigned int x = 0; x < 50; ++x) {cm.setCost(x, 10, 200);}   // expensive straight row
  amr_planners::AStarParams p;
  p.cost_weight = 5.0;
  GridAStar a(p);
  auto path = a.search(cm, {5, 10}, {45, 10});
  ASSERT_FALSE(path.empty());
  int on_expensive_row = 0;
  for (const auto & c : path) {on_expensive_row += (c.second == 10);}
  EXPECT_LT(on_expensive_row, 10);
}

TEST(AStarCore, NearestTraversableGoal)
{
  auto cm = makeMap();
  cm.setCost(20, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  GridAStar a;
  GridAStar::Cell out;
  ASSERT_TRUE(a.findNearestTraversable(cm, {20, 20}, 0.3, out));
  EXPECT_NE(out, GridAStar::Cell(20, 20));
  EXPECT_LE(std::abs(static_cast<int>(out.first) - 20) + std::abs(static_cast<int>(out.second) - 20), 2);
}

TEST(AStarCore, SmootherKeepsEndpointsAndShortensZigZag)
{
  auto cm = makeMap();
  std::vector<amr_planners::Point2D> zig;
  for (int i = 0; i < 20; ++i) {zig.push_back({0.5 + 0.1 * i, (i % 2) ? 1.1 : 1.0});}
  amr_planners::SmootherParams sp;
  auto s = amr_planners::smoothPath(cm, zig, sp);
  ASSERT_EQ(s.size(), zig.size());
  EXPECT_DOUBLE_EQ(s.front().x, zig.front().x);
  EXPECT_DOUBLE_EQ(s.back().y, zig.back().y);
  EXPECT_LT(amr_planners::pathLength(s), amr_planners::pathLength(zig));
}
