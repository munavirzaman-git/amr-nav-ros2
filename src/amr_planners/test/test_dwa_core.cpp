// Copyright 2026 Munavir Zaman — Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "amr_planners/dwa_core.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using amr_planners::DWACore;
using amr_planners::Pose2D;
using nav2_costmap_2d::Costmap2D;

namespace
{
// 4 m x 4 m local costmap, origin at (-2,-2), robot at the centre
Costmap2D makeMap() {return Costmap2D(80, 80, 0.05, -2.0, -2.0, nav2_costmap_2d::FREE_SPACE);}
std::vector<Pose2D> straightPath()
{
  std::vector<Pose2D> p;
  for (int i = 0; i <= 30; ++i) {p.push_back({0.05 * i, 0.0, 0.0});}
  return p;
}
}  // namespace

TEST(DWACore, DrivesStraightAlongPath)
{
  auto cm = makeMap();
  DWACore dwa;
  auto res = dwa.plan(cm, {0, 0, 0}, 0.4, 0.0, straightPath(), {0.8, 0.0, 0.0}, 0.5);
  ASSERT_TRUE(res.found);
  EXPECT_GT(res.best.v, 0.3);
  EXPECT_LT(std::fabs(res.best.w), 0.3);
}

TEST(DWACore, RespectsDynamicWindow)
{
  auto cm = makeMap();
  amr_planners::DWAParams p;
  p.acc_lim_x = 1.0;
  p.acc_dt = 0.1;
  DWACore dwa(p);
  auto res = dwa.plan(cm, {0, 0, 0}, 0.0, 0.0, straightPath(), {0.8, 0.0, 0.0}, 0.5);
  ASSERT_TRUE(res.found);
  EXPECT_LE(res.best.v, 0.1 + 1e-9);   // from rest, at most a*dt
}

TEST(DWACore, AvoidsObstacleOnPath)
{
  auto cm = makeMap();
  // block straight ahead (0.35..0.6 m) across y in [-0.15, 0.15]
  for (unsigned int x = 47; x <= 52; ++x) {
    for (unsigned int y = 37; y <= 43; ++y) {cm.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);}
  }
  DWACore dwa;
  auto res = dwa.plan(cm, {0, 0, 0}, 0.4, 0.0, straightPath(), {0.8, 0.0, 0.0}, 0.5);
  ASSERT_TRUE(res.found);
  for (const auto & q : res.best.poses) {
    unsigned int mx, my;
    ASSERT_TRUE(cm.worldToMap(q.x, q.y, mx, my));
    EXPECT_LT(cm.getCost(mx, my), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
  EXPECT_GT(res.rejected, 0);
}

TEST(DWACore, NoSolutionWhenBoxedIn)
{
  auto cm = makeMap();
  for (unsigned int x = 0; x < 80; ++x) {
    for (unsigned int y = 0; y < 80; ++y) {
      const double wx = -2.0 + 0.05 * (x + 0.5), wy = -2.0 + 0.05 * (y + 0.5);
      if (std::hypot(wx, wy) > 0.06) {cm.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);}
    }
  }
  amr_planners::DWAParams p;
  p.min_vel_x = 0.1;   // must move => every rollout leaves the free cell
  DWACore dwa(p);
  auto res = dwa.plan(cm, {0, 0, 0}, 0.3, 0.0, straightPath(), {0.8, 0.0, 0.0}, 0.5);
  EXPECT_FALSE(res.found);
}

TEST(DWACore, ArcIntegrationIsExact)
{
  auto cm = makeMap();
  amr_planners::DWAParams p;
  p.sim_time = M_PI;   // half circle with v=0.2, w=1 -> radius 0.2
  p.sim_dt = M_PI / 20;
  DWACore dwa(p);
  auto t = dwa.rollout(cm, {0, 0, 0}, 0.2, 1.0);
  ASSERT_TRUE(t.valid);
  EXPECT_NEAR(t.poses.back().x, 0.0, 1e-6);
  EXPECT_NEAR(t.poses.back().y, 0.4, 1e-6);
}
