// Copyright 2026 Munavir Zaman — Apache-2.0
#include "amr_planners/dwa_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace amr_planners
{

double DWACore::normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

Trajectory DWACore::rollout(
  const nav2_costmap_2d::Costmap2D & cm, const Pose2D & start, double v, double w) const
{
  Trajectory t;
  t.v = v;
  t.w = w;
  t.valid = true;

  const double dt = std::max(1e-3, p_.sim_dt);
  const int steps = std::max(1, static_cast<int>(std::ceil(p_.sim_time / dt)));
  t.poses.reserve(steps + 1);

  Pose2D s = start;
  t.poses.push_back(s);
  for (int k = 0; k < steps; ++k) {
    // exact unicycle integration over dt (arc), straight line when w ~ 0
    if (std::fabs(w) > 1e-6) {
      const double th1 = s.theta + w * dt;
      s.x += (v / w) * (std::sin(th1) - std::sin(s.theta));
      s.y += (v / w) * (-std::cos(th1) + std::cos(s.theta));
      s.theta = normalizeAngle(th1);
    } else {
      s.x += v * std::cos(s.theta) * dt;
      s.y += v * std::sin(s.theta) * dt;
    }

    unsigned int mx, my;
    if (!cm.worldToMap(s.x, s.y, mx, my)) {
      if (!p_.allow_unknown) {
        t.valid = false;
        break;
      }
      t.poses.push_back(s);
      continue;
    }
    const unsigned char c = cm.getCost(mx, my);
    if (c == nav2_costmap_2d::NO_INFORMATION) {
      if (!p_.allow_unknown) {
        t.valid = false;
        break;
      }
    } else if (c >= p_.lethal_threshold) {
      t.valid = false;   // centre inside inscribed radius => footprint collides
      break;
    } else {
      t.max_cost = std::max(t.max_cost, static_cast<double>(c));
    }
    t.poses.push_back(s);
  }
  return t;
}

DWAResult DWACore::plan(
  const nav2_costmap_2d::Costmap2D & cm, const Pose2D & robot, double v0, double w0,
  const std::vector<Pose2D> & reference_path, const Pose2D & target, double max_vel_x) const
{
  DWAResult res;

  // ---- dynamic window: velocities reachable within acc_dt and admissible by limits
  const double vmax = std::max(p_.min_vel_x, std::min(max_vel_x, p_.max_vel_x));
  double v_hi = std::min(vmax, v0 + p_.acc_lim_x * p_.acc_dt);
  double v_lo = std::max(p_.min_vel_x, v0 - p_.acc_lim_x * p_.acc_dt);
  v_hi = std::clamp(v_hi, p_.min_vel_x, vmax);
  v_lo = std::clamp(v_lo, p_.min_vel_x, v_hi);

  double w_hi = std::min(p_.max_vel_theta, w0 + p_.acc_lim_theta * p_.acc_dt);
  double w_lo = std::max(-p_.max_vel_theta, w0 - p_.acc_lim_theta * p_.acc_dt);
  w_hi = std::clamp(w_hi, -p_.max_vel_theta, p_.max_vel_theta);
  w_lo = std::clamp(w_lo, -p_.max_vel_theta, w_hi);

  const int nv = std::max(1, p_.vx_samples);
  const int nw = std::max(1, p_.vtheta_samples);
  res.candidates.reserve(static_cast<std::size_t>(nv) * nw);

  for (int i = 0; i < nv; ++i) {
    const double v = (nv == 1) ? v_hi : v_lo + (v_hi - v_lo) * i / (nv - 1);
    for (int j = 0; j < nw; ++j) {
      const double w = (nw == 1) ? 0.5 * (w_lo + w_hi) : w_lo + (w_hi - w_lo) * j / (nw - 1);
      Trajectory t = rollout(cm, robot, v, w);
      if (!t.valid) {
        ++res.rejected;
        continue;
      }
      const Pose2D & e = t.poses.back();

      double dmin = 0.0;
      if (!reference_path.empty()) {
        dmin = std::numeric_limits<double>::infinity();
        for (const auto & q : reference_path) {
          dmin = std::min(dmin, std::hypot(q.x - e.x, q.y - e.y));
        }
      }
      t.path_cost = dmin;
      t.goal_cost = std::hypot(target.x - e.x, target.y - e.y);
      t.heading_cost = std::fabs(normalizeAngle(std::atan2(target.y - e.y, target.x - e.x) - e.theta));
      t.obstacle_cost = t.max_cost / 252.0;
      t.speed_cost = p_.max_vel_x > 0.0 ? (p_.max_vel_x - v) / p_.max_vel_x : 0.0;
      res.candidates.push_back(std::move(t));
    }
  }

  if (res.candidates.empty()) {
    return res;
  }

  // ---- normalise each criterion over the valid set (classic DWA)
  double m_path = 0, m_goal = 0, m_head = 0, m_obs = 0, m_speed = 0;
  for (const auto & t : res.candidates) {
    m_path = std::max(m_path, t.path_cost);
    m_goal = std::max(m_goal, t.goal_cost);
    m_head = std::max(m_head, t.heading_cost);
    m_obs = std::max(m_obs, t.obstacle_cost);
    m_speed = std::max(m_speed, t.speed_cost);
  }
  auto nrm = [](double x, double m) {return m > 1e-9 ? x / m : 0.0;};

  double best = std::numeric_limits<double>::infinity();
  std::size_t best_i = 0;
  for (std::size_t i = 0; i < res.candidates.size(); ++i) {
    auto & t = res.candidates[i];
    t.total = p_.w_path * nrm(t.path_cost, m_path) +
      p_.w_goal * nrm(t.goal_cost, m_goal) +
      p_.w_heading * nrm(t.heading_cost, m_head) +
      p_.w_obstacle * nrm(t.obstacle_cost, m_obs) +
      p_.w_speed * nrm(t.speed_cost, m_speed);
    if (t.total < best) {
      best = t.total;
      best_i = i;
    }
  }
  res.best = res.candidates[best_i];
  res.found = true;
  return res;
}

}  // namespace amr_planners
