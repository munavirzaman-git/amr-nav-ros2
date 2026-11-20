// Copyright 2026 Munavir Zaman — Apache-2.0
// Dynamic Window Approach (Fox, Burgard & Thrun 1997) for a differential-drive robot.
// ROS-free core: sample (v, w) inside the dynamic window, forward-simulate each pair,
// reject colliding rollouts on the local costmap, and score the rest.
#ifndef AMR_PLANNERS__DWA_CORE_HPP_
#define AMR_PLANNERS__DWA_CORE_HPP_

#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace amr_planners
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct DWAParams
{
  // kinematic limits
  double min_vel_x{0.0};
  double max_vel_x{0.5};
  double max_vel_theta{1.8};
  double acc_lim_x{2.0};
  double acc_lim_theta{3.0};
  double acc_dt{0.2};          // window half-width = acc * acc_dt
  // sampling / simulation
  int vx_samples{15};
  int vtheta_samples{31};
  double sim_time{1.5};
  double sim_dt{0.1};
  // objective weights (criteria are normalised to [0,1] across valid samples)
  double w_path{1.0};          // end point distance to the reference path
  double w_goal{1.2};          // end point distance to the lookahead target
  double w_heading{0.6};       // heading error towards the lookahead target
  double w_obstacle{1.0};      // max costmap cost along the rollout
  double w_speed{0.5};         // prefer faster forward motion
  // collision checking
  bool allow_unknown{false};
  unsigned char lethal_threshold{253};
};

struct Trajectory
{
  double v{0.0};
  double w{0.0};
  std::vector<Pose2D> poses;
  bool valid{false};
  double max_cost{0.0};
  // raw criteria
  double path_cost{0.0};
  double goal_cost{0.0};
  double heading_cost{0.0};
  double obstacle_cost{0.0};
  double speed_cost{0.0};
  double total{0.0};
};

struct DWAResult
{
  bool found{false};
  Trajectory best;
  std::vector<Trajectory> candidates;   // valid rollouts (for visualisation)
  int rejected{0};
};

class DWACore
{
public:
  explicit DWACore(const DWAParams & params = DWAParams{})
  : p_(params) {}

  void setParams(const DWAParams & params) {p_ = params;}
  const DWAParams & params() const {return p_;}

  Trajectory rollout(
    const nav2_costmap_2d::Costmap2D & cm, const Pose2D & start, double v, double w) const;

  /// robot pose, measured velocity (v0, w0), reference path and target in costmap frame.
  /// max_vel_x lets the caller apply speed limits / goal approach slow-down.
  DWAResult plan(
    const nav2_costmap_2d::Costmap2D & cm, const Pose2D & robot, double v0, double w0,
    const std::vector<Pose2D> & reference_path, const Pose2D & target,
    double max_vel_x) const;

  static double normalizeAngle(double a);

private:
  DWAParams p_;
};

}  // namespace amr_planners

#endif  // AMR_PLANNERS__DWA_CORE_HPP_
