// Copyright 2026 Munavir Zaman — Apache-2.0
#include "amr_planners/astar_planner.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_planners
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("AStarPlanner: unable to lock parent node");
  }
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string & key, const rclcpp::ParameterValue & def) {
      nav2_util::declare_parameter_if_not_declared(node, name_ + "." + key, def);
      return node->get_parameter(name_ + "." + key);
    };

  AStarParams ap;
  ap.cost_weight = declare("cost_weight", rclcpp::ParameterValue(3.0)).as_double();
  ap.heuristic_weight = declare("heuristic_weight", rclcpp::ParameterValue(1.0)).as_double();
  ap.unknown_cost_ratio = declare("unknown_cost_ratio", rclcpp::ParameterValue(0.5)).as_double();
  ap.allow_unknown = declare("allow_unknown", rclcpp::ParameterValue(true)).as_bool();
  ap.lethal_threshold = static_cast<unsigned char>(
    declare("lethal_threshold", rclcpp::ParameterValue(253)).as_int());
  ap.max_iterations = static_cast<int>(
    declare("max_iterations", rclcpp::ParameterValue(2000000)).as_int());
  astar_.setParams(ap);

  tolerance_ = declare("tolerance", rclcpp::ParameterValue(0.3)).as_double();
  max_planning_time_ = declare("max_planning_time", rclcpp::ParameterValue(2.0)).as_double();
  smooth_path_ = declare("smooth_path", rclcpp::ParameterValue(true)).as_bool();
  smoother_params_.w_data = declare("smoother.w_data", rclcpp::ParameterValue(0.2)).as_double();
  smoother_params_.w_smooth = declare("smoother.w_smooth", rclcpp::ParameterValue(0.3)).as_double();
  smoother_params_.iterations = static_cast<int>(
    declare("smoother.iterations", rclcpp::ParameterValue(200)).as_int());
  smoother_params_.cost_tolerance = static_cast<int>(
    declare("smoother.cost_tolerance", rclcpp::ParameterValue(128)).as_int());
  smoother_params_.lethal_threshold = ap.lethal_threshold;

  RCLCPP_INFO(
    logger_, "AStarPlanner '%s' configured (cost_weight=%.2f, heuristic_weight=%.2f, "
    "allow_unknown=%s, smoothing=%s)", name_.c_str(), ap.cost_weight, ap.heuristic_weight,
    ap.allow_unknown ? "true" : "false", smooth_path_ ? "on" : "off");
}

void AStarPlanner::cleanup() {RCLCPP_INFO(logger_, "AStarPlanner '%s' cleanup", name_.c_str());}
void AStarPlanner::activate() {RCLCPP_INFO(logger_, "AStarPlanner '%s' active", name_.c_str());}
void AStarPlanner::deactivate() {RCLCPP_INFO(logger_, "AStarPlanner '%s' inactive", name_.c_str());}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  const auto t0 = std::chrono::steady_clock::now();
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  unsigned int sx, sy, gx, gy;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
    throw nav2_core::StartOutsideMapBounds("AStarPlanner: start is outside the costmap");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    throw nav2_core::GoalOutsideMapBounds("AStarPlanner: goal is outside the costmap");
  }
  // The start cell is allowed inside the inflation (robot may be close to a wall),
  // but not inside an actual obstacle.
  if (costmap_->getCost(sx, sy) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::StartOccupied("AStarPlanner: start is inside an obstacle");
  }

  GridAStar::Cell goal_cell{gx, gy};
  const bool goal_moved = !astar_.isTraversable(*costmap_, gx, gy);
  if (goal_moved && !astar_.findNearestTraversable(*costmap_, {gx, gy}, tolerance_, goal_cell)) {
    throw nav2_core::GoalOccupied("AStarPlanner: goal is occupied and no free cell within tolerance");
  }

  if (sx == goal_cell.first && sy == goal_cell.second) {
    geometry_msgs::msg::PoseStamped p = goal;
    p.header = path.header;
    path.poses.push_back(p);
    return path;
  }

  bool timed_out = false;
  bool cancelled = false;
  auto should_abort = [&]() {
      if (cancel_checker && cancel_checker()) {
        cancelled = true;
        return true;
      }
      const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (max_planning_time_ > 0.0 && dt > max_planning_time_) {
        timed_out = true;
        return true;
      }
      return false;
    };

  AStarStats stats;
  const auto cells = astar_.search(*costmap_, {sx, sy}, goal_cell, &stats, should_abort);
  if (cells.empty()) {
    if (cancelled) {
      throw nav2_core::PlannerException("AStarPlanner: planning cancelled");
    }
    if (timed_out) {
      throw nav2_core::PlannerTimedOut("AStarPlanner: exceeded max_planning_time");
    }
    throw nav2_core::NoValidPathCouldBeFound("AStarPlanner: no path found");
  }

  // cells -> world, with exact start / goal positions at the ends
  std::vector<Point2D> pts;
  pts.reserve(cells.size());
  for (const auto & c : cells) {
    Point2D p;
    costmap_->mapToWorld(c.first, c.second, p.x, p.y);
    pts.push_back(p);
  }
  pts.front() = {start.pose.position.x, start.pose.position.y};
  if (!goal_moved) {
    pts.back() = {goal.pose.position.x, goal.pose.position.y};
  }
  if (smooth_path_) {
    pts = smoothPath(*costmap_, pts, smoother_params_);
  }

  path.poses.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = pts[i].x;
    ps.pose.position.y = pts[i].y;
    if (i + 1 < pts.size()) {
      const double yaw = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      ps.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(ps);
  }

  const double ms = 1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  RCLCPP_DEBUG(
    logger_, "A*: %zu poses, %.2f m, %d expansions, %.1f ms", path.poses.size(),
    pathLength(pts), stats.expansions, ms);
  return path;
}

}  // namespace amr_planners

PLUGINLIB_EXPORT_CLASS(amr_planners::AStarPlanner, nav2_core::GlobalPlanner)
