// Copyright 2026 Munavir Zaman — Apache-2.0
#include "amr_planners/dwa_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_planners
{
namespace
{
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
geometry_msgs::msg::Quaternion quatOf(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}
}  // namespace

void DWAController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("DWAController: unable to lock parent node");
  }
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string & key, const rclcpp::ParameterValue & def) {
      nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + "." + key, def);
      return node->get_parameter(plugin_name_ + "." + key);
    };

  DWAParams p;
  p.min_vel_x = declare("min_vel_x", rclcpp::ParameterValue(0.0)).as_double();
  p.max_vel_x = declare("max_vel_x", rclcpp::ParameterValue(0.5)).as_double();
  p.max_vel_theta = declare("max_vel_theta", rclcpp::ParameterValue(1.8)).as_double();
  p.acc_lim_x = declare("acc_lim_x", rclcpp::ParameterValue(2.0)).as_double();
  p.acc_lim_theta = declare("acc_lim_theta", rclcpp::ParameterValue(3.0)).as_double();
  p.acc_dt = declare("acc_dt", rclcpp::ParameterValue(0.2)).as_double();
  p.vx_samples = static_cast<int>(declare("vx_samples", rclcpp::ParameterValue(15)).as_int());
  p.vtheta_samples = static_cast<int>(
    declare("vtheta_samples", rclcpp::ParameterValue(31)).as_int());
  p.sim_time = declare("sim_time", rclcpp::ParameterValue(1.5)).as_double();
  p.sim_dt = declare("sim_dt", rclcpp::ParameterValue(0.1)).as_double();
  p.w_path = declare("w_path", rclcpp::ParameterValue(1.0)).as_double();
  p.w_goal = declare("w_goal", rclcpp::ParameterValue(1.2)).as_double();
  p.w_heading = declare("w_heading", rclcpp::ParameterValue(0.6)).as_double();
  p.w_obstacle = declare("w_obstacle", rclcpp::ParameterValue(1.0)).as_double();
  p.w_speed = declare("w_speed", rclcpp::ParameterValue(0.5)).as_double();
  p.allow_unknown = declare("allow_unknown", rclcpp::ParameterValue(false)).as_bool();
  p.lethal_threshold = static_cast<unsigned char>(
    declare("lethal_threshold", rclcpp::ParameterValue(253)).as_int());
  core_.setParams(p);

  base_max_vel_x_ = max_vel_x_ = p.max_vel_x;
  lookahead_dist_ = declare("lookahead_dist", rclcpp::ParameterValue(0.8)).as_double();
  approach_dist_ = declare("approach_dist", rclcpp::ParameterValue(0.6)).as_double();
  min_approach_vel_ = declare("min_approach_vel", rclcpp::ParameterValue(0.08)).as_double();
  rotate_in_place_angle_ = declare("rotate_in_place_angle", rclcpp::ParameterValue(1.2)).as_double();
  rotate_gain_ = declare("rotate_gain", rclcpp::ParameterValue(1.5)).as_double();
  final_rotate_dist_ = declare("final_rotate_dist", rclcpp::ParameterValue(0.2)).as_double();
  transform_tolerance_ = declare("transform_tolerance", rclcpp::ParameterValue(0.3)).as_double();
  publish_candidates_ = declare("publish_candidates", rclcpp::ParameterValue(true)).as_bool();
  candidate_stride_ = std::max(
    1, static_cast<int>(declare("candidate_stride", rclcpp::ParameterValue(3)).as_int()));

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  candidates_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "dwa_candidates", 1);

  RCLCPP_INFO(
    logger_, "DWAController '%s' configured: v<=%.2f m/s, w<=%.2f rad/s, %dx%d samples, "
    "sim %.1fs", plugin_name_.c_str(), p.max_vel_x, p.max_vel_theta, p.vx_samples,
    p.vtheta_samples, p.sim_time);
}

void DWAController::cleanup()
{
  local_plan_pub_.reset();
  candidates_pub_.reset();
}

void DWAController::activate()
{
  local_plan_pub_->on_activate();
  candidates_pub_->on_activate();
}

void DWAController::deactivate()
{
  local_plan_pub_->on_deactivate();
  candidates_pub_->on_deactivate();
}

void DWAController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> guard(mutex_);
  global_plan_ = path;
}

void DWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    max_vel_x_ = base_max_vel_x_;
  } else if (percentage) {
    max_vel_x_ = base_max_vel_x_ * speed_limit / 100.0;
  } else {
    max_vel_x_ = std::min(speed_limit, base_max_vel_x_);
  }
}

void DWAController::buildLocalPath(
  const Pose2D & robot, std::vector<Pose2D> & local_path, Pose2D & target, Pose2D & goal)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("DWAController: received an empty plan");
  }
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();

  geometry_msgs::msg::TransformStamped plan_to_costmap;
  try {
    plan_to_costmap = tf_->lookupTransform(
      costmap_frame, global_plan_.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    throw nav2_core::ControllerTFError(std::string("DWAController: ") + ex.what());
  }

  std::vector<Pose2D> plan;
  plan.reserve(global_plan_.poses.size());
  for (const auto & ps : global_plan_.poses) {
    geometry_msgs::msg::Pose out;
    tf2::doTransform(ps.pose, out, plan_to_costmap);
    plan.push_back({out.position.x, out.position.y, yawOf(out.orientation)});
  }
  goal = plan.back();

  // closest pose, searched only over the first few metres of the (already pruned) plan
  const double search_dist = std::max(2.0, 2.0 * lookahead_dist_);
  std::size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  double travelled = 0.0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (i > 0) {
      travelled += std::hypot(plan[i].x - plan[i - 1].x, plan[i].y - plan[i - 1].y);
      if (travelled > search_dist) {break;}
    }
    const double d = std::hypot(plan[i].x - robot.x, plan[i].y - robot.y);
    if (d < best) {
      best = d;
      closest = i;
    }
  }
  // prune what is behind the robot (keeps later searches local and cheap)
  if (closest > 0) {
    global_plan_.poses.erase(
      global_plan_.poses.begin(),
      global_plan_.poses.begin() + static_cast<std::ptrdiff_t>(closest));
    plan.erase(plan.begin(), plan.begin() + static_cast<std::ptrdiff_t>(closest));
  }

  // keep the part of the plan that lies inside the local costmap
  auto * costmap = costmap_ros_->getCostmap();
  const double horizon = 0.5 * std::min(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) -
    costmap->getResolution();
  local_path.clear();
  for (const auto & q : plan) {
    if (std::hypot(q.x - robot.x, q.y - robot.y) > horizon) {break;}
    local_path.push_back(q);
  }
  if (local_path.empty()) {
    local_path.push_back(plan.front());
  }

  target = local_path.back();
  for (const auto & q : local_path) {
    if (std::hypot(q.x - robot.x, q.y - robot.y) >= lookahead_dist_) {
      target = q;
      break;
    }
  }
}

geometry_msgs::msg::TwistStamped DWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const DWAParams & p = core_.params();
  const std::string frame = costmap_ros_->getGlobalFrameID();

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = clock_->now();

  const Pose2D robot{pose.pose.position.x, pose.pose.position.y, yawOf(pose.pose.orientation)};
  std::vector<Pose2D> local_path;
  Pose2D target, goal;
  buildLocalPath(robot, local_path, target, goal);
  publishLocalPlan(local_path, frame);

  const double w0 = velocity.angular.z;
  const double dw = p.acc_lim_theta * p.acc_dt;
  auto rotate_towards = [&](double err) {
      const double w_des = std::clamp(rotate_gain_ * err, -p.max_vel_theta, p.max_vel_theta);
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = std::clamp(w_des, w0 - dw, w0 + dw);
      return cmd;
    };

  const double dist_to_goal = std::hypot(goal.x - robot.x, goal.y - robot.y);

  // 1) at the goal position: align with the goal heading (goal checker needs yaw)
  if (dist_to_goal < final_rotate_dist_) {
    return rotate_towards(DWACore::normalizeAngle(goal.theta - robot.theta));
  }

  // 2) large heading error: turn on the spot first (circular robot => always collision-safe
  //    as long as the current cell is not inside the inscribed radius)
  const double heading_err = DWACore::normalizeAngle(
    std::atan2(target.y - robot.y, target.x - robot.x) - robot.theta);
  if (std::fabs(heading_err) > rotate_in_place_angle_) {
    return rotate_towards(heading_err);
  }

  // 3) DWA search, with a linear slow-down zone in front of the goal
  double vmax = max_vel_x_;
  if (approach_dist_ > 0.0 && dist_to_goal < approach_dist_) {
    vmax = std::max(min_approach_vel_, vmax * dist_to_goal / approach_dist_);
  }

  DWAResult res;
  {
    auto * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    res = core_.plan(*costmap, robot, velocity.linear.x, w0, local_path, target, vmax);
  }
  publishVisuals(res, frame);

  if (!res.found) {
    throw nav2_core::NoValidControl(
            "DWAController: all " + std::to_string(res.rejected) + " sampled trajectories collide");
  }
  cmd.twist.linear.x = res.best.v;
  cmd.twist.angular.z = res.best.w;
  return cmd;
}

void DWAController::publishLocalPlan(const std::vector<Pose2D> & poses, const std::string & frame)
{
  if (!local_plan_pub_ || !local_plan_pub_->is_activated()) {return;}
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame;
  msg.header.stamp = clock_->now();
  for (const auto & q : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg.header;
    ps.pose.position.x = q.x;
    ps.pose.position.y = q.y;
    ps.pose.orientation = quatOf(q.theta);
    msg.poses.push_back(ps);
  }
  local_plan_pub_->publish(msg);
}

void DWAController::publishVisuals(const DWAResult & res, const std::string & frame)
{
  if (!publish_candidates_ || !candidates_pub_ || !candidates_pub_->is_activated()) {return;}
  visualization_msgs::msg::MarkerArray arr;
  const auto stamp = clock_->now();

  visualization_msgs::msg::Marker cand;
  cand.header.frame_id = frame;
  cand.header.stamp = stamp;
  cand.ns = "dwa_candidates";
  cand.id = 0;
  cand.type = visualization_msgs::msg::Marker::LINE_LIST;
  cand.action = visualization_msgs::msg::Marker::ADD;
  cand.pose.orientation.w = 1.0;
  cand.scale.x = 0.008;
  cand.color.r = 0.6f; cand.color.g = 0.6f; cand.color.b = 0.9f; cand.color.a = 0.5f;
  for (std::size_t i = 0; i < res.candidates.size(); i += candidate_stride_) {
    const auto & t = res.candidates[i];
    for (std::size_t k = 1; k < t.poses.size(); ++k) {
      geometry_msgs::msg::Point a, b;
      a.x = t.poses[k - 1].x; a.y = t.poses[k - 1].y; a.z = 0.02;
      b.x = t.poses[k].x; b.y = t.poses[k].y; b.z = 0.02;
      cand.points.push_back(a);
      cand.points.push_back(b);
    }
  }
  if (cand.points.empty()) {cand.action = visualization_msgs::msg::Marker::DELETE;}
  arr.markers.push_back(cand);

  visualization_msgs::msg::Marker best;
  best.header = cand.header;
  best.ns = "dwa_best";
  best.id = 1;
  best.type = visualization_msgs::msg::Marker::LINE_STRIP;
  best.action = res.found ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  best.pose.orientation.w = 1.0;
  best.scale.x = 0.03;
  best.color.r = 0.1f; best.color.g = 0.9f; best.color.b = 0.2f; best.color.a = 1.0f;
  if (res.found) {
    for (const auto & q : res.best.poses) {
      geometry_msgs::msg::Point pt;
      pt.x = q.x; pt.y = q.y; pt.z = 0.04;
      best.points.push_back(pt);
    }
  }
  arr.markers.push_back(best);
  candidates_pub_->publish(arr);
}

}  // namespace amr_planners

PLUGINLIB_EXPORT_CLASS(amr_planners::DWAController, nav2_core::Controller)
