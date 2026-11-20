// Copyright 2026 Munavir Zaman — Apache-2.0
// Nav2 Controller plugin wrapping DWACore.
#ifndef AMR_PLANNERS__DWA_CONTROLLER_HPP_
#define AMR_PLANNERS__DWA_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amr_planners/dwa_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr_planners
{

class DWAController : public nav2_core::Controller
{
public:
  DWAController() = default;
  ~DWAController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  /// Transform + prune the global plan into the costmap frame and pick the lookahead target.
  void buildLocalPath(
    const Pose2D & robot, std::vector<Pose2D> & local_path, Pose2D & target, Pose2D & goal);
  void publishVisuals(const DWAResult & res, const std::string & frame);
  void publishLocalPlan(const std::vector<Pose2D> & poses, const std::string & frame);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("DWAController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  std::mutex mutex_;
  nav_msgs::msg::Path global_plan_;
  DWACore core_;

  double base_max_vel_x_{0.5};
  double max_vel_x_{0.5};         // after speed limits
  double lookahead_dist_{0.8};
  double approach_dist_{0.6};
  double min_approach_vel_{0.08};
  double rotate_in_place_angle_{1.2};
  double rotate_gain_{1.5};
  double final_rotate_dist_{0.2};
  double transform_tolerance_{0.3};
  bool publish_candidates_{true};
  int candidate_stride_{3};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    candidates_pub_;
};

}  // namespace amr_planners

#endif  // AMR_PLANNERS__DWA_CONTROLLER_HPP_
