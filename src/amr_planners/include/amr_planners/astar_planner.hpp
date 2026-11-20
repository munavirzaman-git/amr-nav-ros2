// Copyright 2026 Munavir Zaman — Apache-2.0
// Nav2 GlobalPlanner plugin wrapping GridAStar.
#ifndef AMR_PLANNERS__ASTAR_PLANNER_HPP_
#define AMR_PLANNERS__ASTAR_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "amr_planners/astar_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace amr_planners
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string global_frame_;

  GridAStar astar_;
  SmootherParams smoother_params_;
  bool smooth_path_{true};
  double tolerance_{0.3};
  double max_planning_time_{2.0};
};

}  // namespace amr_planners

#endif  // AMR_PLANNERS__ASTAR_PLANNER_HPP_
