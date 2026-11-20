# Changelog

All notable changes to this project are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow [SemVer](https://semver.org/).

## [1.0.0] - 2026-11-20

### Added
- Differential-drive AMR model with ros2_control wheel encoders, GPU LiDAR and IMU (`amr_description`).
- EKF odometry fusing encoder odometry and IMU yaw rate (`robot_localization`).
- Warehouse world and pixel-aligned ground-truth map generated from one specification (`amr_gazebo`).
- Two velocity-controlled dynamic obstacles with patrol routes.
- SLAM Toolbox mapping (teleop and fully autonomous `auto_mapper`), map saving and pose-graph serialization.
- AMCL localization and the full Nav2 stack with velocity smoother and collision monitor.
- `amr_planners::AStarPlanner` (Nav2 `GlobalPlanner`) and `amr_planners::DWAController` (Nav2 `Controller`),
  with ROS-free cores and 12 unit tests.
- Launch-time selection of planner (`navfn | smac | astar`) and controller (`mppi | dwb | dwa`).
- Benchmarks: `planner_benchmark`, `nav_benchmark`, `analyze_results`, `run_comparison.sh`.
- SLAM map accuracy evaluation against ground truth (`compare_maps`).
- CI (lint, build, unit tests, parameter and map-evaluation checks), Docker image, documentation.

### Fixed
- Gazebo transport discovery on hosts with Docker/VPN interfaces (`GZ_IP` pinned to loopback).
