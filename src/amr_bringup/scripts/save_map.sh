#!/usr/bin/env bash
# Save the current SLAM Toolbox map as <prefix>.pgm/.yaml (for map_server/AMCL)
# and as a serialized pose graph <prefix>.posegraph/.data (to continue mapping later).
#   ros2 run amr_bringup save_map.sh ~/maps/warehouse_slam
set -e
PREFIX=${1:-$HOME/maps/warehouse_slam}
mkdir -p "$(dirname "$PREFIX")"
ros2 run nav2_map_server map_saver_cli -f "$PREFIX" --ros-args -p use_sim_time:=true \
  -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$PREFIX'}" || echo "(pose-graph serialization skipped)"
echo "Map saved to ${PREFIX}.yaml"
