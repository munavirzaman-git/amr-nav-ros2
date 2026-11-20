"""Autonomous mapping: drive a coverage tour with Nav2 while SLAM Toolbox maps,
then save the map. Replaces manual teleop mapping.

  terminal 1: ros2 launch amr_bringup bringup.launch.py slam:=true dynamic_obstacles:=false
  terminal 2: ros2 run amr_tools auto_mapper --output ~/maps/warehouse_slam
"""
import argparse
import os
import subprocess

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from amr_tools.common import make_pose

# Coverage tour through every aisle of the warehouse (x, y, yaw); all >= 0.68 m from obstacles.
TOUR = [
    (4.5, 0.0, 0.0), (7.25, 3.0, 1.57), (8.5, 5.0, 1.57), (2.0, 5.5, 3.14),
    (-2.0, 5.5, 3.14), (-7.0, 5.5, 3.14), (-8.5, 2.0, -1.57), (-8.5, -2.0, -1.57),
    (-6.5, -5.5, 0.0), (-2.0, -5.5, 0.0), (2.0, -5.5, 0.0), (6.5, -5.5, 0.0),
    (8.5, -2.5, 1.57), (0.0, 0.0, 3.14),
]


def save_map(prefix, log):
    os.makedirs(os.path.dirname(prefix) or '.', exist_ok=True)
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix,
           '--ros-args', '-p', 'use_sim_time:=true', '-p', 'map_subscribe_transient_local:=true']
    log.info('saving map: ' + ' '.join(cmd))
    ok = subprocess.run(cmd).returncode == 0
    subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                    'slam_toolbox/srv/SerializePoseGraph', f"{{filename: '{prefix}'}}"])
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default=os.path.expanduser('~/maps/warehouse_slam'))
    ap.add_argument('--timeout', type=float, default=90.0, help='per-waypoint timeout [sim s]')
    args, _ = ap.parse_known_args()
    prefix = os.path.expanduser(args.output)

    rclpy.init()
    nav = BasicNavigator(node_name='auto_mapper')
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    log = nav.get_logger()
    # SLAM provides map->odom, so do not wait for AMCL
    nav.waitUntilNav2Active(localizer='robot_localization')

    reached = 0
    for i, (x, y, yaw) in enumerate(TOUR):
        log.info(f'waypoint {i + 1}/{len(TOUR)}: ({x:.1f}, {y:.1f})')
        if not nav.goToPose(make_pose(nav, x, y, yaw)):
            log.warn('  goal rejected, skipping')
            continue
        t0 = nav.get_clock().now()
        while not nav.isTaskComplete():
            if nav.get_clock().now() - t0 > Duration(seconds=args.timeout):
                log.warn('  timeout, skipping')
                nav.cancelTask()
        result = nav.getResult()
        reached += result == TaskResult.SUCCEEDED
        log.info(f'  {getattr(result, "name", result)}')

    log.info(f'tour finished: {reached}/{len(TOUR)} waypoints reached')
    if save_map(prefix, log):
        log.info(f'map saved: {prefix}.yaml / {prefix}.pgm')
    else:
        log.error('map_saver_cli failed - is slam_toolbox publishing /map?')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
