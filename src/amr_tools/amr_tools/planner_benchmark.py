"""Global planner benchmark: NavFn vs Smac2D vs custom A* on identical queries.

All three planners are loaded side by side in planner_server (see nav2_params.yaml),
so this needs only one running stack and does not move the robot.

  ros2 run amr_tools planner_benchmark --planners NavFn Smac AStar --repeats 10
"""
import argparse
import csv
import math
import os
import statistics
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_tools.common import angle_diff, load_yaml, make_pose

FIELDS = ['planner', 'query', 'rep', 'success', 'wall_ms', 'planner_ms', 'length_m',
          'n_poses', 'total_turn_rad', 'max_cost', 'mean_cost']


class CostmapCache:
    def __init__(self, node, topic):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.grid = None
        node.create_subscription(OccupancyGrid, topic, self.cb, qos)

    def cb(self, msg):
        self.grid = msg

    def cost(self, x, y):
        g = self.grid
        if g is None:
            return None
        i = int((x - g.info.origin.position.x) / g.info.resolution)
        j = int((y - g.info.origin.position.y) / g.info.resolution)
        if 0 <= i < g.info.width and 0 <= j < g.info.height:
            return g.data[j * g.info.width + i]
        return None


def path_metrics(path, cmap):
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    turn, prev = 0.0, None
    for a, b in zip(pts, pts[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-3:
            continue
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        if prev is not None:
            turn += abs(angle_diff(h, prev))
        prev = h
    costs = [c for c in (cmap.cost(x, y) for x, y in pts) if c is not None and c >= 0]
    return {
        'length_m': round(length, 3), 'n_poses': len(pts), 'total_turn_rad': round(turn, 3),
        'max_cost': max(costs) if costs else '',
        'mean_cost': round(statistics.mean(costs), 2) if costs else '',
    }


def compute(nav, start, goal, planner):
    """Return (path, planner_ms). Uses the private impl to read planning_time when present."""
    try:
        res = nav._getPathImpl(start, goal, planner, use_start=True)
        if res is None or nav.status != GoalStatus.STATUS_SUCCEEDED:
            return None, None
        pt = res.planning_time
        return res.path, pt.sec * 1e3 + pt.nanosec * 1e-6
    except AttributeError:
        return nav.getPath(start, goal, planner_id=planner, use_start=True), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--planners', nargs='+', default=['NavFn', 'Smac', 'AStar'])
    ap.add_argument('--goals', default=os.path.join(
        get_package_share_directory('amr_bringup'), 'config', 'benchmark_goals.yaml'))
    ap.add_argument('--repeats', type=int, default=5)
    ap.add_argument('--localizer', default='amcl')
    ap.add_argument('--costmap', default='/global_costmap/costmap')
    ap.add_argument('--output', default=os.path.expanduser('~/amr_benchmarks/planner_results.csv'))
    args, _ = ap.parse_known_args()

    cfg = load_yaml(args.goals)
    rclpy.init()
    nav = BasicNavigator(node_name='planner_benchmark')
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    cmap = CostmapCache(nav, args.costmap)
    nav.waitUntilNav2Active(localizer=args.localizer)

    t_end = time.time() + 10.0
    while cmap.grid is None and time.time() < t_end:
        rclpy.spin_once(nav, timeout_sec=0.2)
    if cmap.grid is None:
        nav.get_logger().warn('no global costmap received: cost metrics will be empty')

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    new_file = not os.path.exists(args.output)
    with open(args.output, 'a', newline='') as out:
        w = csv.DictWriter(out, fieldnames=FIELDS)
        if new_file:
            w.writeheader()
        for q in cfg['planner_queries']:
            start = make_pose(nav, *q['start'])
            goal = make_pose(nav, *q['goal'])
            for planner in args.planners:
                for rep in range(args.repeats):
                    t0 = time.perf_counter()
                    path, planner_ms = compute(nav, start, goal, planner)
                    wall_ms = (time.perf_counter() - t0) * 1e3
                    row = {'planner': planner, 'query': q['name'], 'rep': rep,
                           'success': int(path is not None and len(path.poses) > 0),
                           'wall_ms': round(wall_ms, 2),
                           'planner_ms': round(planner_ms, 3) if planner_ms is not None else ''}
                    if row['success']:
                        row.update(path_metrics(path, cmap))
                    w.writerow(row)
                nav.get_logger().info(f'{q["name"]:>14s} | {planner:>6s} done')
    nav.get_logger().info(f'results appended to {args.output}')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
