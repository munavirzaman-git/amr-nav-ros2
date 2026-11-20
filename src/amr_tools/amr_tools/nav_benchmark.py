"""End-to-end navigation benchmark (planner + controller + recoveries).

Drives the robot through the goal sequence in benchmark_goals.yaml and records,
per goal: success, time, travelled distance, minimum obstacle distance, command
smoothness, recoveries and final pose error. Run it once per stack configuration
(e.g. navfn+mppi, astar+dwa) with a different --label, then use analyze_results.

  ros2 run amr_tools nav_benchmark --label astar_dwa --repeats 3
"""
import argparse
import csv
import math
import os

import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from amr_tools.common import angle_diff, load_yaml, make_pose, quat_to_yaw

FIELDS = ['label', 'run', 'goal', 'result', 'success', 'time_s', 'path_length_m',
          'mean_speed_mps', 'min_obstacle_dist_m', 'mean_abs_dw_dt', 'recoveries',
          'final_pos_err_m', 'final_yaw_err_rad']


class Recorder:
    """Collects metrics from topics while the navigator spins its own node."""

    def __init__(self, node, odom_topic):
        node.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        node.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        self.reset()

    def reset(self):
        self.active = False
        self.dist = 0.0
        self.last_xy = None
        self.min_range = math.inf
        self.last_w = None
        self.dw_sum = 0.0
        self.dw_n = 0

    def on_odom(self, msg):
        if not self.active:
            return
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def on_scan(self, msg):
        if not self.active:
            return
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and r < self.min_range:
                self.min_range = r

    def on_cmd(self, msg):
        if not self.active:
            return
        w = msg.angular.z
        if self.last_w is not None:
            self.dw_sum += abs(w - self.last_w)
            self.dw_n += 1
        self.last_w = w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True, help='e.g. navfn_mppi, astar_dwa')
    ap.add_argument('--goals', default=os.path.join(
        get_package_share_directory('amr_bringup'), 'config', 'benchmark_goals.yaml'))
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=120.0, help='per-goal timeout [sim s]')
    ap.add_argument('--localizer', default='amcl',
                    help="'amcl', or 'robot_localization' to skip the localizer check (SLAM)")
    ap.add_argument('--odom', default='/odometry/filtered')
    ap.add_argument('--cmd-rate', type=float, default=20.0, help='controller rate for |dw/dt|')
    ap.add_argument('--output', default=os.path.expanduser('~/amr_benchmarks/nav_results.csv'))
    args, _ = ap.parse_known_args()

    cfg = load_yaml(args.goals)
    rclpy.init()
    nav = BasicNavigator(node_name='nav_benchmark')
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    rec = Recorder(nav, args.odom)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, nav)

    if args.localizer == 'amcl' and 'initial_pose' in cfg:
        x, y, yaw = cfg['initial_pose']
        nav.setInitialPose(make_pose(nav, x, y, yaw))
    nav.waitUntilNav2Active(localizer=args.localizer)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    new_file = not os.path.exists(args.output)
    out = open(args.output, 'a', newline='')
    writer = csv.DictWriter(out, fieldnames=FIELDS)
    if new_file:
        writer.writeheader()

    for run in range(args.repeats):
        for g in cfg['goals']:
            name = g['name']
            gx, gy, gyaw = g['pose']
            rec.reset()
            rec.active = True
            t0 = nav.get_clock().now()
            nav.get_logger().info(f'[{args.label}] run {run} -> {name} ({gx:.1f}, {gy:.1f})')
            recoveries = 0
            if not nav.goToPose(make_pose(nav, gx, gy, gyaw)):
                result = TaskResult.FAILED
            else:
                while not nav.isTaskComplete():
                    fb = nav.getFeedback()
                    if fb is not None:
                        recoveries = fb.number_of_recoveries
                    if nav.get_clock().now() - t0 > Duration(seconds=args.timeout):
                        nav.get_logger().warn('timeout -> cancelling')
                        nav.cancelTask()
                result = nav.getResult()
            rec.active = False
            elapsed = (nav.get_clock().now() - t0).nanoseconds * 1e-9

            for _ in range(5):                       # let TF catch up
                rclpy.spin_once(nav, timeout_sec=0.05)
            pos_err = yaw_err = float('nan')
            try:
                tf = tf_buffer.lookup_transform('map', 'base_footprint', Time())
                tr = tf.transform.translation
                pos_err = math.hypot(tr.x - gx, tr.y - gy)
                yaw_err = abs(angle_diff(quat_to_yaw(tf.transform.rotation), gyaw))
            except tf2_ros.TransformException as e:
                nav.get_logger().warn(f'TF lookup failed: {e}')

            ok = result == TaskResult.SUCCEEDED
            row = {
                'label': args.label, 'run': run, 'goal': name,
                'result': getattr(result, 'name', str(result)), 'success': int(ok),
                'time_s': round(elapsed, 3), 'path_length_m': round(rec.dist, 3),
                'mean_speed_mps': round(rec.dist / elapsed, 3) if elapsed > 0 else 0.0,
                'min_obstacle_dist_m': round(rec.min_range, 3) if rec.min_range < math.inf else '',
                'mean_abs_dw_dt': round(rec.dw_sum / rec.dw_n * args.cmd_rate, 4) if rec.dw_n else 0.0,
                'recoveries': recoveries,
                'final_pos_err_m': round(pos_err, 3), 'final_yaw_err_rad': round(yaw_err, 3),
            }
            writer.writerow(row)
            out.flush()
            nav.get_logger().info(f'  {row["result"]}: {elapsed:.1f}s, {rec.dist:.2f} m, '
                                  f'{recoveries} recoveries')

    out.close()
    nav.get_logger().info(f'results appended to {args.output}')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
