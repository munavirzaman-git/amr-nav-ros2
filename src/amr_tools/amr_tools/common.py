"""Shared helpers for the benchmark tools."""
import math

import yaml
from geometry_msgs.msg import PoseStamped


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_pose(node, x, y, yaw, frame='map'):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    _, _, ps.pose.orientation.z, ps.pose.orientation.w = yaw_to_quat(float(yaw))
    return ps


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)
