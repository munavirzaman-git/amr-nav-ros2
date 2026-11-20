"""Drives the dynamic obstacles in Gazebo along cyclic patrol routes.

Each obstacle has gz VelocityControl + OdometryPublisher systems. Waypoints are
offsets from the first odometry pose received, so the node does not depend on
whether the odometry is world-referenced or start-referenced.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


class Obstacle:
    def __init__(self, name, waypoints, speed):
        self.name = name
        self.wps = [(waypoints[i], waypoints[i + 1]) for i in range(0, len(waypoints) - 1, 2)]
        self.speed = speed
        self.origin = None
        self.pose = None   # (x, y, yaw)
        self.idx = 0


class DynamicObstacleDriver(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_driver')
        self.declare_parameter('obstacles', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('reach_tolerance', 0.1)
        names = self.get_parameter('obstacles').value or []
        self.tol = self.get_parameter('reach_tolerance').value

        self.obstacles = []
        self.pubs = {}
        for n in names:
            self.declare_parameter(f'{n}.waypoints', Parameter.Type.DOUBLE_ARRAY)
            self.declare_parameter(f'{n}.speed', 0.3)
            wps = list(self.get_parameter(f'{n}.waypoints').value or [])
            if len(wps) < 4:
                self.get_logger().warn(f'{n}: needs >= 2 waypoints, skipping')
                continue
            ob = Obstacle(n, wps, self.get_parameter(f'{n}.speed').value)
            self.obstacles.append(ob)
            self.pubs[n] = self.create_publisher(Twist, f'/model/{n}/cmd_vel', 10)
            self.create_subscription(
                Odometry, f'/model/{n}/odometry', lambda m, o=ob: self.on_odom(o, m), 10)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(f'driving {len(self.obstacles)} dynamic obstacles')

    @staticmethod
    def yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_odom(self, ob, msg):
        p = msg.pose.pose
        ob.pose = (p.position.x, p.position.y, self.yaw(p.orientation))
        if ob.origin is None:
            ob.origin = (p.position.x, p.position.y)

    def step(self):
        for ob in self.obstacles:
            cmd = Twist()
            if ob.pose is not None:
                tx = ob.origin[0] + ob.wps[ob.idx][0]
                ty = ob.origin[1] + ob.wps[ob.idx][1]
                ex, ey = tx - ob.pose[0], ty - ob.pose[1]
                d = math.hypot(ex, ey)
                if d < self.tol:
                    ob.idx = (ob.idx + 1) % len(ob.wps)
                else:
                    v = min(ob.speed, 1.5 * d)        # ease into the turn-around point
                    vx, vy = v * ex / d, v * ey / d    # world frame
                    c, s = math.cos(ob.pose[2]), math.sin(ob.pose[2])
                    cmd.linear.x = c * vx + s * vy     # -> body frame
                    cmd.linear.y = -s * vx + c * vy
            self.pubs[ob.name].publish(cmd)


def main():
    rclpy.init()
    node = DynamicObstacleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
