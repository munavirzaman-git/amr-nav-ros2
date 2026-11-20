"""Twist -> TwistStamped relay.

Nav2 (collision_monitor) and teleop_twist_keyboard publish geometry_msgs/Twist on
/cmd_vel, while diff_drive_controller in Jazzy consumes geometry_msgs/TwistStamped.
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_footprint')
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(TwistStamped, out, 10)
        self.create_subscription(Twist, inp, self.on_twist, 10)
        self.get_logger().info(f'relaying {inp} (Twist) -> {out} (TwistStamped)')

    def on_twist(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelRelay()
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
