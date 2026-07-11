#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelStamper(Node):
    """Stamps /cmd_vel into /cmd_vel_stamped for diff_drive_controller - Jazzy's
    diff_drive_controller takes TwistStamped, but teleop_twist_keyboard and Nav2's
    controller_server both publish plain Twist on /cmd_vel.
    """

    def __init__(self):
        super().__init__('cmd_vel_stamper')

        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.stamp_cmd_vel, 10)

    def stamp_cmd_vel(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
