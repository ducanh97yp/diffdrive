#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusTopicPrinter(Node):
    def __init__(self):
        super().__init__('status_printer')
        
        # Subscribe to the status topic
        self.create_subscription(
            String, 
            '/mola_diagnostics/lidar_odom/status', 
            self.status_callback, 
            10)
        
        self.get_logger().info('Status printer node started. Listening for data on /mola_diagnostics/lidar_odom/status...')

    def status_callback(self, msg):
        # Print the entire data string to the console
        self.get_logger().info('--- New Status Message Received ---')
        self.get_logger().info(f"\n{msg.data}")
        self.get_logger().info('-------------------------------------')


def main(args=None):
    rclpy.init(args=args)
    node = StatusTopicPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down status printer...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()