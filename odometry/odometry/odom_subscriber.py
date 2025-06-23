#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        self.get_logger().info(f'Position: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, theta={msg.pose.pose.orientation.z}')

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

