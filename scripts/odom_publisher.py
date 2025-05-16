#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry



class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.create_timer(0.1, self.publish_odom)
        self.publisher = self.create_publisher(Odometry, '/odometry/encoders', 10)
        
    def publish_odom(self):
        self.publisher.publish(Odometry())

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()