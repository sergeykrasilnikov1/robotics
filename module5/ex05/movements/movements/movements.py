#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class MovementsNode(Node):
    def __init__(self):
        super().__init__('movements_node')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        for _ in range(3):
            # Move forward
            twist.linear.x = 0.5 
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(2) 

            twist.linear.x = 0.0
            twist.angular.z = math.radians(120) / 1.5  
            self.publisher_.publish(twist)
            time.sleep(1.5)  

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MovementsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

