#!/usr/bin/env python3
import rclpy
import os
from geometry_msgs.msg import Twist
from rclpy.node import Node

class Robot(Node):
    def __init__(self):
        super().__init__('rosbot_spin_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        vel_topic = self.get_parameter('vel_topic').value
        self.declare_parameter('angular_vel', 0.4)
        self.w = self.get_parameter('angular_vel').value
        self.cmd_vel_pub = self.create_publisher(Twist, vel_topic, 10)
        self.create_timer(0.1, self.rotate)
        
    def rotate(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.angular.z = self.w
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
def main(args=None):
    rclpy.init(args=args)
    rspin_node = Robot()
    rclpy.spin(rspin_node)
    rspin_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
