#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
from tf_transformations import euler_from_quaternion
from rclpy.signals import SignalHandlerOptions 
import csv
import math as m

class Robot(Node):
    def __init__(self):
        super().__init__('wall_follow_d')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 1.5),
                ('distance', 1.0),
                ('output_filename', 'wf_d')
            ]
        )
        
        # kp value to use in the execution
        self.kp = self.get_parameter("kp").value

        # # follow the wall at a concrete distance
        self.dist = self.get_parameter('distance').value

        # velocity topic      
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
    
        # Pose topic
        self.pose_sub = self.create_subscription(Pose2D, "rosbot_pose", self.get_position, 1)

        ''' output filename (with no extension). 
        kp value and .csv extension will be added 
        to this name afterwards 
        '''
        # self.declare_parameter("output_filename", 'wf_kp')
        f = self.get_parameter('output_filename').value
        fout = f + '_' + str(self.kp) + '.csv'
        file = open(fout, 'w')
        self.csvwriter = csv.writer(file, delimiter = ',')
        self.csvwriter.writerow(['kp', 'error', 'v', 'w', 'x', 'y'])        
        
        self.get_logger().info("The robot will follow the wall at distance %.2f using kp = %.2f"%(self.dist, self.kp))

        # LaserScan topic
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.follow_wall, 1)
            
        self.scan_count = 0
        self.rx = 0.0
        self.ry = 0.0
        self.rtheta = 0.0
        self.uninitialized = True
        
    def get_position(self, msg):
        # Gets robot pose (x, y, yaw) from odometry
        self.rx = msg.x
        self.ry = msg.y
        self.rtheta = msg.theta
        
    
    def follow_wall(self, scan):             
        if self.uninitialized:
            self.uninitialized = False
            self.scan_count = len(scan.ranges)
        self.scan = [scan.ranges[i - 800] for i in range(self.scan_count)]
        ## TODO

        # 1.- Kalkulatu paretarako batezbesteko distantzia laserra erabiliz / Use laser to calculate distance to the right wall
        realdist = 0.0
	    # 2.- Kalkulatu errorea / Compute the error
        err = (self.dist - realdist)
        
        # 3.- Abiadurak finkatu proportzionalki / Set velocities proportionally
        w = min(max(err * self.kp, -0.5), 0.5)
        v = 0.0
        ## END TODO

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x  = v
        cmd_vel_msg.linear.y  = 0.0
        cmd_vel_msg.angular.z = w
        self.vel_pub.publish( cmd_vel_msg )
        self.csvwriter.writerow([f"{self.kp:.2f}", f"{err:.2f}", f"{v:.2f}", f"{w:.2f}", f"{self.rx:.2f}", f"{self.ry:.2f}"])   
def main(args=None):
    rclpy.init(args=args) 
    wf_node = Robot()
    try:
        rclpy.spin(wf_node)
    except KeyboardInterrupt:
        wf_node.destroy_node()

if __name__ == '__main__':
    main()
