#!/usr/bin/env python

import glob
from ament_index_python import get_package_share_directory
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
import numpy as np



class Robot(Node):
    def __init__(self):
        super().__init__('rwander_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('pose_topic', 'rosbot_pose')
        vel_topic_ = self.get_parameter('vel_topic').value
        scan_topic_ = self.get_parameter('scan_topic').value
        pose_topic_ = self.get_parameter('pose_topic').value

        # ROS Subscribers
        self._laser_sub = self.create_subscription(LaserScan, scan_topic_, self.obstacle_detect, 10)
        self._pose_sub = self.create_subscription(Pose2D, pose_topic_, self.pose_callback, 10)
        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, vel_topic_, 10)       
        self._scan_count = 0
        self._max_range = 100.0
        self._uninitialized = 1
        self._bearings = []
        self._scan = []   
    
    def most_space(self, scan_arr:list):

        mostSpaceIndex = -1
        maxSpaceWeight = -1
        mostDepthIndex = -1
        wayDepth = -1

        spaceAmount = 0

        for ix in range(len(scan_arr)):
            if scan_arr[ix] > 4.5:
                spaceAmount+=1
                if wayDepth<scan_arr[ix]:
                    wayDepth = scan_arr[ix]
                    mostDepthIndex = ix
            else:
                cur_weight = spaceAmount*0.9 + wayDepth*0.1
                # Calculate Weight amountSpace*0.6 + maxDepth*0.4
                if cur_weight>maxSpaceWeight: # if new weight> max weight
                    maxSpaceWeight = cur_weight
                    rightIndex = ix
                    leftIndex = ix-spaceAmount
                    spaceIndex = (rightIndex+leftIndex)/2
                    mostSpaceIndex = spaceIndex
                    #mostSpaceIndex = (spaceIndex+mostDepthIndex)/2
                spaceAmount = 0
                wayDepth = -1
                mostDepthIndex=-1
        return int(mostSpaceIndex)
    
    def prevent_front_wall(self, scan_arr:list, index:int):
        sum = True
        if scan_arr[index+10]<scan_arr[index-10]: sum = False
        while scan_arr[index]<4.5:
            if sum:
                index-=1
            else:
                index+=1
            
            if index>=len(scan_arr)-1 or index<-1:
                self.get_logger().info("FORCED BREAK")
                break
        return index
            

    def obstacle_detect(self, scan_msg):
        
        self._scan = scan_msg.ranges

        if self._uninitialized:
            self._uninitialized = 0
            self._scan_count = len(scan_msg.ranges)
            self._max_range = scan_msg.range_max
            # Bearings stores the angle of each range measurement (radians)
            for i in range(0, self._scan_count):
                self._bearings.append(scan_msg.angle_min + scan_msg.angle_increment * i)
            self.get_logger().info("# Scan count %d"%(self._scan_count))  
            self.get_logger().info("# Laser angle min: %.2f"%(np.rad2deg(scan_msg.angle_min)))
            self.get_logger().info("# Laser angle max: %.2f"%(np.rad2deg(scan_msg.angle_max)))
            self.get_logger().info("# Laser angle increment:  %.4f rad (%.2f deg)"%(scan_msg.angle_increment, np.rad2deg(scan_msg.angle_increment)))
            self.get_logger().info("# Time between mesurements [seconds]:  %.2f"%(scan_msg.time_increment))
            self.get_logger().info("# Time between scans [seconds]:  %.2f"%(scan_msg.scan_time))
            self.get_logger().info("# Minimum range value:  %.2f"%(scan_msg.range_min))
            self.get_logger().info("# Maximum range value:  %.2f "%(scan_msg.range_max))
            resolution = (scan_msg.angle_max - scan_msg.angle_min)/len(scan_msg.ranges)
            self.get_logger().info("# Resolution:  %.2f"%(np.rad2deg(resolution))) 

        # Replace infinity values with max_range 
        self._scan = [x if x < self._max_range else self._max_range for x in self._scan] 

        # Reorganize scan indexes to make it easier to work with. 
        # 0 index corresponds to the back side of the robot for both, scan and bearings.
        self._scan = [self._scan[i - 800] for i in range(self._scan_count)]  
    
        # TODO: add your code here

        #filterScan = [0 if val < 3 else 1 for val in self._scan[400:1200]]
        filterScan = self._scan[400:1200] 
        self.get_logger().info("Filetered Scanner " + str(filterScan))
        mostSpaceIx = self.most_space(filterScan)
        finalIndex = mostSpaceIx
        #finalIndex = self.prevent_front_wall(filterScan,mostSpaceIx)
        self.get_logger().info("Index with most SPace: " + str(finalIndex))
        turn = 0.025*(finalIndex-400)
        speed = 8*(np.cos(turn))
        # turn = 0.0
        # speed = 0.0

        
        ## end TODO
        cmd_vel_msg_ = Twist()
        cmd_vel_msg_.linear.x  = speed
        cmd_vel_msg_.linear.y  = 0.0
        cmd_vel_msg_.angular.z = turn
        self._cmd_vel_pub.publish( cmd_vel_msg_ ) 
    
    def pose_callback(self, msg):
        pass
        #self.get_logger().info("Robot pose: x: %.2f, y: %.2f, theta: %.2f"%(msg.x, msg.y, msg.theta))
        
        pass
                                            
def main(args=None):
    rclpy.init(args=args)
    rwander_node = Robot()
    try:
        rclpy.spin(rwander_node)
    except KeyboardInterrupt:
        rwander_node.get_logger().info("Node interrupted by keyboard (CTRL+C)")
    finally:  
        rwander_node.destroy_node()  
        rclpy.shutdown()
    
if __name__=="__main__":
    main()
