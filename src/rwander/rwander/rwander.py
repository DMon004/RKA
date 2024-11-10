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
        # Hasieraketak:
        # mostSpaceIndex: Gordeko du nondik joan behar den robota
        # maxSpace: for-ean aurkitu den pareta gabeko zati handienaren tamaina
        # spaceAmount: Pareta bat aurkitu denetik pareta gabeko zatiaren tamaina gordeko du
        mostSpaceIndex = 0
        maxSpace= -1
        spaceAmount = 0

        # LIDAR-ak lortu duen balio guztiak begiratzen ditugu, 
        # ikusteko non dagoen pareta eta non ez
        for ix in range(len(scan_arr)):
            # LIDAR-aren balioa 2.6 baino handiagoa bada, ez da pareta.
            # 2.6 izatea erabaki dugu proba asko egin eta gero
            if scan_arr[ix] > 2.6:
                # Pareta gabekoren tamaina haunditzen dugu
                spaceAmount+=1
            else:
                # Ez bada pareta, ikusi behar dugu ea gutxienez pareta ez den pixka bat ikusi duen ikusi behar dugu
                if spaceAmount>0:
                    # IKusi badu pareta ez den zerbait, 
                    # jakin nahi dugu zati hori jadanik aurkitutako baino handiagoa bada
                    if spaceAmount>maxSpace:

                        # Behin jakinda oraingo bidea aurreko baino handiagoa dela,
                        # Kalkulatzen dugu erdiko puntua, robota hortik joateko,
                        # ez badugu beste zatia handiago bat
                        rightIndex = ix
                        leftIndex = ix-spaceAmount
                        spaceIndex = (rightIndex+leftIndex)/2
                        mostSpaceIndex = spaceIndex
                spaceAmount = 0
        return int(mostSpaceIndex)
    
    def wall_prevention(self,scan_arr:list, index:int):

        # Honeki, ikusten dugu ea robotaren aldea bakoiztean
        # Paretaren bat gertuegi dagoen ala ez.
        left_wall = any([True if (val<1.5 or val==-np.inf) else False for val in scan_arr[:400]])
        right_wall = any([True if (val<1.5 or val==-np.inf) else False for val in scan_arr[401:]])

        # Paretaren bat gertuegi badago,
        # Paretatik aldendu behar dugu robota
        # Gauza berdina egiten da pareta ezkerrean ala eskuinean badago
        # Aldatzen den gauza bakarra robotaren norazkoa izango da, paretatik aldendu ahal izateko
        if right_wall and not left_wall:

            #Lortzen dugu paretatik gertuen dagoen puntuaren distantzia
            min_depth = min(scan_arr[401:])

            # -inf bueltatzen badu, hau da LIDAR-aren puntu itsuan badago pareta,
            # -1 batera jartzen dugu distantzia, kalkuluak egin ahal izateko
            if min_depth==-np.inf: min_depth=-1

            # Formula honekin kalkulatzen dugu zenabt biratu behar den robota paretatik aldentzeko
            # 1-min_depth egiten dugu, paretaren oso gertu bagaude, bira handiago egiteko
            # Aurreko balioarin +1 egiten diogu, gero 8rekin biderkatzerakoan, zenbakia handiago izateko.
            index-=int(8*(1+(1-min_depth)))

            # Gertatu ahal da, lortuako balioa, nahi dugu direkzio batera joatea, adibidez,
            # atzerantz-martxa egitea, horretarako behartzen dugu balio minimo batzuk izatera beti

            if index<0: 
                index=0
        # Beste paretarekin berdina egiten dugu    
        elif left_wall and not right_wall:
            min_depth = min(scan_arr[:400])

            if min_depth==-np.inf: min_depth=-1
            
            index+=int(8*(1+(1-min_depth)))
            if index>799: 
                index=799

        # Bi paretak aurkitzen baditugu, ez dugu ezer egin behar, hobe da robot-ak 'aurrera' jarraitzea
        # Eta pareta batetik gerturatzen denean, bestetik aldendu beharko da, 
        # beraz hor metodoak ondo funtzionatuko du.

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

        # _scan-ek ditugu zirkunferentzia osoko balioak, baina guk bakarrik behar dugu,
        # zirkunferentzia horren goiko zatia, beraz array-a zatitzen dugu guk behar ditugun balioekin
        filterScan = self._scan[400:1200] 

        # most_space metodoarik deitzen diogu, kalkulatuko duena nondik joan behar den robota
        mostSpaceIx = self.most_space(filterScan)

        # Gero, ikusten dugu ea jasotako direkzioari ahaldaketak egin behar badiogu,
        # paretaren bat oso gertu baldin badago
        finalIndex= self.wall_prevention(filterScan,mostSpaceIx)
        
        # Abiadura lineala eta abaiura angeluarra kalkulatzen ditugu
        # Abiadura angeluarrarentzat, lortuako direkzioa, erdiarekiko diferentzia kalkulatzen dugu
        # eta gero 0.02 bidertzen dugu
        turn = 0.02*(finalIndex-400)

        # Abiadura lineala abiadura angeluarraren dependentea egin dugu.
        # Arazo nagusia abiadura angeluarra 0 bazen zegoen, horretarako cos() etabili dugu, 
        # 0 den balio bat ez duena inoiz bueltatuko
        # Horretaz aparte abiadura positibioa izateaz ziurtatu gara, robot-a atzerantz ez joateko.
        speed = abs(0.8*(1/np.cos(turn)))
        
        ## end TODO
        cmd_vel_msg_ = Twist()
        cmd_vel_msg_.linear.x  = speed
        cmd_vel_msg_.linear.y  = 0.0
        cmd_vel_msg_.angular.z = turn
        self._cmd_vel_pub.publish( cmd_vel_msg_ ) 
    
    def pose_callback(self, msg):
        self.get_logger().info("Robot pose: x: %.2f, y: %.2f, theta: %.2f"%(msg.x, msg.y, msg.theta))
        
                                            
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