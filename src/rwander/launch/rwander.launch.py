#import os
#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Robot Wander Node
    rwander_node = Node(
		package="rwander",
		executable="rwander",
		parameters=[
			{"vel_topic": "cmd_vel"},
			{"scan_topic": "scan"},
		],
		output='screen',
		emulate_tty=True,
	)
    # Odom to Pose2D Node
    odom2pose_node = Node(
		package="odom2pose",
		executable="odom2pose",
		parameters=[
			{"input_topic": "gz_pose"},
			{"output_topic": "rosbot_pose"},
		],
		# output='screen',
		emulate_tty=True,
	)
    return LaunchDescription([
		rwander_node, 
        odom2pose_node
	])
    
