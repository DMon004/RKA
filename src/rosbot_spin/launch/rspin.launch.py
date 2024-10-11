from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        return LaunchDescription([
		Node(
		    package="rosbot_spin",
		    executable="rspin",
		    parameters=[
		    {"vel_topic": "cmd_vel"},
		    {"angular_vel": 0.2},
		    ],
		    output='screen',
		    emulate_tty=True,
		)
        ])
    
