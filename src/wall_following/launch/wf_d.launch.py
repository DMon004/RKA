import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration

from launch_ros.actions import Node
import os


def generate_launch_description():
    kp_arg = DeclareLaunchArgument(
        "kp", default_value=TextSubstitution(text="1.5")
    )

    distance_arg = DeclareLaunchArgument(
        "distance", default_value=TextSubstitution(text="1.0")
    )

    wall_following_node = Node(
        package="wall_following",
		executable="wall_follow_d",
        name='wall_follow_d',
		parameters = [
            {"kp": LaunchConfiguration('kp')},
            {"distance": LaunchConfiguration('distance')}
            ],
        output = "screen"
    )

    # Odom to Pose2D Node
    odom2pose_node = Node(
		package="odom2pose",
		executable="odom2pose",
		parameters=[
			{"input_topic": "gz_pose"},
			{"output_topic": "rosbot_pose"},
		],
		output='screen',
		emulate_tty=True,
	)

    return LaunchDescription([
        kp_arg,
        distance_arg, 
        wall_following_node,
        odom2pose_node
    ])