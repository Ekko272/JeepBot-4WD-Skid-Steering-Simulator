"""Gazebo + ROS bridge. Publish /jeepbot/cmd_vel for direct mode."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('jeepbot_sim')
    gz_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(sim, 'worlds', 'jeepbot_flat.sdf')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r ' + world}.items(),
        ),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[
                 '/jeepbot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/jeepbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
             ],
             output='screen'),
    ])
