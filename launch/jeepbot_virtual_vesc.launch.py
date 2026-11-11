"""Gazebo + ROS bridge + VESC CAN command emitter + virtual VESC emulation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('jeepbot_sim')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(sim, 'launch', 'jeepbot_gazebo.launch.py'))),
        Node(package='jeepbot_sim', executable='sim_skid_controller', output='screen'),
        Node(package='jeepbot_sim', executable='virtual_vescs', output='screen'),
    ])
