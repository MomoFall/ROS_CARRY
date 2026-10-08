"""
RViz2 on the PC: scan, legs, target, state and safety zone.

Usage: ros2 launch carry_my_luggage rviz.launch.py [use_sim_time:=true]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('carry_my_luggage'), 'rviz',
                          'carry_my_luggage.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
