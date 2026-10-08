"""
Real robot: our four nodes, on the Raspberry Pi of the Burger.

Usage: ros2 launch carry_my_luggage robot.launch.py [bringup:=true] [record:=true]

By default turtlebot3_bringup is expected to run already; bringup:=true starts it
too (TURTLEBOT3_MODEL and LDS_MODEL must be set).
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NODES = ('leg_detector', 'operator_tracker', 'gesture_detector', 'follow_manager')
BAG_TOPICS = ['/scan', '/odom', '/tf', '/tf_static', '/battery_state', '/legs', '/target',
              '/target_polar', '/follow_state', '/stop_gesture', '/cmd_vel', '/follow_markers']


def generate_launch_description():
    pkg = get_package_share_directory('carry_my_luggage')
    params = [os.path.join(pkg, 'config', 'params.yaml')]
    bag = os.path.join('bags', datetime.now().strftime('robot_%Y-%m-%d_%H-%M-%S'))
    bringup = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_bringup'), 'launch', 'robot.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='false',
                              description='Lancer aussi turtlebot3_bringup'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Enregistrer un rosbag dans bags/'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup),
                                 condition=IfCondition(LaunchConfiguration('bringup'))),
        *[Node(package='carry_my_luggage', executable=name, name=name, output='screen',
               parameters=params) for name in NODES],
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag, *BAG_TOPICS],
                       output='screen', condition=IfCondition(LaunchConfiguration('record'))),
    ])
