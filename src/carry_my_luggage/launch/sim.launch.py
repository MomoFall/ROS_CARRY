"""
Simulation: Gazebo + TurtleBot3 Burger + simulated operator + our four nodes.

Usage: ros2 launch carry_my_luggage sim.launch.py [operator:=path] [obstacle:=true]
[gui:=false] [rviz:=false] [record:=true]
With operator:=teleop (default), drive the operator from another terminal with
ros2 run carry_my_luggage operator_teleop
obstacle:=true adds a 40 cm box at (obstacle_x, obstacle_y) to test the avoidance.
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = ('leg_detector', 'operator_tracker', 'gesture_detector', 'follow_manager')
BAG_TOPICS = ['/scan', '/odom', '/tf', '/tf_static', '/legs', '/target', '/target_polar',
              '/follow_state', '/stop_gesture', '/cmd_vel', '/follow_markers']


def spawn(entity, sdf_file, x, y, z, condition=None):
    """Spawn a model in Gazebo, waiting up to 120 s for Gazebo to be ready."""
    return Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                arguments=['-entity', entity, '-file', sdf_file, '-x', x, '-y', y, '-z', z,
                           '-timeout', '120'],
                condition=condition)


def generate_launch_description():
    pkg = get_package_share_directory('carry_my_luggage')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    params = [os.path.join(pkg, 'config', 'params.yaml'),
              os.path.join(pkg, 'config', 'params_sim.yaml'),
              {'use_sim_time': True}]
    bag = os.path.join('bags', datetime.now().strftime('sim_%Y-%m-%d_%H-%M-%S'))

    def include(path, condition=None, **arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=arguments.items(),
                                        condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('operator', default_value='teleop', choices=['teleop', 'path'],
                              description='teleop : opérateur piloté au clavier '
                                          '(ros2 run carry_my_luggage operator_teleop) ; '
                                          'path : parcours scripté de params_sim.yaml'),
        DeclareLaunchArgument('obstacle', default_value='false',
                              description="Ajouter un carton pour tester l'évitement"),
        DeclareLaunchArgument('obstacle_x', default_value='2.0'),
        DeclareLaunchArgument('obstacle_y', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true', description='Client Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Lancer RViz2'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Enregistrer un rosbag dans bags/'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        # No sound card (WSL, VM): OpenAL would block Gazebo ~30 s at startup.
        SetEnvironmentVariable('ALSOFT_DRIVERS', 'null'),

        include(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'),
                world=os.path.join(pkg, 'worlds', 'follow_test.world')),
        include(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'),
                condition=IfCondition(LaunchConfiguration('gui'))),
        include(os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py'),
                use_sim_time='true'),
        spawn('burger', os.path.join(tb3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf'),
              '0.0', '0.0', '0.01'),
        spawn('operator_legs', os.path.join(pkg, 'models', 'operator_legs', 'model.sdf'),
              '0.8', '0.0', '0.0'),
        spawn('obstacle_box', os.path.join(pkg, 'models', 'obstacle_box', 'model.sdf'),
              LaunchConfiguration('obstacle_x'), LaunchConfiguration('obstacle_y'), '0.0',
              condition=IfCondition(LaunchConfiguration('obstacle'))),

        *[Node(package='carry_my_luggage', executable=name, name=name, output='screen',
               parameters=params) for name in NODES],
        Node(package='carry_my_luggage', executable='sim_operator', name='sim_operator',
             output='screen',
             parameters=params + [{'mode': LaunchConfiguration('operator')}]),

        include(os.path.join(pkg, 'launch', 'rviz.launch.py'),
                condition=IfCondition(LaunchConfiguration('rviz')), use_sim_time='true'),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag, *BAG_TOPICS],
                       output='screen', condition=IfCondition(LaunchConfiguration('record'))),
    ])
