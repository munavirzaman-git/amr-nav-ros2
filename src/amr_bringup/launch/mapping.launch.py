"""Teleop mapping: simulation (static world) + SLAM Toolbox + RViz.

Drive with:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
Save with:   ros2 run amr_bringup save_map.sh ~/maps/warehouse_slam
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('amr_bringup')
    launch_dir = os.path.join(bringup, 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={'dynamic_obstacles': 'false',
                              'headless': LaunchConfiguration('headless')}.items()),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')))]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(bringup, 'rviz', 'navigation.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
