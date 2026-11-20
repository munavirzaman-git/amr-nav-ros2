"""SLAM Toolbox (online async) with this project's parameters."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('amr_bringup')
    slam = get_package_share_directory('slam_toolbox')
    params = LaunchConfiguration('slam_params_file')
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(bringup, 'config', 'slam_toolbox.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': params, 'use_sim_time': 'true'}.items()),
    ])
