"""map_server + AMCL + lifecycle manager."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_params_util import resolve_params  # noqa: E402


def _setup(context):
    params = resolve_params(LaunchConfiguration('params_file').perform(context),
                            map_yaml=LaunchConfiguration('map').perform(context))
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(bringup, 'maps', 'warehouse.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(bringup, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=_setup),
    ])
