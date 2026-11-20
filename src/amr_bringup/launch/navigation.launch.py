"""Nav2 navigation servers with a selectable global planner and controller.

  planner:=navfn|smac|astar     controller:=mppi|dwb|dwa
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_params_util import resolve_params  # noqa: E402

LIFECYCLE_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                   'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']


def _setup(context):
    planner = LaunchConfiguration('planner').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    params = resolve_params(LaunchConfiguration('params_file').perform(context),
                            planner=planner, controller=controller)
    cmd_nav = [('cmd_vel', 'cmd_vel_nav')]

    def server(pkg, exe, remaps=None):
        return Node(package=pkg, executable=exe, name=exe, output='screen',
                    parameters=[params], remappings=remaps or [])

    return [
        LogInfo(msg=f'[amr_bringup] global planner = {planner}, controller = {controller}'),
        server('nav2_controller', 'controller_server', cmd_nav),
        server('nav2_smoother', 'smoother_server'),
        server('nav2_planner', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', cmd_nav),
        # controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
        #            -> collision_monitor -> cmd_vel -> cmd_vel_relay -> diff_drive_controller
        server('nav2_velocity_smoother', 'velocity_smoother', cmd_nav),
        server('nav2_collision_monitor', 'collision_monitor'),
        server('nav2_bt_navigator', 'bt_navigator'),
        server('nav2_waypoint_follower', 'waypoint_follower'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': LIFECYCLE_NODES}]),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='astar',
                              description='navfn | smac | astar'),
        DeclareLaunchArgument('controller', default_value='dwa',
                              description='mppi | dwb | dwa'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(bringup, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=_setup),
    ])
