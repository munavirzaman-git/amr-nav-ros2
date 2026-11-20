"""Complete system: Gazebo + robot + (SLAM | map_server/AMCL) + Nav2 + RViz.

Examples
  ros2 launch amr_bringup bringup.launch.py                          # A* + DWA, AMCL on prebuilt map
  ros2 launch amr_bringup bringup.launch.py planner:=navfn controller:=mppi
  ros2 launch amr_bringup bringup.launch.py slam:=true                # navigate while mapping
  ros2 launch amr_bringup bringup.launch.py map:=$HOME/maps/warehouse_slam.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('amr_bringup')
    launch_dir = os.path.join(bringup, 'launch')
    lc = LaunchConfiguration

    def include(name, condition=None, **kw):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=kw.items(), condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='false',
                              description='true: SLAM Toolbox instead of map_server + AMCL'),
        DeclareLaunchArgument('map', default_value=os.path.join(bringup, 'maps', 'warehouse.yaml')),
        DeclareLaunchArgument('planner', default_value='astar', description='navfn | smac | astar'),
        DeclareLaunchArgument('controller', default_value='dwa', description='mppi | dwb | dwa'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        include('sim.launch.py', dynamic_obstacles=lc('dynamic_obstacles'),
                headless=lc('headless')),

        # give Gazebo, controllers and the EKF a moment to bring TF up
        TimerAction(period=5.0, actions=[
            include('slam.launch.py', condition=IfCondition(lc('slam'))),
            include('localization.launch.py', condition=UnlessCondition(lc('slam')),
                    map=lc('map')),
        ]),
        TimerAction(period=8.0, actions=[
            include('navigation.launch.py', planner=lc('planner'), controller=lc('controller')),
        ]),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(bringup, 'rviz', 'navigation.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(lc('rviz'))),
    ])
