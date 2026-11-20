"""Gazebo Harmonic + robot + ros2_control + EKF + bridges (+ dynamic obstacles)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# name, spawn x, spawn y  (routes are in amr_gazebo/config/dynamic_obstacles.yaml)
DYNAMIC_OBSTACLES = [('dyn_obstacle_1', -2.0, 0.0), ('dyn_obstacle_2', 5.0, -1.0)]


def generate_launch_description():
    desc = get_package_share_directory('amr_description')
    gz = get_package_share_directory('amr_gazebo')
    bringup = get_package_share_directory('amr_bringup')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    dynamic = LaunchConfiguration('dynamic_obstacles')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(gz, 'worlds', 'warehouse.sdf')),
        DeclareLaunchArgument('headless', default_value='false', description='run gz server only'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # Pin Gazebo transport to loopback. On machines with Docker / VPN / several NICs,
    # gz-transport discovery otherwise picks the wrong interface and the server, GUI,
    # ros_gz_bridge and `create` never find each other. Respects a user-set GZ_IP.
    gz_ip = SetEnvironmentVariable('GZ_IP', os.environ.get('GZ_IP', '127.0.0.1'))

    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(gz, 'models') + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PythonExpression(["'-s --headless-rendering ' if '", headless,
                                          "' == 'true' else ''"]), '-r -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items())

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc, 'urdf', 'amr.urdf.xacro')]), value_type=str)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'amr',
                            '-x', x, '-y', y, '-z', '0.02', '-Y', yaw])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(gz, 'config', 'robot_bridge.yaml'),
                               'use_sim_time': True}])

    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'diff_drive_controller',
                                  '--controller-manager', '/controller_manager'])
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[controllers]))

    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               output='screen',
               parameters=[os.path.join(bringup, 'config', 'ekf.yaml'), {'use_sim_time': True}])

    relay = Node(package='amr_tools', executable='cmd_vel_relay', output='screen',
                 parameters=[{'use_sim_time': True}])

    obstacle_actions = [
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', os.path.join(gz, 'models', 'dynamic_obstacle', 'model.sdf'),
                        '-name', name, '-x', str(ox), '-y', str(oy), '-z', '0.25'])
        for name, ox, oy in DYNAMIC_OBSTACLES]
    obstacle_actions += [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='obstacle_bridge',
             output='screen',
             parameters=[{'config_file': os.path.join(gz, 'config', 'obstacle_bridge.yaml'),
                          'use_sim_time': True}]),
        Node(package='amr_tools', executable='dynamic_obstacle_driver', output='screen',
             parameters=[os.path.join(gz, 'config', 'dynamic_obstacles.yaml')]),
    ]
    obstacles = GroupAction(obstacle_actions, condition=IfCondition(dynamic))

    return LaunchDescription(args + [gz_ip, resource_path, gz_sim, rsp, spawn, bridge,
                                     start_controllers, ekf, relay, obstacles])
