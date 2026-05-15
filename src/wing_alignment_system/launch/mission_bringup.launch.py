import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'wing_alignment_system'

    default_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'mission_params.yaml'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to mission parameter yaml file'
    )
    config_file = LaunchConfiguration('config_file')
    node_output_arg = DeclareLaunchArgument(
        'node_output',
        default_value='log',
        description='ROS node output mode: log or screen'
    )
    node_output = LaunchConfiguration('node_output')
    enable_return_home_arg = DeclareLaunchArgument(
        'enable_return_home',
        default_value='false',
        description='Enable independent three-tracer return-home coordinator'
    )
    enable_return_home = LaunchConfiguration('enable_return_home')
    enable_mission_coordinator_arg = DeclareLaunchArgument(
        'enable_mission_coordinator',
        default_value='true',
        description='Enable legacy mission_coordinator'
    )
    enable_mission_coordinator = LaunchConfiguration('enable_mission_coordinator')

    # 三台车 driver
    driver1 = Node(
        package=pkg_name,
        executable='goto_pose_driver',
        name='goto_pose_node',
        namespace='tracer1',
        output=node_output,
        parameters=[config_file]
    )

    driver2 = Node(
        package=pkg_name,
        executable='goto_pose_driver',
        name='goto_pose_node',
        namespace='tracer2',
        output=node_output,
        parameters=[config_file]
    )

    driver3 = Node(
        package=pkg_name,
        executable='goto_pose_driver',
        name='goto_pose_node',
        namespace='tracer3',
        output=node_output,
        parameters=[config_file]
    )

    watchdog1 = Node(
        package=pkg_name,
        executable='cmd_watchdog',
        name='cmd_watchdog',
        namespace='tracer1',
        output=node_output,
        parameters=[config_file],
        ros_arguments=['--log-level', 'WARN']
    )

    watchdog2 = Node(
        package=pkg_name,
        executable='cmd_watchdog',
        name='cmd_watchdog',
        namespace='tracer2',
        output=node_output,
        parameters=[config_file],
        ros_arguments=['--log-level', 'WARN']
    )

    watchdog3 = Node(
        package=pkg_name,
        executable='cmd_watchdog',
        name='cmd_watchdog',
        namespace='tracer3',
        output=node_output,
        parameters=[config_file],
        ros_arguments=['--log-level', 'WARN']
    )

    scheduler = Node(
        package=pkg_name,
        executable='cmd_scheduler',
        name='cmd_scheduler',
        output=node_output,
        parameters=[config_file],
        ros_arguments=['--log-level', 'WARN']
    )

    # 一个多车 coordinator（不放 namespace，统一调度）
    coordinator = Node(
        package=pkg_name,
        executable='mission_coordinator',
        name='mission_coordinator',
        output=node_output,
        parameters=[config_file],
        condition=IfCondition(enable_mission_coordinator)
    )

    return_home = Node(
        package=pkg_name,
        executable='multi_tracer_return_home',
        name='multi_tracer_return_home',
        output=node_output,
        parameters=[config_file],
        condition=IfCondition(enable_return_home)
    )

    return LaunchDescription([
        config_file_arg,
        node_output_arg,
        enable_return_home_arg,
        enable_mission_coordinator_arg,
        LogInfo(msg=['[CONFIG] mission params file: ', config_file]),
        driver1, driver2, driver3,
        watchdog1, watchdog2, watchdog3,
        scheduler,
        coordinator,
        return_home
    ])
