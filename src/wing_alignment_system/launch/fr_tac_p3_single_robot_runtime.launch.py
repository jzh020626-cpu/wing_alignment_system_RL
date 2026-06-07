import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sys_pkg = "wing_alignment_system"
    default_config_file = os.path.join(
        get_package_share_directory(sys_pkg),
        "config",
        "mission_params.yaml",
    )

    config_file = LaunchConfiguration("config_file")
    run_id = LaunchConfiguration("run_id")
    node_output = LaunchConfiguration("node_output")
    log_dir = LaunchConfiguration("log_dir")
    safe_idle_no_publish = LaunchConfiguration("safe_idle_no_publish")
    enable_execution_mode_output = LaunchConfiguration("enable_execution_mode_output")
    emergency_stop_file = LaunchConfiguration("emergency_stop_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config_file),
            DeclareLaunchArgument("run_id", default_value="p3c_runtime"),
            DeclareLaunchArgument("node_output", default_value="log"),
            DeclareLaunchArgument("log_dir", default_value="~/.ros/fr_tac_p3c_runtime"),
            DeclareLaunchArgument("safe_idle_no_publish", default_value="true"),
            DeclareLaunchArgument("enable_execution_mode_output", default_value="true"),
            DeclareLaunchArgument("emergency_stop_file", default_value="/tmp/p3c_emergency_stop.flag"),
            Node(
                package=sys_pkg,
                executable="cmd_watchdog",
                name="cmd_watchdog",
                namespace="tracer1",
                output=node_output,
                parameters=[
                    config_file,
                    {
                        "robot_name": "tracer1",
                        "run_id": run_id,
                        "log_dir": log_dir,
                        "safe_idle_no_publish": ParameterValue(safe_idle_no_publish, value_type=bool),
                        "enable_execution_mode_output": ParameterValue(enable_execution_mode_output, value_type=bool),
                    },
                ],
                ros_arguments=["--log-level", "WARN"],
            ),
            Node(
                package=sys_pkg,
                executable="p3c_emergency_stop_publisher",
                name="p3c_emergency_stop_publisher",
                output=node_output,
                parameters=[
                    {
                        "topic": "/wing_alignment/emergency_stop",
                        "default_state": False,
                        "assert_stop_on_start": False,
                        "shutdown_publish_true": True,
                        "stop_file": emergency_stop_file,
                    }
                ],
                ros_arguments=["--log-level", "WARN"],
            ),
        ]
    )
