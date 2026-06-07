import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=default_config_file,
            description="Path to mission parameter yaml file"
        ),
        DeclareLaunchArgument(
            "run_id", default_value="p3d_d1c_r2",
            description="Run identifier for log correlation"
        ),
        DeclareLaunchArgument(
            "node_output", default_value="log",
            description="ROS node output mode: log or screen"
        ),

        LogInfo(msg="[D1c-R2] tracer1 shadow bringup"),
        LogInfo(msg="[D1c-R2] safe_idle_no_publish=true, no real motion"),

        # T=0.0s: goto_pose_driver for tracer1
        TimerAction(
            period=0.0,
            actions=[
                LogInfo(msg="[T=0.0s] goto_pose_driver (tracer1)"),
                Node(
                    package=sys_pkg,
                    executable="goto_pose_driver",
                    name="goto_pose_node",
                    namespace="tracer1",
                    output=node_output,
                    parameters=[config_file],
                    ros_arguments=["--log-level", "WARN"],
                ),
            ],
        ),

        # T=0.5s: cmd_watchdog for tracer1 (shadow)
        TimerAction(
            period=0.5,
            actions=[
                LogInfo(msg="[T=0.5s] cmd_watchdog (tracer1, shadow)"),
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
                            "safe_idle_no_publish": True,
                            "enable_execution_mode_output": False,
                        },
                    ],
                    ros_arguments=["--log-level", "WARN"],
                ),
            ],
        ),

        # T=1.0s: cmd_scheduler (natural bridge)
        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg="[T=1.0s] cmd_scheduler (natural bridge)"),
                Node(
                    package=sys_pkg,
                    executable="cmd_scheduler",
                    name="cmd_scheduler",
                    output=node_output,
                    parameters=[
                        config_file,
                        {
                            "robots": ["tracer1"],
                            "safe_idle_no_publish": False,
                            "enable_execution_mode_output": False,
                        },
                    ],
                    ros_arguments=["--log-level", "WARN"],
                ),
            ],
        ),

        # T=1.0s: emergency_stop_publisher
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package=sys_pkg,
                    executable="p3c_emergency_stop_publisher",
                    name="p3c_emergency_stop_publisher",
                    output=node_output,
                    parameters=[{
                        "topic": "/wing_alignment/emergency_stop",
                        "publish_hz": 5.0,
                        "default_state": False,
                        "assert_stop_on_start": False,
                        "shutdown_publish_true": False,
                        "stop_file": "/tmp/p3c_emergency_stop.flag",
                    }],
                    ros_arguments=["--log-level", "WARN"],
                ),
            ],
        ),

        # T=3.0s: mission_coordinator (shadow, tracer1)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="[T=3.0s] mission_coordinator (shadow)"),
                Node(
                    package=sys_pkg,
                    executable="mission_coordinator",
                    name="mission_coordinator",
                    output=node_output,
                    parameters=[
                        config_file,
                        {
                            "workflow": "full",
                            "skip_preflight": False,
                            "slide_align_mode": "direct_only",
                            "start_state": "",
                            "resume_phase": "",
                            "managed_phase_mode": True,
                            "safe_idle_no_publish": True,
                            "enable_execution_mode_output": False,
                        },
                    ],
                    ros_arguments=["--log-level", "INFO"],
                ),
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="[T=8.0s] D1c-R2 bringup complete."),
            ],
        ),
    ])
