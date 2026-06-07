import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _qr_delta_params():
    return {
        "topic_pose_in": "object_position",
        "topic_delta_out": "wing_alignment/delta",
        "out_frame_id": "car_frame",
        "deadband_m": 0.001,
        "max_pub_hz": 30.0,
        "publish_enabled_on_start": True,
        "publish_mode": "absolute",
        "output_stamp_source": "now",
        "fallback_to_local_stamp_on_clock_offset": True,
        "enforce_pose_max_age": False,
        "pose_max_age_sec": 2.0,
        "future_stamp_tolerance_sec": 1.0,
        "clock_offset_warn_sec": 60.0,
        "max_backward_jump_sec": 0.5,
        "zero_frames": 8,
        "zero_std_max_m": 0.010,
        "zero_max_wait_sec": 4.0,
        "max_jump_m": 0.80,
        "acquire_frames": 5,
        "acquire_std_max_m": 0.030,
        "reacquire_consistent_frames": 5,
        "duplicate_position_eps": 1e-5,
    }


def generate_launch_description():
    sys_pkg = "wing_alignment_system"
    sen_pkg = "wing_alignment_sensing"

    default_config_file = os.path.join(
        get_package_share_directory(sys_pkg),
        "config",
        "mission_params.yaml",
    )

    config_file_arg = DeclareLaunchArgument("config_file", default_value=default_config_file, description="Path to mission parameter yaml file")
    node_output_arg = DeclareLaunchArgument("node_output", default_value="log", description="ROS node output mode: log or screen")
    run_id_arg = DeclareLaunchArgument("run_id", default_value="", description="Optional shared run_id for safe-idle mission-chain logs")
    cmd_safety_log_root_arg = DeclareLaunchArgument("cmd_safety_log_root", default_value="~/.ros/cmd_safety_logs", description="Root directory for scheduler/watchdog logs")
    mission_log_root_arg = DeclareLaunchArgument("mission_log_root", default_value="~/.ros/mission_bench_logs", description="Root directory for mission runtime logs")
    coordinator_log_level_arg = DeclareLaunchArgument("coordinator_log_level", default_value="INFO", description="Log level for mission_coordinator")
    enable_execution_mode_output_arg = DeclareLaunchArgument("enable_execution_mode_output", default_value="false", description="If true, let watchdog execution_mode scale or stop cmd_vel output")
    start_passive_recorder_arg = DeclareLaunchArgument("start_passive_recorder", default_value="false", description="If true, start the passive measurement recorder as a read-only sidecar")
    measurement_log_dir_arg = DeclareLaunchArgument("measurement_log_dir", default_value="~/.ros/hardware_preliminary_logs", description="Output directory for passive measurement recorder CSV artifacts")
    measurement_run_id_arg = DeclareLaunchArgument("measurement_run_id", default_value="", description="Optional run_id forwarded to the passive measurement recorder")
    measurement_robots_arg = DeclareLaunchArgument("measurement_robots", default_value="tracer1,tracer2,tracer3", description="Comma-separated robot names for the passive measurement recorder")
    measurement_slides_arg = DeclareLaunchArgument("measurement_slides", default_value="huatai1,huatai2,huatai3", description="Comma-separated slide names for the passive measurement recorder")

    config_file = LaunchConfiguration("config_file")
    node_output = LaunchConfiguration("node_output")
    run_id = LaunchConfiguration("run_id")
    cmd_safety_log_root = LaunchConfiguration("cmd_safety_log_root")
    mission_log_root = LaunchConfiguration("mission_log_root")
    coordinator_log_level = LaunchConfiguration("coordinator_log_level")
    enable_execution_mode_output = LaunchConfiguration("enable_execution_mode_output")
    start_passive_recorder = LaunchConfiguration("start_passive_recorder")
    measurement_log_dir = LaunchConfiguration("measurement_log_dir")
    measurement_run_id = LaunchConfiguration("measurement_run_id")
    measurement_robots = LaunchConfiguration("measurement_robots")
    measurement_slides = LaunchConfiguration("measurement_slides")

    ld = LaunchDescription()
    for action in (
        config_file_arg,
        node_output_arg,
        run_id_arg,
        cmd_safety_log_root_arg,
        mission_log_root_arg,
        coordinator_log_level_arg,
        enable_execution_mode_output_arg,
        start_passive_recorder_arg,
        measurement_log_dir_arg,
        measurement_run_id_arg,
        measurement_robots_arg,
        measurement_slides_arg,
    ):
        ld.add_action(action)

    ld.add_action(LogInfo(msg=["[SAFE_IDLE] config file: ", config_file]))
    ld.add_action(LogInfo(msg=["[SAFE_IDLE] run_id: ", run_id]))
    ld.add_action(LogInfo(msg="[SAFE_IDLE] starting scheduler/watchdog/mission in no-publish instrumentation mode"))

    for idx, robot in enumerate(("tracer1", "tracer2", "tracer3")):
        ld.add_action(
            TimerAction(
                period=0.2 * idx,
                actions=[
                    Node(
                        package=sen_pkg,
                        executable="qr_delta_publisher",
                        name="qr_delta_publisher",
                        namespace=robot,
                        output=node_output,
                        parameters=[_qr_delta_params()],
                    )
                ],
            )
        )
        ld.add_action(
            TimerAction(
                period=0.6 + 0.2 * idx,
                actions=[
                    Node(
                        package=sys_pkg,
                        executable="cmd_watchdog",
                        name="cmd_watchdog",
                        namespace=robot,
                        output=node_output,
                        parameters=[config_file, {
                            "run_id": run_id,
                            "log_dir": cmd_safety_log_root,
                            "safe_idle_no_publish": True,
                            "enable_execution_mode_output": enable_execution_mode_output,
                        }],
                        ros_arguments=["--log-level", "WARN"],
                    )
                ],
            )
        )

    ld.add_action(
        TimerAction(
            period=1.4,
            actions=[
                Node(
                    package=sys_pkg,
                    executable="cmd_scheduler",
                    name="cmd_scheduler",
                    output=node_output,
                    parameters=[config_file, {
                        "run_id": run_id,
                        "log_dir": cmd_safety_log_root,
                        "safe_idle_no_publish": True,
                    }],
                    ros_arguments=["--log-level", "WARN"],
                )
            ],
        )
    )

    ld.add_action(
        TimerAction(
            period=1.8,
            actions=[
                Node(
                    package=sys_pkg,
                    executable="mission_coordinator",
                    name="mission_coordinator",
                    output=node_output,
                    parameters=[config_file, {
                        "run_id": run_id,
                        "mission_log_dir": mission_log_root,
                        "managed_phase_mode": True,
                        "workflow": "full",
                        "start_state": "",
                        "safe_idle_no_publish": True,
                    }],
                    ros_arguments=["--log-level", coordinator_log_level],
                )
            ],
        )
    )

    ld.add_action(
        Node(
            package=sys_pkg,
            executable="passive_measurement_recorder",
            name="passive_measurement_recorder",
            output=node_output,
            condition=IfCondition(start_passive_recorder),
            arguments=[
                "--run-id", measurement_run_id,
                "--require-run-id",
                "--out-dir", measurement_log_dir,
                "--config-file", config_file,
                "--robots", measurement_robots,
                "--slides", measurement_slides,
            ],
        )
    )

    return ld
