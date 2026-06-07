from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scenario_id = LaunchConfiguration("scenario_id")
    method_id = LaunchConfiguration("method_id")
    measurement_run_id = LaunchConfiguration("measurement_run_id")
    operator_id = LaunchConfiguration("operator_id")
    mission_log_root = LaunchConfiguration("mission_log_root")
    mission_runtime_events_path = LaunchConfiguration("mission_runtime_events_path")
    phase_source_mode = LaunchConfiguration("phase_source_mode")
    fallback_policy = LaunchConfiguration("fallback_policy")
    phase_source_config = LaunchConfiguration("phase_source_config")
    comm_scenarios_config = LaunchConfiguration("comm_scenarios_config")
    observe_only_robot = LaunchConfiguration("observe_only_robot")
    start_observe_only_synthetic_source = LaunchConfiguration("start_observe_only_synthetic_source")
    observe_only_publish_rate_hz = LaunchConfiguration("observe_only_publish_rate_hz")
    observe_only_payload_bytes = LaunchConfiguration("observe_only_payload_bytes")
    wrapper_metadata_log_root = LaunchConfiguration("wrapper_metadata_log_root")
    node_output = LaunchConfiguration("node_output")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_id", default_value="real-nominal"),
            DeclareLaunchArgument("method_id", default_value="FR-TPO"),
            DeclareLaunchArgument("measurement_run_id", default_value="safe_idle_validation"),
            DeclareLaunchArgument("operator_id", default_value="operator"),
            DeclareLaunchArgument("mission_log_root", default_value="~/.ros/mission_bench_logs"),
            DeclareLaunchArgument(
                "mission_runtime_events_path",
                default_value=PathJoinSubstitution([mission_log_root, measurement_run_id, "mission_runtime_events.csv"]),
            ),
            DeclareLaunchArgument("phase_source_mode", default_value="mission_runtime_tail"),
            DeclareLaunchArgument("fallback_policy", default_value="geometry_only_if_explicit"),
            DeclareLaunchArgument(
                "phase_source_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("freshness_real_robot_validation"), "config", "real_system_phase_source.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "comm_scenarios_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("freshness_real_robot_validation"), "config", "real_comm_scenarios.yaml"]
                ),
            ),
            DeclareLaunchArgument("observe_only_robot", default_value="tracer1"),
            DeclareLaunchArgument("start_observe_only_synthetic_source", default_value="false"),
            DeclareLaunchArgument("observe_only_publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("observe_only_payload_bytes", default_value="96"),
            DeclareLaunchArgument(
                "wrapper_metadata_log_root",
                default_value="~/.ros/freshness_real_robot_validation/wrapper_metadata",
            ),
            DeclareLaunchArgument("node_output", default_value="log"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("wing_alignment_system"), "launch", "hardware_preliminary_safe_idle.launch.py"]
                    )
                ),
                launch_arguments={
                    "measurement_run_id": measurement_run_id,
                    "measurement_log_dir": mission_log_root,
                    "run_id": measurement_run_id,
                    "mission_log_root": mission_log_root,
                }.items(),
            ),
            Node(
                package="freshness_real_robot_validation",
                executable="phase_source_bridge",
                name="phase_source_bridge",
                output=node_output,
                parameters=[
                    {
                        "config_path": phase_source_config,
                        "run_id": measurement_run_id,
                        "mission_log_root": mission_log_root,
                        "mission_runtime_events_path": mission_runtime_events_path,
                        "phase_source_mode": phase_source_mode,
                        "fallback_policy": fallback_policy,
                        "operator_id": operator_id,
                    }
                ],
            ),
            Node(
                package="freshness_real_robot_validation",
                executable="shadow_policy_sidecar",
                name="shadow_policy_sidecar",
                output=node_output,
                parameters=[{"derived_phase_topic": "/fr_validation/derived_phase_status"}],
            ),
            Node(
                package="freshness_real_robot_validation",
                executable="cmd_channel_wrapper",
                name="cmd_channel_wrapper",
                output=node_output,
                parameters=[
                    {
                        "robot_name": observe_only_robot,
                        "run_id": measurement_run_id,
                        "metadata_log_root": wrapper_metadata_log_root,
                        "config_path": comm_scenarios_config,
                        "scenario_id": scenario_id,
                        "method_id": method_id,
                        "wrapper_mode": "observe",
                        "publish_passthrough": False,
                    }
                ],
            ),
            Node(
                package="freshness_real_robot_validation",
                executable="observe_only_traffic_source",
                name="observe_only_traffic_source",
                output=node_output,
                condition=IfCondition(start_observe_only_synthetic_source),
                parameters=[
                    {
                        "robot_name": observe_only_robot,
                        "publish_rate_hz": observe_only_publish_rate_hz,
                        "payload_bytes": observe_only_payload_bytes,
                        "scenario_id": scenario_id,
                    }
                ],
            ),
        ]
    )
