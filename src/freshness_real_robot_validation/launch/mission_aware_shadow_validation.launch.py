from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


ROBOTS = ("tracer1", "tracer2", "tracer3")
SOURCE_TOPIC_TEMPLATE = "/fr_validation/{robot}/cmd_vel_stamped_source"
TX_TOPIC_TEMPLATE = "/fr_validation/{robot}/cmd_vel_stamped_tx"


def _tx_nodes(
    *,
    robot: str,
    scenario_id,
    method_id,
    enable_execution_mode,
    comm_scenarios_config,
    wrapper_metadata_log_root,
    measurement_run_id,
    node_output,
):
    return [
        Node(
            package="freshness_real_robot_validation",
            executable="tx_policy_sidecar",
            name=f"{robot}_tx_policy_sidecar",
            output=node_output,
            parameters=[
                {
                    "robot_name": robot,
                    "method_id": method_id,
                    "enable_execution_mode": enable_execution_mode,
                    "input_topic": SOURCE_TOPIC_TEMPLATE.format(robot=robot),
                    "output_topic": TX_TOPIC_TEMPLATE.format(robot=robot),
                    "derived_phase_topic": "/fr_validation/derived_phase_status",
                }
            ],
        ),
        Node(
            package="freshness_real_robot_validation",
            executable="cmd_channel_wrapper",
            name=f"{robot}_cmd_channel_wrapper",
            output=node_output,
            parameters=[
                {
                    "robot_name": robot,
                    "scenario_id": scenario_id,
                    "method_id": method_id,
                    "run_id": measurement_run_id,
                    "metadata_log_root": wrapper_metadata_log_root,
                    "config_path": comm_scenarios_config,
                    "wrapper_mode": "active",
                    "publish_passthrough": True,
                    "input_topic": TX_TOPIC_TEMPLATE.format(robot=robot),
                    "output_topic": f"/{robot}/cmd_vel_stamped",
                    "meta_topic": f"/fr_validation/{robot}/cmd_channel_meta",
                }
            ],
        ),
    ]


def _watchdog_only_nodes(*, robot: str, config_file, measurement_run_id, cmd_safety_log_root, node_output):
    return [
        Node(
            package="wing_alignment_system",
            executable="cmd_watchdog",
            name="cmd_watchdog",
            namespace=robot,
            output=node_output,
            parameters=[
                config_file,
                {
                    "run_id": measurement_run_id,
                    "log_dir": cmd_safety_log_root,
                    "safe_idle_no_publish": True,
                    "enable_execution_mode_output": False,
                },
            ],
            ros_arguments=["--log-level", "WARN"],
        )
    ]


def _replay_source_nodes(
    *,
    robot: str,
    scenario_id,
    observe_only_publish_rate_hz,
    observe_only_payload_bytes,
    node_output,
):
    return [
        Node(
            package="freshness_real_robot_validation",
            executable="observe_only_traffic_source",
            name=f"{robot}_observe_only_traffic_source",
            output=node_output,
            parameters=[
                {
                    "robot_name": robot,
                    "output_topic": SOURCE_TOPIC_TEMPLATE.format(robot=robot),
                    "publish_rate_hz": observe_only_publish_rate_hz,
                    "payload_bytes": observe_only_payload_bytes,
                    "scenario_id": scenario_id,
                    "method_id": "synthetic_shadow_source",
                    "transmission_mode": "synthetic_heartbeat",
                    "task_phase": "standby",
                    "task_progress": 0.0,
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    validation_mode = LaunchConfiguration("validation_mode")
    scenario_id = LaunchConfiguration("scenario_id")
    method_id = LaunchConfiguration("method_id")
    enable_execution_mode = LaunchConfiguration("enable_execution_mode")
    measurement_run_id = LaunchConfiguration("measurement_run_id")
    operator_id = LaunchConfiguration("operator_id")
    config_file = LaunchConfiguration("config_file")
    cmd_safety_log_root = LaunchConfiguration("cmd_safety_log_root")
    mission_log_root = LaunchConfiguration("mission_log_root")
    mission_runtime_events_path = LaunchConfiguration("mission_runtime_events_path")
    phase_source_mode = LaunchConfiguration("phase_source_mode")
    replay_speed = LaunchConfiguration("replay_speed")
    fallback_policy = LaunchConfiguration("fallback_policy")
    phase_source_config = LaunchConfiguration("phase_source_config")
    comm_scenarios_config = LaunchConfiguration("comm_scenarios_config")
    observe_only_publish_rate_hz = LaunchConfiguration("observe_only_publish_rate_hz")
    observe_only_payload_bytes = LaunchConfiguration("observe_only_payload_bytes")
    wrapper_metadata_log_root = LaunchConfiguration("wrapper_metadata_log_root")
    node_output = LaunchConfiguration("node_output")

    mission_safe_idle = GroupAction(
        condition=IfCondition(PythonExpression(["'", validation_mode, "' == 'mission_safe_idle'"])),
        actions=[
            SetRemap(src=f"/{robot}/cmd_vel_stamped", dst=SOURCE_TOPIC_TEMPLATE.format(robot=robot))
            for robot in ROBOTS
        ]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("wing_alignment_system"), "launch", "hardware_preliminary_safe_idle.launch.py"]
                    )
                ),
                launch_arguments={
                    "config_file": config_file,
                    "measurement_run_id": measurement_run_id,
                    "measurement_log_dir": mission_log_root,
                    "run_id": measurement_run_id,
                    "mission_log_root": mission_log_root,
                    "cmd_safety_log_root": cmd_safety_log_root,
                    "enable_execution_mode_output": "false",
                }.items(),
            )
        ],
    )

    replay_watchdogs = []
    for robot in ROBOTS:
        replay_watchdogs.extend(
            _watchdog_only_nodes(
                robot=robot,
                config_file=config_file,
                measurement_run_id=measurement_run_id,
                cmd_safety_log_root=cmd_safety_log_root,
                node_output=node_output,
            )
        )
    replay_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", validation_mode, "' == 'runtime_replay' or '", validation_mode, "' == 'mission_safe_idle'"])),
        actions=replay_watchdogs,
    )

    core_validation_nodes = [
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
                    "replay_speed": replay_speed,
                    "fallback_policy": fallback_policy,
                    "operator_id": operator_id,
                    "enable_comm_proxy": True,
                    "scenario_id": scenario_id,
                    "comm_scenarios_config": comm_scenarios_config,
                }
            ],
        ),
        Node(
            package="freshness_real_robot_validation",
            executable="shadow_policy_sidecar",
            name="shadow_policy_sidecar",
            output=node_output,
            parameters=[
                {
                    "derived_phase_topic": "/fr_validation/derived_phase_status",
                    "enable_execution_mode": enable_execution_mode,
                }
            ],
        ),
    ]
    tx_wrapper_nodes = []
    replay_source_nodes = []
    for robot in ROBOTS:
        tx_wrapper_nodes.extend(
            _tx_nodes(
                robot=robot,
                scenario_id=scenario_id,
                method_id=method_id,
                enable_execution_mode=enable_execution_mode,
                comm_scenarios_config=comm_scenarios_config,
                wrapper_metadata_log_root=wrapper_metadata_log_root,
                measurement_run_id=measurement_run_id,
                node_output=node_output,
            )
        )
        replay_source_nodes.extend(
            _replay_source_nodes(
                robot=robot,
                scenario_id=scenario_id,
                observe_only_publish_rate_hz=observe_only_publish_rate_hz,
                observe_only_payload_bytes=observe_only_payload_bytes,
                node_output=node_output,
            )
        )

    replay_source_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", validation_mode, "' == 'runtime_replay' or '", validation_mode, "' == 'mission_safe_idle'"])),
        actions=replay_source_nodes,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("validation_mode", default_value="runtime_replay"),
            DeclareLaunchArgument("scenario_id", default_value="real-main"),
            DeclareLaunchArgument("method_id", default_value="freshness_aware_tx"),
            DeclareLaunchArgument("enable_execution_mode", default_value="true"),
            DeclareLaunchArgument("measurement_run_id", default_value="fr_tac_p2_shadow"),
            DeclareLaunchArgument("operator_id", default_value="operator"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution([FindPackageShare("wing_alignment_system"), "config", "mission_params.yaml"]),
            ),
            DeclareLaunchArgument("cmd_safety_log_root", default_value="~/.ros/fr_tac_p2_shadow/cmd_safety"),
            DeclareLaunchArgument("mission_log_root", default_value="~/.ros/fr_tac_p2_shadow/mission"),
            DeclareLaunchArgument(
                "mission_runtime_events_path",
                default_value=PathJoinSubstitution([mission_log_root, measurement_run_id, "mission_runtime_events.csv"]),
            ),
            DeclareLaunchArgument("phase_source_mode", default_value="mission_runtime_tail"),
            DeclareLaunchArgument("replay_speed", default_value="5.0"),
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
            DeclareLaunchArgument("observe_only_publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("observe_only_payload_bytes", default_value="96"),
            DeclareLaunchArgument(
                "wrapper_metadata_log_root",
                default_value="~/.ros/fr_tac_p2_shadow/wrapper_metadata",
            ),
            DeclareLaunchArgument("node_output", default_value="log"),
            mission_safe_idle,
            replay_group,
        ]
        + core_validation_nodes
        + tx_wrapper_nodes
        + [replay_source_group]
    )
