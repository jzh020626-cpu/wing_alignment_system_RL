from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


ROBOTS = ("tracer1", "tracer2", "tracer3")
SOURCE_TOPIC_TEMPLATE = "/fr_validation/{robot}/cmd_vel_stamped_source"
TX_TOPIC_TEMPLATE = "/fr_validation/{robot}/cmd_vel_stamped_tx"


def _tx_nodes(robot: str, comm_scenarios_config, scenario_id, method_id, wrapper_mode, node_output):
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
                    "input_topic": SOURCE_TOPIC_TEMPLATE.format(robot=robot),
                    "output_topic": TX_TOPIC_TEMPLATE.format(robot=robot),
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
                    "wrapper_mode": wrapper_mode,
                    "config_path": comm_scenarios_config,
                    "input_topic": TX_TOPIC_TEMPLATE.format(robot=robot),
                    "output_topic": f"/{robot}/cmd_vel_stamped",
                    "meta_topic": f"/fr_validation/{robot}/cmd_channel_meta",
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    scenario_id = LaunchConfiguration("scenario_id")
    method_id = LaunchConfiguration("method_id")
    wrapper_mode = LaunchConfiguration("wrapper_mode")
    mission_log_dir = LaunchConfiguration("mission_log_dir")
    measurement_run_id = LaunchConfiguration("measurement_run_id")
    operator_id = LaunchConfiguration("operator_id")
    phase_source_config = LaunchConfiguration("phase_source_config")
    comm_scenarios_config = LaunchConfiguration("comm_scenarios_config")
    node_output = LaunchConfiguration("node_output")

    bringup_group = GroupAction(
        actions=[
            SetRemap(src=f"/{robot}/cmd_vel_stamped", dst=SOURCE_TOPIC_TEMPLATE.format(robot=robot))
            for robot in ROBOTS
        ]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("wing_alignment_system"), "launch", "system_bringup.launch.py"])
                ),
                launch_arguments={
                    "start_passive_recorder": "true",
                    "measurement_run_id": measurement_run_id,
                    "measurement_log_dir": mission_log_dir,
                }.items(),
            )
        ]
    )

    validation_nodes = [
        Node(
            package="freshness_real_robot_validation",
            executable="phase_source_bridge",
            name="phase_source_bridge",
            output=node_output,
            parameters=[
                {
                    "config_path": phase_source_config,
                    "run_id": measurement_run_id,
                    "mission_log_dir": mission_log_dir,
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
    ]
    for robot in ROBOTS:
        validation_nodes.extend(_tx_nodes(robot, comm_scenarios_config, scenario_id, method_id, wrapper_mode, node_output))

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_id", default_value="real-main"),
            DeclareLaunchArgument("method_id", default_value="FR-TPO"),
            DeclareLaunchArgument("wrapper_mode", default_value="active"),
            DeclareLaunchArgument("mission_log_dir", default_value="~/.ros/hardware_preliminary"),
            DeclareLaunchArgument("measurement_run_id", default_value="controlled_validation"),
            DeclareLaunchArgument("operator_id", default_value="operator"),
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
            DeclareLaunchArgument("node_output", default_value="log"),
            bringup_group,
        ]
        + validation_nodes
    )
