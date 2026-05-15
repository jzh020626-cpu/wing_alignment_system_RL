import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node


def default_config_file(package_name: str) -> str:
    return os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mission_params.yaml',
    )


def create_robot_nodes_staggered(
    *,
    robot_index: int,
    base_time: float,
    sys_pkg: str,
    sen_pkg: str,
    config_file,
    node_output='log',
):
    robot_name = f'tracer{robot_index}'
    huatai_name = f'huatai{robot_index}'

    nodes_actions = []

    nodes_actions.append(
        TimerAction(
            period=base_time + 0.0,
            actions=[
                Node(
                    package=sys_pkg,
                    executable='goto_pose_driver',
                    name='goto_pose_node',
                    namespace=robot_name,
                    output=node_output,
                    parameters=[config_file],
                )
            ],
        )
    )

    nodes_actions.append(
        TimerAction(
            period=base_time + 0.5,
            actions=[
                Node(
                    package=sys_pkg,
                    executable='cmd_watchdog',
                    name='cmd_watchdog',
                    namespace=robot_name,
                    output=node_output,
                    parameters=[config_file],
                    ros_arguments=['--log-level', 'WARN'],
                )
            ],
        )
    )

    nodes_actions.append(
        TimerAction(
            period=base_time + 1.0,
            actions=[
                Node(
                    package=sen_pkg,
                    executable='force_monitor',
                    name=f'force_monitor_{huatai_name}',
                    output=node_output,
                    parameters=[{
                        'topic_force_in': f'/{huatai_name}_force',
                        'topic_stop_out': f'/{huatai_name}/force_contact',
                        'calib_frames': 50,
                        'threshold': 65.0,
                        'release_ratio': 0.5,
                        'latched': True,
                        'trigger_count': 4,
                        'release_count': 4,
                        'stop_pub_hz': 10.0,
                        'use_norm_threshold': True,
                        'threshold_norm': 65.0,
                        'topic_force_filtered_out': f'/{huatai_name}_force_filtered',
                        'filtered_pub_hz': 30.0,
                        'force_timeout_enable': False,
                        'force_timeout_sec': 0.5,
                    }],
                    ros_arguments=['--log-level', 'WARN'],
                )
            ],
        )
    )

    nodes_actions.append(
        TimerAction(
            period=base_time + 1.5,
            actions=[
                Node(
                    package=sen_pkg,
                    executable='qr_delta_publisher',
                    name='qr_delta_publisher',
                    namespace=robot_name,
                    output=node_output,
                    parameters=[{
                        'topic_pose_in': 'object_position',
                        'topic_delta_out': 'wing_alignment/delta',
                        'out_frame_id': 'car_frame',
                        'deadband_m': 0.001,
                        'max_pub_hz': 30.0,
                        'publish_enabled_on_start': True,
                        'publish_mode': 'absolute',
                        'output_stamp_source': 'now',
                        'fallback_to_local_stamp_on_clock_offset': True,
                        'enforce_pose_max_age': False,
                        'pose_max_age_sec': 2.0,
                        'future_stamp_tolerance_sec': 1.0,
                        'clock_offset_warn_sec': 60.0,
                        'max_backward_jump_sec': 0.5,
                        'zero_frames': 8,
                        'zero_std_max_m': 0.010,
                        'zero_max_wait_sec': 4.0,
                        'max_jump_m': 0.80,
                        'acquire_frames': 5,
                        'acquire_std_max_m': 0.030,
                        'reacquire_consistent_frames': 5,
                        'duplicate_position_eps': 1e-5,
                    }],
                    ros_arguments=['--log-level', 'WARN'],
                )
            ],
        )
    )

    return nodes_actions


def create_global_nodes(
    *,
    base_time: float,
    sys_pkg: str,
    config_file,
    workflow_cfg='full',
    skip_preflight_cfg=False,
    slide_align_mode_cfg='direct_only',
    start_state_cfg='',
    resume_phase_cfg='',
    managed_phase_mode_cfg=False,
    node_output='log',
):
    return [
        TimerAction(
            period=base_time,
            actions=[
                Node(
                    package=sys_pkg,
                    executable='cmd_scheduler',
                    name='cmd_scheduler',
                    output=node_output,
                    parameters=[config_file],
                    ros_arguments=['--log-level', 'WARN'],
                )
            ],
        ),
        TimerAction(
            period=base_time + 0.5,
            actions=[
                Node(
                    package=sys_pkg,
                    executable='mission_coordinator',
                    name='mission_coordinator',
                    output=node_output,
                    parameters=[
                        config_file,
                        {
                            'workflow': workflow_cfg,
                            'skip_preflight': skip_preflight_cfg,
                            'start_state': start_state_cfg,
                            'resume_phase': resume_phase_cfg,
                            'slide_align_mode': slide_align_mode_cfg,
                            'managed_phase_mode': managed_phase_mode_cfg,
                        },
                    ],
                )
            ],
        ),
    ]
