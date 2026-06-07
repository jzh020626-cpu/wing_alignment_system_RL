import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from wing_alignment_system.launch_builders import create_global_nodes


def _robot_launch_condition(robot_name, target_robots_cfg, allow_real_motion_cfg, real_motion):
    expected = 'true' if real_motion else 'false'
    return IfCondition(PythonExpression([
        "'", robot_name, "' in '", target_robots_cfg,
        "'.replace(' ', '').split(',') and '",
        allow_real_motion_cfg, "'.lower() == '", expected, "'"
    ]))


def _robot_nodes(
    *,
    robot_index: int,
    base_time: float,
    sys_pkg: str,
    sen_pkg: str,
    config_file,
    node_output='log',
    driver_log_level_cfg='WARN',
    run_id_cfg='p3d_d1_controlled',
    log_dir_cfg='~/.ros/fr_tac_p3d_d1_logs',
    safe_idle_no_publish=True,
    enable_execution_mode_output=False,
    condition=None,
):
    """Robot nodes for P3-D1. condition controls shadow vs real motion."""
    robot_name = f'tracer{robot_index}'
    huatai_name = f'huatai{robot_index}'

    nodes_actions = []

    nodes_actions.append(
        TimerAction(
            period=base_time + 0.0,
            condition=condition,
            actions=[
                Node(
                    package=sys_pkg,
                    executable='goto_pose_driver',
                    name='goto_pose_node',
                    namespace=robot_name,
                    output=node_output,
                    parameters=[config_file],
                    ros_arguments=['--log-level', driver_log_level_cfg],
                )
            ],
        )
    )

    nodes_actions.append(
        TimerAction(
            period=base_time + 0.5,
            condition=condition,
            actions=[
                Node(
                    package=sys_pkg,
                    executable='cmd_watchdog',
                    name='cmd_watchdog',
                    namespace=robot_name,
                    output=node_output,
                    parameters=[
                        config_file,
                        {
                            'safe_idle_no_publish': safe_idle_no_publish,
                            'enable_execution_mode_output': enable_execution_mode_output,
                            'run_id': run_id_cfg,
                            'log_dir': log_dir_cfg,
                        },
                    ],
                    ros_arguments=['--log-level', 'WARN'],
                )
            ],
        )
    )

    nodes_actions.append(
        TimerAction(
            period=base_time + 1.0,
            condition=condition,
            actions=[
                Node(
                    package=sen_pkg,
                    executable='force_monitor',
                    name=f'force_monitor_{huatai_name}',
                    output=node_output,
                    parameters=[config_file],
                    ros_arguments=['--log-level', 'WARN'],
                )
            ],
        )
    )

    nodes_actions.append(
        TimerAction(
            period=base_time + 1.5,
            condition=condition,
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


def generate_launch_description():
    sys_pkg = 'wing_alignment_system'
    sen_pkg = 'wing_alignment_sensing'

    default_cfg = os.path.join(
        get_package_share_directory(sys_pkg),
        'config',
        'mission_params.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_cfg,
        description='Path to mission parameter yaml'
    )
    driver_log_level_arg = DeclareLaunchArgument(
        'driver_log_level', default_value='WARN',
    )
    coordinator_log_level_arg = DeclareLaunchArgument(
        'coordinator_log_level', default_value='INFO',
    )
    node_output_arg = DeclareLaunchArgument(
        'node_output', default_value='log',
    )
    run_id_arg = DeclareLaunchArgument(
        'run_id', default_value='p3d_d1_controlled',
    )
    log_dir_arg = DeclareLaunchArgument(
        'log_dir', default_value='~/.ros/fr_tac_p3d_d1_logs',
    )
    allow_real_motion_arg = DeclareLaunchArgument(
        'allow_real_motion', default_value='false',
        description='Allow real cmd_vel publishing to robots (true/false)'
    )
    start_passive_recorder_arg = DeclareLaunchArgument(
        'start_passive_recorder', default_value='true',
    )
    measurement_log_dir_arg = DeclareLaunchArgument(
        'measurement_log_dir', default_value='~/.ros/fr_tac_p3d_d1_measurements',
    )
    measurement_run_id_arg = DeclareLaunchArgument(
        'measurement_run_id', default_value='p3d_d1_controlled',
    )
    measurement_robots_arg = DeclareLaunchArgument(
        'measurement_robots', default_value='tracer1,tracer2,tracer3',
    )
    measurement_slides_arg = DeclareLaunchArgument(
        'measurement_slides', default_value='huatai1,huatai2,huatai3',
    )

    target_robots_arg = DeclareLaunchArgument(
        'target_robots', default_value='tracer1,tracer2,tracer3',
        description='Comma-separated list of robots to launch (e.g. tracer1)'
    )
    synthetic_cmd_arg = DeclareLaunchArgument(
        'synthetic_cmd', default_value='false',
    )
    synthetic_v_cmd_arg = DeclareLaunchArgument(
        'synthetic_v_cmd', default_value='0.03',
    )
    synthetic_w_cmd_arg = DeclareLaunchArgument(
        'synthetic_w_cmd', default_value='0.06',
    )
    real_synthetic_cmd_arg = DeclareLaunchArgument(
        'real_synthetic_cmd', default_value='false',
        description='Real-motion synthetic cmd_vel_stamped injection (true/false)'
    )

    config_file = LaunchConfiguration('config_file')
    driver_log_level = LaunchConfiguration('driver_log_level')
    coordinator_log_level = LaunchConfiguration('coordinator_log_level')
    node_output = LaunchConfiguration('node_output')
    run_id = LaunchConfiguration('run_id')
    log_dir = LaunchConfiguration('log_dir')
    allow_real_motion = LaunchConfiguration('allow_real_motion')
    start_passive_recorder = LaunchConfiguration('start_passive_recorder')
    measurement_log_dir = LaunchConfiguration('measurement_log_dir')
    measurement_run_id = LaunchConfiguration('measurement_run_id')
    measurement_robots = LaunchConfiguration('measurement_robots')
    measurement_slides = LaunchConfiguration('measurement_slides')
    synthetic_cmd = LaunchConfiguration('synthetic_cmd')
    synthetic_v_cmd = LaunchConfiguration('synthetic_v_cmd')
    synthetic_w_cmd = LaunchConfiguration('synthetic_w_cmd')
    real_synthetic_cmd = LaunchConfiguration('real_synthetic_cmd')
    target_robots = LaunchConfiguration('target_robots')

    is_real = IfCondition(allow_real_motion)
    is_shadow = UnlessCondition(allow_real_motion)

    ld = LaunchDescription()

    for a in [
        config_file_arg, driver_log_level_arg, coordinator_log_level_arg,
        node_output_arg, run_id_arg, log_dir_arg, allow_real_motion_arg,
        start_passive_recorder_arg, measurement_log_dir_arg,
        measurement_run_id_arg, measurement_robots_arg, measurement_slides_arg,
        synthetic_cmd_arg, synthetic_v_cmd_arg, synthetic_w_cmd_arg,
        real_synthetic_cmd_arg, target_robots_arg,
    ]:
        ld.add_action(a)

    ld.add_action(LogInfo(msg='=========================================================='))
    ld.add_action(LogInfo(msg='FR-TAC-P3-D1 Degraded-Only Controlled Bringup'))
    ld.add_action(LogInfo(msg='=========================================================='))
    ld.add_action(LogInfo(msg=['allow_real_motion: ', allow_real_motion]))
    ld.add_action(LogInfo(msg=['run_id: ', run_id]))
    ld.add_action(LogInfo(msg='Force exec profile: degraded_only (no hold/safe_stop)'))

    # ---- Shared infrastructure (always launched) ----
    # Replay Phase Source at T=0
    ld.add_action(TimerAction(
        period=0.0,
        actions=[
            Node(
                package='wing_alignment_system',
                executable='p3d_replay_phase_source',
                name='p3d_replay_phase_source',
                output=node_output,
                parameters=[{
                    'robots': target_robots,
                    'publish_hz': 20.0,
                }],
                ros_arguments=['--log-level', 'WARN'],
            ),
            LogInfo(msg='[T=0s] P3-D1 Replay Phase Source started (phase_source=replay)'),
        ]
    ))

    # Shadow Bridge with degraded_only profile
    # Task B: suppress bridge in real-synthetic mode to avoid duplicate cmd_vel_stamped publishers
    ld.add_action(TimerAction(
        period=0.5,
        condition=UnlessCondition(real_synthetic_cmd),
        actions=[
            Node(
                package='wing_alignment_system',
                executable='p3d_mission_aware_shadow_bridge',
                name='p3d_mission_aware_shadow_bridge',
                output=node_output,
                parameters=[{
                    'robots': target_robots,
                    'phase_topic': '/fr_validation/derived_phase_status',
                    'force_exec_profile': 'degraded_only',
                }],
                ros_arguments=['--log-level', 'WARN'],
            ),
            LogInfo(msg='[T=0.5s] P3-D1 Mission-Aware Shadow Bridge started (degraded_only)'),
        ]
    ))
    ld.add_action(TimerAction(
        period=0.5,
        condition=IfCondition(real_synthetic_cmd),
        actions=[
            LogInfo(msg='[T=0.5s] P3-D1 Shadow Bridge SKIPPED (real-synthetic mode: synthetic publisher provides cmd_vel_stamped)'),
        ]
    ))

    # Emergency Stop Publisher
    ld.add_action(TimerAction(
        period=1.0,
        actions=[
            Node(
                package='wing_alignment_system',
                executable='p3c_emergency_stop_publisher',
                name='p3c_emergency_stop_publisher',
                output=node_output,
                parameters=[{
                    'topic': '/wing_alignment/emergency_stop',
                    'publish_hz': 5.0,
                    'default_state': False,
                    'assert_stop_on_start': False,
                    'shutdown_publish_true': False,
                    'stop_file': '/tmp/p3c_emergency_stop.flag',
                }],
                ros_arguments=['--log-level', 'WARN'],
            ),
            LogInfo(msg='[T=1.0s] Emergency Stop Publisher started'),
        ]
    ))

    # ---- Robot nodes ----
    robot_configs = [
        (2, 1.0, 'TRACER-2 (Leader)'),
        (1, 5.0, 'TRACER-1'),
        (3, 9.0, 'TRACER-3'),
    ]

    for idx, base_t, label in robot_configs:
        robot_name = f"tracer{idx}"
        real_selected = _robot_launch_condition(robot_name, target_robots, allow_real_motion, True)
        shadow_selected = _robot_launch_condition(robot_name, target_robots, allow_real_motion, False)
        # Real motion variant
        ld.add_action(TimerAction(
            period=base_t,
            condition=real_selected,
            actions=[LogInfo(msg=f'[T={base_t}s] {label} -- CONTROLLED (real motion)')],
        ))
        for action in _robot_nodes(
            robot_index=idx, base_time=base_t,
            sys_pkg=sys_pkg, sen_pkg=sen_pkg, config_file=config_file,
            node_output=node_output, driver_log_level_cfg=driver_log_level,
            run_id_cfg=run_id, log_dir_cfg=log_dir,
            safe_idle_no_publish=False, enable_execution_mode_output=True,
            condition=real_selected,
        ):
            ld.add_action(action)

        # Shadow variant
        ld.add_action(TimerAction(
            period=base_t,
            condition=shadow_selected,
            actions=[LogInfo(msg=f'[T={base_t}s] {label} -- SHADOW (no motion)')],
        ))
        for action in _robot_nodes(
            robot_index=idx, base_time=base_t,
            sys_pkg=sys_pkg, sen_pkg=sen_pkg, config_file=config_file,
            node_output=node_output, driver_log_level_cfg=driver_log_level,
            run_id_cfg=run_id, log_dir_cfg=log_dir,
            safe_idle_no_publish=True, enable_execution_mode_output=False,
            condition=shadow_selected,
        ):
            ld.add_action(action)

    # Global nodes (cmd_scheduler + mission_coordinator) at T=13
    ld.add_action(TimerAction(
        period=13.0,
        actions=[LogInfo(msg='[T=13s] All robots online. Starting MANAGED CONTROLLER...')],
    ))
    # P3-D1: pass shadow-mode safety params to global nodes
    #   - shadow: safe_idle_no_publish=True, enable_execution_mode_output=False
    #   - real:   safe_idle_no_publish=False, enable_execution_mode_output=True
    for action in create_global_nodes(
        base_time=13.0, sys_pkg=sys_pkg, config_file=config_file,
        workflow_cfg='full', skip_preflight_cfg=True,
        slide_align_mode_cfg='direct_only', start_state_cfg='',
        resume_phase_cfg='', managed_phase_mode_cfg=True,
        node_output=node_output, coordinator_log_level_cfg=coordinator_log_level,
        safe_idle_no_publish_cfg=True,
        enable_execution_mode_output_cfg=False,
    ):
        ld.add_action(action)

    # Passive measurement recorder
    ld.add_action(
        Node(
            package='wing_alignment_system',
            executable='passive_measurement_recorder',
            name='passive_measurement_recorder',
            output=node_output,
            condition=IfCondition(start_passive_recorder),
            arguments=[
                '--run-id', measurement_run_id,
                '--require-run-id',
                '--out-dir', measurement_log_dir,
                '--config-file', config_file,
                '--robots', measurement_robots,
                '--slides', measurement_slides,
            ],
        )
    )

    return ld
