import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from wing_alignment_system.launch_builders import default_config_file, create_global_nodes


def _create_shadow_robot_nodes_staggered(
    *,
    robot_index: int,
    base_time: float,
    sys_pkg: str,
    sen_pkg: str,
    config_file,
    node_output='log',
    driver_log_level_cfg='WARN',
    run_id_cfg='p3d_shadow',
    log_dir_cfg='~/.ros/fr_tac_p3d_shadow_logs',
):
    """Same as create_robot_nodes_staggered but with shadow (safe_idle_no_publish=true)
    parameters on cmd_watchdog. All other nodes unchanged."""
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
                    ros_arguments=['--log-level', driver_log_level_cfg],
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
                    parameters=[
                        config_file,
                        {
                            'safe_idle_no_publish': True,
                            'enable_execution_mode_output': False,
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
        description='Path to YAML config file',
    )
    config_file = LaunchConfiguration('config_file')

    node_output_arg = DeclareLaunchArgument(
        'node_output', default_value='log',
        description='ROS node output mode',
    )
    node_output = LaunchConfiguration('node_output')

    driver_log_level_arg = DeclareLaunchArgument(
        'driver_log_level', default_value='WARN',
        description='Driver log level',
    )
    driver_log_level = LaunchConfiguration('driver_log_level')

    coordinator_log_level_arg = DeclareLaunchArgument(
        'coordinator_log_level', default_value='WARN',
        description='Coordinator log level',
    )
    coordinator_log_level = LaunchConfiguration('coordinator_log_level')

    start_state_arg = DeclareLaunchArgument(
        'start_state', default_value='INIT',
        description='mission_coordinator start state',
    )
    start_state = LaunchConfiguration('start_state')

    resume_phase_arg = DeclareLaunchArgument(
        'resume_phase', default_value='',
        description='mission_coordinator resume phase',
    )
    resume_phase = LaunchConfiguration('resume_phase')

    skip_preflight_arg = DeclareLaunchArgument(
        'skip_preflight', default_value='false',
        description='Skip preflight checks',
    )
    skip_preflight = LaunchConfiguration('skip_preflight')

    slide_align_mode_arg = DeclareLaunchArgument(
        'slide_align_mode', default_value='sync_all',
        description='Slide alignment mode',
    )
    slide_align_mode = LaunchConfiguration('slide_align_mode')

    run_id_arg = DeclareLaunchArgument(
        'run_id', default_value='p3d_shadow',
        description='Run ID for log directories',
    )
    run_id = LaunchConfiguration('run_id')

    log_dir_arg = DeclareLaunchArgument(
        'log_dir', default_value='~/.ros/fr_tac_p3d_shadow_logs',
        description='Watchdog log directory',
    )
    log_dir = LaunchConfiguration('log_dir')

    start_passive_recorder_arg = DeclareLaunchArgument(
        'start_passive_recorder', default_value='false',
    )
    start_passive_recorder = LaunchConfiguration('start_passive_recorder')

    measurement_log_dir_arg = DeclareLaunchArgument(
        'measurement_log_dir', default_value='~/.ros/fr_tac_p3d_measurement_logs',
    )
    measurement_log_dir = LaunchConfiguration('measurement_log_dir')

    measurement_run_id_arg = DeclareLaunchArgument(
        'measurement_run_id', default_value='p3d_shadow',
    )
    measurement_run_id = LaunchConfiguration('measurement_run_id')

    measurement_robots_arg = DeclareLaunchArgument(
        'measurement_robots', default_value='tracer1,tracer2,tracer3',
    )
    measurement_robots = LaunchConfiguration('measurement_robots')

    measurement_slides_arg = DeclareLaunchArgument(
        'measurement_slides', default_value='huatai1,huatai2,huatai3',
    )
    measurement_slides = LaunchConfiguration('measurement_slides')

    # ---- P3-D0b: mission-aware shadow flag ----
    mission_aware_shadow_arg = DeclareLaunchArgument(
        'mission_aware_shadow', default_value='false',
        description='Enable P3-D0b mission-aware shadow bridge + replay phase source',
    )
    mission_aware_shadow = LaunchConfiguration('mission_aware_shadow')

    # ---- P3-D0b: force exec profile for stress testing ----
    force_exec_profile_arg = DeclareLaunchArgument(
        'force_exec_profile', default_value='none',
        description='Stress execution profile: none, degraded_only, full_sweep',
    )
    force_exec_profile = LaunchConfiguration('force_exec_profile')

    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(node_output_arg)
    ld.add_action(driver_log_level_arg)
    ld.add_action(coordinator_log_level_arg)
    ld.add_action(start_state_arg)
    ld.add_action(resume_phase_arg)
    ld.add_action(skip_preflight_arg)
    ld.add_action(slide_align_mode_arg)
    ld.add_action(run_id_arg)
    ld.add_action(log_dir_arg)
    ld.add_action(start_passive_recorder_arg)
    ld.add_action(measurement_log_dir_arg)
    ld.add_action(measurement_run_id_arg)
    ld.add_action(measurement_robots_arg)
    ld.add_action(measurement_slides_arg)
    ld.add_action(mission_aware_shadow_arg)
    ld.add_action(force_exec_profile_arg)

    ld.add_action(LogInfo(msg=['[P3-D0 Shadow] Config file: ', config_file]))
    ld.add_action(LogInfo(msg='[P3-D0 Shadow] mode: shadow (safe_idle_no_publish=true, enable_execution_mode_output=false)'))

    # ---- P3-D0b: Replay Phase Source at T=0 when mission_aware_shadow=true ----
    ld.add_action(TimerAction(
        period=0.0,
        actions=[
            Node(
                package='wing_alignment_system',
                executable='p3d_replay_phase_source',
                name='p3d_replay_phase_source',
                output=node_output,
                condition=IfCondition(mission_aware_shadow),
                parameters=[{
                    'output_topic': '/fr_validation/derived_phase_status',
                    'run_id': run_id,
                    'replay_speed': 1.0,
                    'replay_loop': False,
                    'publish_rate_hz': 5.0,
                }],
                ros_arguments=['--log-level', 'WARN'],
            ),
            LogInfo(msg='[T=0s] P3-D0b Replay Phase Source started (mission_aware_shadow=true)'),
        ]
    ))

    # ---- P3-D0b: Mission-Aware Shadow Bridge at T=0.5 when mission_aware_shadow=true ----
    ld.add_action(TimerAction(
        period=0.5,
        actions=[
            Node(
                package='wing_alignment_system',
                executable='p3d_mission_aware_shadow_bridge',
                name='p3d_mission_aware_shadow_bridge',
                output=node_output,
                condition=IfCondition(mission_aware_shadow),
                parameters=[{
                    'robots': 'tracer1,tracer2,tracer3',
                    'phase_topic': '/fr_validation/derived_phase_status',
                    'force_exec_profile': force_exec_profile,
                }],
                ros_arguments=['--log-level', 'WARN'],
            ),
            LogInfo(msg='[T=0.5s] P3-D0b Mission-Aware Shadow Bridge started'),
        ]
    ))

    # ---- P3-D0a: Original Shadow Cmd Bridge at T=0 when mission_aware_shadow=false ----
    ld.add_action(TimerAction(
        period=0.0,
        actions=[
            Node(
                package='wing_alignment_system',
                executable='p3d_shadow_cmd_bridge',
                name='p3d_shadow_cmd_bridge',
                output=node_output,
                condition=UnlessCondition(mission_aware_shadow),
                arguments=['--ros-args', '-p', 'robots:=tracer1,tracer2,tracer3'],
                parameters=[{
                    'robots': 'tracer1,tracer2,tracer3',
                }],
                ros_arguments=['--log-level', 'WARN'],
            ),
            LogInfo(msg='[T=0s] P3-D0a Shadow Cmd Bridge started (mission_aware_shadow=false)'),
        ]
    ))

    # TRACER-2 (Leader) at T=0
    ld.add_action(LogInfo(msg="===================================="))
    ld.add_action(LogInfo(msg="[T=0s] Starting TRACER-2 (Leader)..."))
    ld.add_action(LogInfo(msg="===================================="))
    for action in _create_shadow_robot_nodes_staggered(
        robot_index=2,
        base_time=0.0,
        sys_pkg=sys_pkg,
        sen_pkg=sen_pkg,
        config_file=config_file,
        node_output=node_output,
        driver_log_level_cfg=driver_log_level,
        run_id_cfg=run_id,
        log_dir_cfg=log_dir,
    ):
        ld.add_action(action)

    # TRACER-1 at T=4
    ld.add_action(TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="===================================="),
            LogInfo(msg="[T=4s] Starting TRACER-1..."),
            LogInfo(msg="===================================="),
        ]
    ))
    for action in _create_shadow_robot_nodes_staggered(
        robot_index=1,
        base_time=4.0,
        sys_pkg=sys_pkg,
        sen_pkg=sen_pkg,
        config_file=config_file,
        node_output=node_output,
        driver_log_level_cfg=driver_log_level,
        run_id_cfg=run_id,
        log_dir_cfg=log_dir,
    ):
        ld.add_action(action)

    # TRACER-3 at T=8
    ld.add_action(TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="===================================="),
            LogInfo(msg="[T=8s] Starting TRACER-3..."),
            LogInfo(msg="===================================="),
        ]
    ))
    for action in _create_shadow_robot_nodes_staggered(
        robot_index=3,
        base_time=8.0,
        sys_pkg=sys_pkg,
        sen_pkg=sen_pkg,
        config_file=config_file,
        node_output=node_output,
        driver_log_level_cfg=driver_log_level,
        run_id_cfg=run_id,
        log_dir_cfg=log_dir,
    ):
        ld.add_action(action)

    # Global nodes (cmd_scheduler + mission_coordinator) at T=12
    ld.add_action(TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="===================================================="),
            LogInfo(msg="[T=12s] All robots online. Starting MANAGED CONTROLLER..."),
            LogInfo(msg="===================================================="),
        ]
    ))
    for action in create_global_nodes(
        base_time=12.0,
        sys_pkg=sys_pkg,
        config_file=config_file,
        workflow_cfg='full',
        skip_preflight_cfg=skip_preflight,
        slide_align_mode_cfg=slide_align_mode,
        start_state_cfg=start_state,
        resume_phase_cfg=resume_phase,
        managed_phase_mode_cfg=True,
        node_output=node_output,
        coordinator_log_level_cfg=coordinator_log_level,
    ):
        ld.add_action(action)

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
