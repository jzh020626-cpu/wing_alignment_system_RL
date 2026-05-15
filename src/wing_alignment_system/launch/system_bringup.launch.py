import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from wing_alignment_system.launch_builders import create_global_nodes, create_robot_nodes_staggered


def generate_launch_description():
    sys_pkg = 'wing_alignment_system'
    sen_pkg = 'wing_alignment_sensing'

    default_config_file = os.path.join(
        get_package_share_directory(sys_pkg),
        'config',
        'mission_params.yaml'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to mission parameter yaml file'
    )
    config_file = LaunchConfiguration('config_file')

    start_state_arg = DeclareLaunchArgument(
        'start_state',
        default_value='',
        description='Optional advanced mission start state override'
    )
    start_state = LaunchConfiguration('start_state')

    resume_phase_arg = DeclareLaunchArgument(
        'resume_phase',
        default_value='',
        description='Optional high-level mission resume phase override'
    )
    resume_phase = LaunchConfiguration('resume_phase')

    skip_preflight_arg = DeclareLaunchArgument(
        'skip_preflight',
        default_value='false',
        description='Optional advanced preflight bypass for debug flows'
    )
    skip_preflight = LaunchConfiguration('skip_preflight')

    slide_align_mode_arg = DeclareLaunchArgument(
        'slide_align_mode',
        default_value='direct_only',
        description='Slide alignment mode override'
    )
    slide_align_mode = LaunchConfiguration('slide_align_mode')

    start_trace_logger_arg = DeclareLaunchArgument(
        'start_trace_logger',
        default_value='false',
        description='If true, include wing_mechanism_bench trace logger alongside bringup.'
    )
    start_trace_logger = LaunchConfiguration('start_trace_logger')

    trace_params_file_arg = DeclareLaunchArgument(
        'trace_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wing_mechanism_bench'),
            'config',
            'trace_params.yaml',
        ]),
        description='Path to wing_mechanism_bench trace logger parameter yaml file.'
    )
    trace_params_file = LaunchConfiguration('trace_params_file')

    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(start_state_arg)
    ld.add_action(resume_phase_arg)
    ld.add_action(skip_preflight_arg)
    ld.add_action(slide_align_mode_arg)
    ld.add_action(start_trace_logger_arg)
    ld.add_action(trace_params_file_arg)
    ld.add_action(LogInfo(msg=['[CONFIG] mission params file: ', config_file]))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('wing_mechanism_bench'),
                    'launch',
                    'mechanism_trace_min.launch.py',
                ])
            ),
            condition=IfCondition(start_trace_logger),
            launch_arguments={
                'trace_params_file': trace_params_file,
            }.items(),
        )
    )

    ld.add_action(LogInfo(msg="===================================="))
    ld.add_action(LogInfo(msg="[T=0s] Starting TRACER-2 (Leader)..."))
    ld.add_action(LogInfo(msg="===================================="))
    for action in create_robot_nodes_staggered(
        robot_index=2,
        base_time=0.0,
        sys_pkg=sys_pkg,
        sen_pkg=sen_pkg,
        config_file=config_file,
    ):
        ld.add_action(action)

    ld.add_action(
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg="===================================="),
                LogInfo(msg="[T=4s] Starting TRACER-1..."),
                LogInfo(msg="====================================")
            ]
        )
    )
    for action in create_robot_nodes_staggered(
        robot_index=1,
        base_time=4.0,
        sys_pkg=sys_pkg,
        sen_pkg=sen_pkg,
        config_file=config_file,
    ):
        ld.add_action(action)

    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="===================================="),
                LogInfo(msg="[T=8s] Starting TRACER-3..."),
                LogInfo(msg="====================================")
            ]
        )
    )
    for action in create_robot_nodes_staggered(
        robot_index=3,
        base_time=8.0,
        sys_pkg=sys_pkg,
        sen_pkg=sen_pkg,
        config_file=config_file,
    ):
        ld.add_action(action)

    ld.add_action(
        TimerAction(
            period=12.0,
            actions=[
                LogInfo(msg="===================================================="),
                LogInfo(msg="[T=12s] All robots online. Starting MANAGED CONTROLLER..."),
                LogInfo(msg="====================================================")
            ]
        )
    )
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
    ):
        ld.add_action(action)

    return ld
