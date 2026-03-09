#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Paths ──────────────────────────────────────────────────────────
    pkg_3d_mapping = '3d_mapping'

    xacro_path = PathJoinSubstitution([
        FindPackageShare(pkg_3d_mapping), 'urdf', 'mapping_bot.xacro'
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare(pkg_3d_mapping), 'worlds', 'warehouse.sdf'
    ])

    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_path]),
        value_type=str
    )

    # Convert URDF → SDF at launch time via temp file
    urdf_to_sdf = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'xacro $(ros2 pkg prefix 3d_mapping)/share/3d_mapping/urdf/mapping_bot.xacro '
            '> /tmp/mapping_bot.urdf && '
            'gz sdf -p /tmp/mapping_bot.urdf > /tmp/mapping_bot.sdf'
        ],
        output='screen',
    )

    # ── 1. Launch Gazebo Harmonic ─────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-v 4 -r ', world_path],
            'gz_version': '8',
        }.items(),
    )

    # ── 2. Robot State Publisher ───────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── 3. Spawn robot via gz service ─────────────────────────────────
    spawn_robot = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/world/warehouse/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '30000',
            '--req',
            'sdf_filename: "/tmp/mapping_bot.sdf", '
            'pose: {position: {z: 0.2}}, '
            'name: "mapping_bot"',
        ],
        output='screen',
    )

    # ── 4. Bridge relay ────────────────────────────────────────────────
    # CHANGE: Start immediately (no delay) so the TF buffer (odom→base_footprint)
    # begins filling from t=0. Previously it was delayed inside TimerAction(5s)
    # + waiting for urdf_to_sdf, causing TF to not start until t=~45s.
    # RTAB-Map then couldn't look up TF for camera frames at t=~30s.
    bridge_relay = Node(
        package='3d_mapping',
        executable='gz_bridge_relay',
        name='gz_bridge_relay',
        output='screen',
    )

    # ── 5. Camera bridge ───────────────────────────────────────────────
    # CHANGE: Also start immediately so Gazebo camera topics are discovered
    # early. The warmup logic inside gz_camera_bridge.py (WARMUP_SIM_SECONDS=20)
    # will discard frames until t=20s, so starting early is safe.
    camera_bridge = Node(
        package='3d_mapping',
        executable='gz_camera_bridge',
        name='gz_camera_bridge',
        output='screen',
        parameters=[{
            'gz_topic': 'camera',
            'use_sim_time': True,
        }],
        additional_env={'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION': 'python'},
    )

    # ── 6. URDF→SDF conversion + Robot State Publisher ────────────────
    # These still need a small delay for Gazebo to fully start up
    delayed_urdf_and_rsp = TimerAction(
        period=5.0,
        actions=[urdf_to_sdf, robot_state_publisher],
        # bridge_relay and camera_bridge removed — they start immediately above
    )

    # ── 7. Spawn robot after SDF is ready ─────────────────────────────
    spawn_after_sdf = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_to_sdf,
            on_exit=[
                TimerAction(period=5.0, actions=[spawn_robot]),
            ],
        )
    )

    return LaunchDescription([
        gazebo_launch,

        # Start bridges immediately — TF buffer fills from t=0
        bridge_relay,
        camera_bridge,

        # URDF/RSP need Gazebo to be ready first
        delayed_urdf_and_rsp,
        spawn_after_sdf,
    ])