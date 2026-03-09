#!/usr/bin/env python3
"""
Launch RTAB-Map for 3D mapping with the simulated RealSense D435.
Run this AFTER spawn_robot.launch.py is running.

Usage:
  ros2 launch 3d_mapping mapping.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('3d_mapping'), 'config', 'mapping.rviz'
    ])

    # ── RTAB-Map SLAM node ────────────────────────────────────────────
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # Topic subscriptions
            'subscribe_depth':  True,
            'subscribe_rgb':    True,
            'subscribe_odom':   True,   # use /odom topic directly, not TF lookup
            'subscribe_scan':   False,

            # Sync — odom arrives at 50Hz, camera at 4Hz, use larger interval
            'approx_sync':              True,
            'approx_sync_max_interval': 0.5,
            'sync_queue_size':          50,
            'topic_queue_size':         50,

            # Frames
            'frame_id':       'base_footprint',
            'odom_frame_id':  'odom',

            # TF — only needed for static transforms (camera→base), not odom
            'wait_for_transform': 0.3,

            # RTAB-Map parameters
            'Rtabmap/DetectionRate':        '3.0',
            'RGBD/NeighborLinkRefining':    'true',
            'RGBD/ProximityBySpace':        'true',
            'RGBD/AngularUpdate':           '0.05',
            'RGBD/LinearUpdate':            '0.05',
            'RGBD/OptimizeMaxError':        '3.0',
            'Reg/Strategy':                 '0',
            'Reg/Force3DoF':                'true',
            'Vis/MinInliers':               '5',
            'Vis/FeatureType':              '6',    # ORB
            'Rtabmap/TimeThr':              '0.0',
            'Mem/RehearsalSimilarity':      '0.30',
            'Grid/FromDepth':               'true',
            'Grid/RangeMax':                '5.0',
            'Grid/3D':                      'true',
            'Grid/RayTracing':              'true',
            'GridGlobal/MinSize':           '20.0',
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom',            '/odom'),
        ],
        arguments=['--delete_db_on_start'],
    )

    # ── RTAB-Map Viz ──────────────────────────────────────────────────
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time':             use_sim_time,
            'subscribe_depth':          True,
            'subscribe_rgb':            True,
            'subscribe_odom':           True,   # use /odom topic directly
            'subscribe_scan':           False,
            'approx_sync':              True,
            'approx_sync_max_interval': 0.5,
            'sync_queue_size':          50,
            'topic_queue_size':         50,
            'frame_id':                 'base_footprint',
            'odom_frame_id':            'odom',
        }],
        remappings=[
            ('rgb/image',       '/camera/color/image_raw'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom',            '/odom'),
        ],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),

        # RViz starts immediately — no dependency on TF timing
        rviz_node,

        # Delay RTAB-Map by 15s to ensure:
        #   - TF tree is fully connected (odom→base_footprint publishing)
        #   - Camera bridge warmup has discarded old buffered frames
        #   - /odom topic is publishing steadily
        # Without this delay, RTAB-Map receives camera frames stamped at t=~5s
        # but TF buffer only starts at t=~15s → "extrapolation into the past"
        TimerAction(period=5.0, actions=[
            rtabmap_slam,
            # rtabmap_viz,
        ]),
    ])