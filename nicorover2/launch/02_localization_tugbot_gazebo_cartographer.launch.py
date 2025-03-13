#!/usr/bin/env python3
# coding: utf-8

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions                import Node
from ament_index_python.packages       import get_package_share_directory


# arguments
ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time', 
        default_value = 'true',
        choices       = ['true', 'false'],
        description   = 'Use sim time'
    ),
    
    DeclareLaunchArgument(
        'name', 
        default_value = 'tugbot',
        description   = 'spawn model name'
    ),
]


# generate launch description
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)

    # (1) + (2) + (3) : launch simulation
    _launch_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'launch',
        '00_tugbot_simulation.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _launch_path ]),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
        }.items()
    )
    ld.add_action(_nodes)


    # (4) launch cartographer localization
    _launch_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_slam'),
        'launch',
        '02_localization_cartographer.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _launch_path ])
    )
    ld.add_action(_nodes)


    # (5) launch navigation2
    _launch_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_navigation'),
        'launch',
        '01_navigation_nav2.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _launch_path ])
    )
    ld.add_action(_nodes)


    # (6) launch rviz
    _launch_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2'),
        'launch',
        '99_visualizer_rviz.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _launch_path ])
    )
    ld.add_action(_nodes)

    
    return ld

