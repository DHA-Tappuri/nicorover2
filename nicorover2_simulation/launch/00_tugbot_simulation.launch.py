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

    # spawn world
    _file_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'launch',
        '01_spawn_world_depot.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _file_path ])
    )
    ld.add_action(_nodes)

    # spawn model    
    _file_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'launch',
        '02_spawn_tugbot.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _file_path ])
    )
    ld.add_action(_nodes)

    # bridge between ROS2 and ign Gazebo
    _file_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'launch',
        '03_ign_bridge_tugbot.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _file_path ])
    )
    ld.add_action(_nodes)

    return ld

