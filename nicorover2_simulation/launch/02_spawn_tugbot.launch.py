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

    DeclareLaunchArgument(
        'x', 
        default_value = '0.0',
        description   = 'initial pos X'
    ),

    DeclareLaunchArgument(
        'y', 
        default_value = '-1.0',
        description   = 'initial pos Y'
    ),

    DeclareLaunchArgument(
        'z', 
        default_value = '0.0',
        description   = 'initial pos Z'
    ),

    DeclareLaunchArgument(
        'yaw', 
        default_value = '0.0',
        description   = 'Yaw'
    ),    
]


# generate launch description
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)

    _path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'models',
        LaunchConfiguration('name'),
        'model.sdf'
    ])
    _node = Node(
        package    = 'ros_gz_sim',
        executable = 'create',
        arguments  = [
            '-file', _path,
            '-name', LaunchConfiguration('name'),
            '-x',    LaunchConfiguration('x'),
            '-y',    LaunchConfiguration('y'),
            '-z',    LaunchConfiguration('z'),
            '-Y',    LaunchConfiguration('yaw')
        ],
        output='screen',
    )
    ld.add_action(_node)
    
    return ld

