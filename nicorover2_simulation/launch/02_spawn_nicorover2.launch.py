#!/usr/bin/env python3
# coding: utf-8

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution
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

    _path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'models',
        'nicorover2.sdf'
    ])
    _node = Node(
        package    = 'ros_gz_sim',
        executable = 'create',
        arguments  = [
            '-file', _path,
            '-name', 'nicorover2',
            '-x',    '2.0',
            '-y',    '7.0',
            '-z',    '0.0',
            '-Y',    '0.0'
        ],
        output='screen',
    )
    ld.add_action(_node)
    
    # state
    _path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'models',
        'nicorover2.urdf'
    ])    
    _node = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        arguments  = [ _path ],
        output     = 'screen'
    )
    ld.add_action(_node)

    return ld

