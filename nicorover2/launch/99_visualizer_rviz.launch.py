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
    
    # rviz2
    rviz_file_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2'),
        'config',
        'mapping.rviz'
    ])
    _node = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        arguments  = [
            '-d', rviz_file_path
        ],
        output     = 'screen',
        parameters = [
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
    )
    ld.add_action(_node)
    
    return ld

