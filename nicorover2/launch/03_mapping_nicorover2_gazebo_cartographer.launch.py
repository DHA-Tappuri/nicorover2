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
    
    _launch_path = PathJoinSubstitution([
        get_package_share_directory('teleop_twist_joy'),
        'launch',
        'teleop-launch.py'
    ])
    _config_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2'),
        'config',
        'f710.config.yaml'
    ])    
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _launch_path ]),
        launch_arguments={
            'config_filepath' : _config_path,
        }.items()        
    )
    ld.add_action(_nodes)
    

    """
    # joystick
    _node = Node(
        package    = 'joy',
        executable = 'joy_node',
        output     = 'screen'
    )
    ld.add_action(_node)
    _node = Node(
        package    = 'teleop_twist_joy',
        executable = 'teleop_node',
        output     = 'screen'
    )
    ld.add_action(_node)
    """
    
    return ld

