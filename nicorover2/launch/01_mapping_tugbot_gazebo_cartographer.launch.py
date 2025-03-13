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
        'controller', 
        default_value = 'joystick',
        choices       = ['joystick', 'keyboard'],
        description   = 'control method'
    ),
]


# generate launch description
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)


    # control method
    if( LaunchConfiguration('controller') == 'joystick' ):
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
        
    elif( LaunchConfiguration('controller') == 'keyboard' ):
        pass
        
    else:
        pass


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


    # (4) : launch cartographer mapping
    _launch_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_slam'),
        'launch',
        '01_mapping_cartographer.launch.py'
    ])
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ _launch_path ])
    )
    ld.add_action(_nodes)


    # (6) : launch rviz
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

