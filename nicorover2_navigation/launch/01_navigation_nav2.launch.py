#!/usr/bin/env python3
# coding: utf-8

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions              import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions                import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution
from launch_ros.substitutions          import FindPackageShare
from ament_index_python.packages       import get_package_share_directory



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


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    
    ld.add_action(SetRemap( '/global_costmap/scan', '/scan_front/scan' ))
    ld.add_action(SetRemap( '/local_costmap/scan',  '/scan_front/scan' ))
    
    _file_path = PathJoinSubstitution([
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    ])
    _config_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_navigation'),
        'config',
        'navigation_nav2.yaml'
    ])    
    _nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_file_path),
        launch_arguments = [
            ('use_sim_time',    LaunchConfiguration('use_sim_time') ),
            ('namespace',       ''                                  ),
            ('params_file',     _config_path                        ),
            ('use_composition', 'False'                             ),
        ]
    )
    ld.add_action(_nodes)
    
    return ld

