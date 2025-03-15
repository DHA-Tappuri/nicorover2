#!/usr/bin/env python3
# coding: utf-8

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution
from ament_index_python.packages       import get_package_share_directory


# arguments
ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time', 
        default_value = 'true',
        choices       = ['true', 'false'],
        description   = 'Use sim time'
    )
]


# generate launch description
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    
    # add resource path
    _path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'models'
    ])            
    _env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', _path)
    ld.add_action(_env)

    # world file
    world_file_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'worlds',
        'myroom',
        'model.sdf'
    ])    

    # ros_gz_sim path
    launch_file_path = PathJoinSubstitution([
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])
    nodes_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ launch_file_path ]),
        launch_arguments=[
            ('gz_args', [ world_file_path, ' -r', ' -v 4' ])
        ]
    )
    ld.add_action(nodes_gz)

    return ld

