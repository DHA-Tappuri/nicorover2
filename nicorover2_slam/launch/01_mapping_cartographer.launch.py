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
        'scan_topic', 
        default_value = '/scan_front/scan',
        description   = 'scan topic'
    ),
]


# generate launch description
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    
    # global map publisher
    _file_path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_slam'),
        'config'
    ])
    _node = Node(
        package    = 'cartographer_ros',
        executable = 'cartographer_node',
        name       = 'cartographer_node',
        output     = 'screen',
        parameters = [{
            'use_sim_time': True,
        }],
        remappings = [
            ('/scan', LaunchConfiguration('scan_topic')),
        ],        
        arguments=[
            '-configuration_directory', _file_path,
            '-configuration_basename',  'mapping_cartographer.lua',
        ],        
    )
    ld.add_action(_node)

    # global localizer    
    _node = Node(
        package    = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        name       = 'cartographer_occupancy_grid_node',
        output     = 'screen',
        parameters = [{
            'use_sim_time' : True
        }],
        remappings = [
            ('/scan', LaunchConfiguration('scan_topic')),
        ],
    )
    ld.add_action(_node)

    return ld

