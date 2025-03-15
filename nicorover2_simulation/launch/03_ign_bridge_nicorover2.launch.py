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
        default_value = 'nicorover2',
        description   = 'spawn model name'
    ),
]


# generate launch description
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)

    # parameter bridge    
    _node = Node(
        package    = 'ros_ign_bridge',
        executable = 'parameter_bridge',
        name       = 'bridge_node',
        arguments  = [
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/nicorover2/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/nicorover2/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/world/myroom/model/nicorover2/model/delta2g/link/link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        ],
        remappings = [
            ( '/cmd_vel',                                                                 '/cmd_vel' ),
            ( '/model/nicorover2/tf',                                                     '/tf'      ),
            ( '/model/nicorover2/odometry',                                               '/odom'    ),
            ( '/world/myroom/model/nicorover2/model/delta2g/link/link/sensor/lidar/scan', '/scan'    ),
        ],
        output     = 'screen',
        parameters = [
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
    )
    ld.add_action(_node)

    # state
    _path = PathJoinSubstitution([
        get_package_share_directory('nicorover2_simulation'),
        'models',
        'nicorover2',
        'model.urdf'
    ])    
    _node = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        arguments  = [ _path ],
        output     = 'screen'
    )
    ld.add_action(_node)

    return ld

