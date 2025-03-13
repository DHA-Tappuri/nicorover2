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

    # parameter bridge    
    _node = Node(
        package    = 'ros_ign_bridge',
        executable = 'parameter_bridge',
        name       = 'bridge_node',
        arguments  = [
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/model/tugbot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/tugbot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/tugbot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/world_demo/model/tugbot/link/camera_back/sensor/color/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/world/world_demo/model/tugbot/link/scan_back/sensor/scan_back/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'
        ],
        remappings = [
            ( '/model/tugbot/cmd_vel',                                                      '/cmd_vel' ),
            ( '/model/tugbot/tf',                                                           '/tf'      ),
            ( '/model/tugbot/odometry',                                                     '/odom'    ),
            ( '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu',                '/imu'     ),
            ( '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image',        '/camera_front/image_raw' ),
            ( '/world/world_demo/model/tugbot/link/camera_back/sensor/color/image',         '/camera_back/image_raw'  ),
            ( '/world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan',      '/scan_front/scan'  ),
            ( '/world/world_demo/model/tugbot/link/scan_back/sensor/scan_back/scan',        '/scan_back/scan'   ),
            ( '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points', '/scan_omni/points' ),
        ],
        output     = 'screen',
        parameters = [
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
    )
    ld.add_action(_node)

    # parameter bridge
    _node = Node(
        package    = 'pointcloud_to_laserscan',
        executable = 'pointcloud_to_laserscan_node',
        name       = 'pointcloud_to_laserscan',
        remappings = [
            ('cloud_in', '/scan_omni/points'),
            ('scan',     '/scan_omni/scan'  )
        ],
        parameters=[{
            'target_frame'        : 'tugbot/scan_omni/scan_omni',
            'transform_tolerance' : 0.01,
            'min_height'          : -0.2,
            'max_height'          : 0.2,
            'angle_min'           : -3.1415, # -M_PI/2
            'angle_max'           : 3.1415,  # M_PI/2
            'angle_increment'     : 0.0087,  # M_PI/360.0
            'scan_time'           : 0.3333,
            'range_min'           : 0.45,
            'range_max'           : 30.0,
            'use_inf'             : True,
            'inf_epsilon'         : 1.0
        }],
        output     = 'screen',
    )
    ld.add_action(_node)

    return ld

