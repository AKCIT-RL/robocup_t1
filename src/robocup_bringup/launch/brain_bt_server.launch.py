#!/usr/bin/env python3
"""
Launch file for RoboCup BehaviorTree Server (flex_bt_server).
Starts the BT server with RoboCup-specific configurations.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('robocup_bringup')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'param', 'robocup_params.yaml'),
        description='Full path to the ROS2 parameters file for BT server'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    # Get launch configurations
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Create BT server node
    bt_server_node = Node(
        package='flex_bt_server',
        executable='flex_bt_server_bt_server_executor_node',
        name='bt_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Add any topic remappings here if needed
        ],
        # Set working directory to workspace root for relative path resolution
        cwd='/home/booster/booster_ws'
    )
    
    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        bt_server_node
    ])
