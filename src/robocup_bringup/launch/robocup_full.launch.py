#!/usr/bin/env python3
"""
Complete RoboCup system launch file.
Launches BT server, FlexBE system, game controller, vision, and sound systems.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('robocup_bringup')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    # Include BT Server launch
    bt_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'brain_bt_server.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include FlexBE System launch
    flexbe_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'flexbe_system.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Game Controller node (if package exists)
    # Uncomment when ready to integrate with game controller
    # game_controller_node = Node(
    #     package='game_controller',
    #     executable='game_controller_node',
    #     name='game_controller',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        bt_server_launch,
        flexbe_system_launch,
        # game_controller_node,  # Uncomment when ready
    ])
