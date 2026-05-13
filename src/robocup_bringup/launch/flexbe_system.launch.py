#!/usr/bin/env python3
"""
Launch file for RoboCup FlexBE System.
Starts complete FlexBE system (onboard + OCS) using flexbe_full.launch.py.
This ensures proper connection between WebUI and behavior engine for execution.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    flexbe_behaviors_dir = get_package_share_directory('robocup_flexbe_behaviors')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'flexbe_config',
        default_value=os.path.join(flexbe_behaviors_dir, 'config', 'flexbe_config.json'),
        description='Path to FlexBE configuration file'
    )
    
    webui_port_arg = DeclareLaunchArgument(
        'webui_port',
        default_value='8000',
        description='Port for FlexBE WebUI server'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('flexbe_config')
    webui_port = LaunchConfiguration('webui_port')
    
    # Get FlexBE package directories
    flexbe_webui_dir = get_package_share_directory('flexbe_webui')
    flexbe_onboard_dir = get_package_share_directory('flexbe_onboard')
    
    # Include FlexBE OCS (web-based, headless - no Qt client)
    flexbe_ocs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(flexbe_webui_dir, 'launch', 'flexbe_ocs.launch.py')
        ),
        launch_arguments={
            'offline': 'false',  # Online mode - connects to onboard
            'headless': 'true',  # No Qt client - use browser instead
            'use_sim_time': use_sim_time,
            'port': webui_port,
            'config_file': config_file
        }.items()
    )
    
    # Include FlexBE Onboard (behavior engine)
    flexbe_onboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(flexbe_onboard_dir, 'behavior_onboard.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_enabled': 'False',
            'log_folder': '~/.flexbe_logs',
            'log_serialize': 'yaml',
            'log_level': 'INFO',
            'enable_clear_imports': 'False'
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        webui_port_arg,
        flexbe_ocs_launch,
        flexbe_onboard_launch
    ])
