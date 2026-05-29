#!/usr/bin/env python3
"""Launch file for BehaviorTree Web Monitor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for bt_webmonitor."""
    
    # Declare launch arguments
    webui_port_arg = DeclareLaunchArgument(
        'webui_port',
        default_value='8080',
        description='Port for web UI server'
    )
    
    zmq_host_arg = DeclareLaunchArgument(
        'zmq_host',
        default_value='localhost',
        description='Hostname/IP of brain_node ZMQ server'
    )
    
    zmq_publisher_port_arg = DeclareLaunchArgument(
        'zmq_publisher_port',
        default_value='1666',
        description='ZMQ publisher port (status updates)'
    )
    
    zmq_server_port_arg = DeclareLaunchArgument(
        'zmq_server_port',
        default_value='1667',
        description='ZMQ server port (tree structure)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='WebSocket update rate in Hz'
    )
    
    # Create node
    webmonitor_node = Node(
        package='bt_webmonitor',
        executable='webmonitor',
        name='bt_webmonitor',
        output='screen',
        parameters=[{
            'webui_port': LaunchConfiguration('webui_port'),
            'zmq_host': LaunchConfiguration('zmq_host'),
            'zmq_publisher_port': LaunchConfiguration('zmq_publisher_port'),
            'zmq_server_port': LaunchConfiguration('zmq_server_port'),
            'update_rate': LaunchConfiguration('update_rate'),
        }]
    )
    
    return LaunchDescription([
        webui_port_arg,
        zmq_host_arg,
        zmq_publisher_port_arg,
        zmq_server_port_arg,
        update_rate_arg,
        webmonitor_node,
    ])
