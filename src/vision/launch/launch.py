import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def handle_configuration(context, *args, **kwargs):
    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'vision.yaml') 
    config_local_file = os.path.join(config_path, 'vision_local.yaml') 

    show_det = LaunchConfiguration('show_det')
    show_seg = LaunchConfiguration('show_seg')
    save_data = LaunchConfiguration('save_data')
    save_depth = LaunchConfiguration('save_depth')
    offline_mode = LaunchConfiguration('offline_mode')
    save_fps = LaunchConfiguration('save_fps')
    return [
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            arguments=[config_file, config_local_file],
            parameters=[{
                'offline_mode': offline_mode,
                'show_det': show_det,
                'show_seg': show_seg,
                'save_data': save_data,
                'save_depth': save_depth,
                'save_fps': save_fps
            }]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "offline_mode",
            default_value='false',
            description="enable offline model"
        ),
        DeclareLaunchArgument(
            "show_det",
            default_value='false',
            description="Show detection result"
        ),
        DeclareLaunchArgument(
            "show_seg",
            default_value='false',
            description="Show segmentation result"
        ),
        DeclareLaunchArgument(
            "save_data",
            default_value='true',
            description="Save recevied image data"
        ),
        DeclareLaunchArgument(
            "save_depth",
            default_value='true',
            description="Save recevied depth img data"
        ),
        DeclareLaunchArgument(
            "save_fps",
            default_value='2',
            description="Save n frames of data each second"
        ),
        OpaqueFunction(function=handle_configuration)
    ])