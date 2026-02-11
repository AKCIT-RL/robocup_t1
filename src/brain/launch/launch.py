# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def handle_configuration(context, *args, **kwargs):
    vision_config_path = os.path.join(os.path.dirname(__file__), '../../../../vision/share/vision/config')
    vision_config_file = os.path.join(vision_config_path, 'vision.yaml')
    vision_config_local_file = os.path.join(vision_config_path, 'vision_local.yaml')

    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'config.yaml') 
    config_local_file = os.path.join(config_path, 'config_local.yaml') 

    behavior_trees_dir = os.path.join(os.path.dirname(__file__), '../behavior_trees')
    def make_tree_path(name):
        if not name.endswith('.xml'):
            name += '.xml'
        return os.path.join(behavior_trees_dir, name)
    tree = context.perform_substitution(LaunchConfiguration('tree'))
    tree_path = make_tree_path(tree)

    # 这里的 config 覆盖 config_file 中相同字段(如有) 用于在 launch 时快速指定参数, 而不需要频繁修改 config.yaml       //as configuracoes aqui sobrescrevem os campos do config_file (se houver) especificar parametros rapidamente durante o launch, sem precisar modificar frequentemente o config.yaml
    config = {
            # 加载哪个行为树文件                        // qual arquivo de behavior tree carregar?
            "tree_file_path": tree_path,
            "vision_config_path": vision_config_file,
            "vision_config_local_path": vision_config_local_file,
    }

    role = context.perform_substitution(LaunchConfiguration('role'))
    if not role == '':
        config['game.player_role'] = role

    sim = context.perform_substitution(LaunchConfiguration('sim'))
    if sim in ['true', 'True', '1']:
        config['use_sim_time'] = True

    disableLog = context.perform_substitution(LaunchConfiguration('disable_log'))
    if disableLog in ['true', 'True', '1']:
        config['rerunLog.enable_file'] = False
        config['rerunLog.enable_tcp'] = False

    disableCom = context.perform_substitution(LaunchConfiguration('disable_com'))
    if disableCom in ['true', 'True', '1']:
        config['enable_com'] = False

    return [
        Node(
            package ='brain',
            executable='brain_node',
            output='screen',
            parameters=[
                config_file,
                config_local_file,
                config
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        # 需要可以通过 ros2 launch brain launch.py param:=value 形式提供的参数, 需要在这里用 DeclarelaunchArgument 声明, 然后在 handle_configuration 处理           
        #// os parametros que precisam ser fornecidos na forma ros2 launch brain launch.py param:=value, devem ser declarados aqui com DeclareLaunchArgument e depois processados em handle_configuration
        DeclareLaunchArgument(
            'tree', 
            default_value='game.xml',
            description='Specify behavior tree file name. DO NOT include full path, file should be in src/brain/config/behavior_trees'
        ),
        DeclareLaunchArgument(
            'role', 
            default_value='',
            description='如果需要覆盖 config.yaml 中的 game.player_role, 可以在 launch 时指定参数 role:=striker'
            #se precisar sobrescrever o game.player_role no config.yaml, pode especificar o parametro role:=striker durante o launch
        ),
        DeclareLaunchArgument(
            'sim', 
            default_value='false',
            description='是否在仿真中'
            #se esta rodando em simulacao
        ),
        DeclareLaunchArgument(
            'disable_log', 
            default_value='false',
            description='强制禁用在文件中记录日志'
            #forca a desativacao do registro de logs em arquivo
        ),
        DeclareLaunchArgument(
            'disable_com', 
            default_value='false',
            description='强制禁用开启通信'
            #forca a desativacao da comunicacao
        ),
        OpaqueFunction(function=handle_configuration) # 转到 handle_configuration 中继续处理
            #chama a funcao handle_configuration para continuar o processamento
    ])
