from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from parksim.base_node import find_parksim_root, read_ros_params_file, read_yaml_file

import os

parksim_dir = get_package_share_directory('parksim')
project_root = find_parksim_root(parksim_dir)
config_dir = os.path.join(parksim_dir, 'config')

global_params = read_yaml_file(os.path.join(config_dir, 'global_params.yaml'), root=project_root)
vehicle_params = read_ros_params_file(os.path.join(config_dir, 'vehicle.yaml'), node_name='/**', root=project_root)


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PARKSIM_ROOT', project_root),
        DeclareLaunchArgument('vehicle_id', default_value='0'),
        DeclareLaunchArgument('spot_index', default_value='3'),
        DeclareLaunchArgument('use_existing', default_value='0'),
        DeclareLaunchArgument('agent_type', default_value='rule_based'),
        DeclareLaunchArgument('rl_policy_path', default_value=''),
        DeclareLaunchArgument('qwen_endpoint', default_value=''),
        DeclareLaunchArgument('qwen_model', default_value='Qwen2.5-VL-7B-Instruct'),
        DeclareLaunchArgument('qwen_decision_period', default_value='3.0'),
        DeclareLaunchArgument('qwen_max_candidate_spots', default_value='8'),
        DeclareLaunchArgument('qwen_periodic_replan', default_value='false'),

        Node(
            package='parksim',
            namespace=['vehicle_', LaunchConfiguration('vehicle_id')],
            executable='vehicle_node.py',
            name='vehicle',
            parameters=[vehicle_params] + global_params + [{
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'spot_index': LaunchConfiguration('spot_index'),
                'use_existing': LaunchConfiguration('use_existing'),
                'agent_type': LaunchConfiguration('agent_type'),
                'rl_policy_path': LaunchConfiguration('rl_policy_path'),
                'qwen_endpoint': LaunchConfiguration('qwen_endpoint'),
                'qwen_model': LaunchConfiguration('qwen_model'),
                'qwen_decision_period': LaunchConfiguration('qwen_decision_period'),
                'qwen_max_candidate_spots': LaunchConfiguration('qwen_max_candidate_spots'),
                'qwen_periodic_replan': LaunchConfiguration('qwen_periodic_replan'),
            }],
            output='screen',
            emulate_tty=True
        )
    ])
