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
        DeclareLaunchArgument('timer_period', default_value='0.1'),
        DeclareLaunchArgument('simulation_speedup', default_value='1.0'),
        DeclareLaunchArgument('use_existing', default_value='0'),
        DeclareLaunchArgument('agent_type', default_value='rule_based'),
        DeclareLaunchArgument('is_controlled_ego', default_value='false'),
        DeclareLaunchArgument('vehicle_role', default_value='background'),
        DeclareLaunchArgument('intent_observable', default_value='true'),
        DeclareLaunchArgument('intent_label', default_value=''),
        DeclareLaunchArgument('spawn_event_id', default_value=''),
        DeclareLaunchArgument('reveal_background_intents_to_vla', default_value='false'),
        DeclareLaunchArgument('log_path', default_value=''),
        DeclareLaunchArgument('trace_log_enabled', default_value='true'),
        DeclareLaunchArgument('trace_log_path', default_value=''),
        DeclareLaunchArgument('summary_log_path', default_value=''),
        DeclareLaunchArgument('rl_policy_path', default_value=''),
        DeclareLaunchArgument('qwen_endpoint', default_value=''),
        DeclareLaunchArgument('qwen_model', default_value='Qwen2.5-VL-7B-Instruct'),
        DeclareLaunchArgument('qwen_timeout', default_value='15.0'),
        DeclareLaunchArgument('qwen_decision_period', default_value='3.0'),
        DeclareLaunchArgument('qwen_max_candidate_spots', default_value='8'),
        DeclareLaunchArgument('qwen_periodic_replan', default_value='false'),
        DeclareLaunchArgument('qwen_decision_log_path', default_value=''),
        DeclareLaunchArgument('fleet_coordinator_enabled', default_value='false'),
        DeclareLaunchArgument('fleet_run_id', default_value=''),

        Node(
            package='parksim',
            namespace=['vehicle_', LaunchConfiguration('vehicle_id')],
            executable='vehicle_node.py',
            name='vehicle',
            parameters=[vehicle_params] + global_params + [{
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'spot_index': LaunchConfiguration('spot_index'),
                'timer_period': LaunchConfiguration('timer_period'),
                'simulation_speedup': LaunchConfiguration('simulation_speedup'),
                'use_existing': LaunchConfiguration('use_existing'),
                'agent_type': LaunchConfiguration('agent_type'),
                'is_controlled_ego': LaunchConfiguration('is_controlled_ego'),
                'vehicle_role': LaunchConfiguration('vehicle_role'),
                'intent_observable': LaunchConfiguration('intent_observable'),
                'intent_label': LaunchConfiguration('intent_label'),
                'spawn_event_id': LaunchConfiguration('spawn_event_id'),
                'reveal_background_intents_to_vla': LaunchConfiguration('reveal_background_intents_to_vla'),
                'log_path': LaunchConfiguration('log_path'),
                'trace_log_enabled': LaunchConfiguration('trace_log_enabled'),
                'trace_log_path': LaunchConfiguration('trace_log_path'),
                'summary_log_path': LaunchConfiguration('summary_log_path'),
                'rl_policy_path': LaunchConfiguration('rl_policy_path'),
                'qwen_endpoint': LaunchConfiguration('qwen_endpoint'),
                'qwen_model': LaunchConfiguration('qwen_model'),
                'qwen_timeout': LaunchConfiguration('qwen_timeout'),
                'qwen_decision_period': LaunchConfiguration('qwen_decision_period'),
                'qwen_max_candidate_spots': LaunchConfiguration('qwen_max_candidate_spots'),
                'qwen_periodic_replan': LaunchConfiguration('qwen_periodic_replan'),
                'qwen_decision_log_path': LaunchConfiguration('qwen_decision_log_path'),
                'fleet_coordinator_enabled': LaunchConfiguration('fleet_coordinator_enabled'),
                'fleet_run_id': LaunchConfiguration('fleet_run_id'),
            }],
            output='screen',
            emulate_tty=True
        )
    ])
