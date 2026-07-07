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

        Node(
            package='parksim',
            namespace=['vehicle_', LaunchConfiguration('vehicle_id')],
            executable='vehicle_node.py',
            name='vehicle',
            parameters=[vehicle_params] + global_params + [{
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'spot_index': LaunchConfiguration('spot_index'),
                'use_existing': LaunchConfiguration('use_existing'),
            }],
            output='screen',
            emulate_tty=True
        )
    ])
