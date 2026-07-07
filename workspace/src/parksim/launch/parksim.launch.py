from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from parksim.base_node import find_parksim_root, read_ros_params_file, read_yaml_file

import os

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

parksim_dir = get_package_share_directory('parksim')
project_root = find_parksim_root(parksim_dir)
config_dir = os.path.join(parksim_dir, 'config')

global_params = read_yaml_file(os.path.join(config_dir, 'global_params.yaml'), root=project_root)
visualizer_params = read_ros_params_file(os.path.join(config_dir, 'visualization.yaml'), node_name='visualizer', root=project_root)
simulator_params = read_ros_params_file(os.path.join(config_dir, 'simulator.yaml'), node_name='/simulator', root=project_root)


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PARKSIM_ROOT', project_root),

        Node(
            package='parksim',
            executable='visualizer_node.py',
            name='visualizer',
            parameters=[visualizer_params] + global_params,
            output='screen'
        ),

        # Delay simulator so that the visualization is ready.
        TimerAction(period=3.0,
            actions=[
                Node(
                    package='parksim',
                    executable='simulator_node.py',
                    name='simulator',
                    parameters=[simulator_params] + global_params,
                    output='screen',
                    emulate_tty=True
                )
            ])
    ])


if __name__ == '__main__':
    import launch
    import asyncio

    async def main():
        ld = generate_launch_description()
        launch_service = launch.LaunchService()
        launch_service.include_launch_description(ld)
        return await launch_service.run_async()

    asyncio.run(main())
