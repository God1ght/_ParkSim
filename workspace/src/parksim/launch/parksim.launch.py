import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import LaunchConfigurationEquals
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


def _make_simulator_node(context):
    """按 launch 参数构建 simulator 节点（空值不覆盖配置文件/场景设置）"""
    overrides = {}
    for key in ('scenario', 'map', 'init_mode', 'allocation_method', 'route_planner',
                'ref_path_generator', 'maneuver_provider'):
        value = context.launch_configurations.get(key, '') or ''
        if value:
            overrides[key] = value
    extra_params = [{'launch_overrides': json.dumps(overrides)}] if overrides else []
    return [
        Node(
            package='parksim',
            executable='simulator_node.py',
            name='simulator',
            parameters=[simulator_params] + global_params + extra_params,
            output='screen',
            emulate_tty=True
        )
    ]


def _make_visualizer_node(context):
    # Yccc7: 与 simulator 同约定，把 launch 的 map 透传给可视化器，
    # 使其能加载该地图的 layout_rotated.json（而不是 DLP 默认的 DJI 场景）
    overrides = {}
    value = context.launch_configurations.get("map", "") or ""
    if value:
        overrides["map"] = value
    extra_params = [{"launch_overrides": json.dumps(overrides)}] if overrides else []
    if (context.launch_configurations.get("gui", "true") or "true") != "true":
        return []
    return [
        Node(
            package="parksim",
            executable="visualizer_node.py",
            name="visualizer",
            parameters=[visualizer_params] + global_params + extra_params,
            output="screen"
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch the dearpygui visualizer window (false = headless, e.g. for web visualization)'
        ),
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='场景预设名（config/scenarios/<name>.yaml）或文件路径；空 = 默认 config/scenario.yaml'
        ),
        DeclareLaunchArgument(
            'map', default_value='',
            description='地图预设名（config/maps/<name>.yaml）；空 = 默认 DJI_0012'
        ),
        DeclareLaunchArgument(
            'init_mode', default_value='',
            description='初始化模式覆盖：random | replay | custom（空 = 用场景文件 mode）'
        ),
        DeclareLaunchArgument(
            'allocation_method', default_value='',
            description='泊位分配方法覆盖：random | nearest_entrance | graph_cost | balanced_rows | manual'
        ),
        DeclareLaunchArgument(
            'route_planner', default_value='',
            description='路由规划器覆盖：astar | dijkstra | via'
        ),
        DeclareLaunchArgument(
            'ref_path_generator', default_value='',
            description='参考路径生成器覆盖：spline | linear'
        ),
        DeclareLaunchArgument(
            'maneuver_provider', default_value='',
            description='机动提供者覆盖：offline | online_rs'
        ),

        SetEnvironmentVariable('PARKSIM_ROOT', project_root),

        OpaqueFunction(function=_make_visualizer_node),

        # Delay simulator so that the visualization is ready.
        TimerAction(period=3.0,
            actions=[
                OpaqueFunction(function=_make_simulator_node)
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
