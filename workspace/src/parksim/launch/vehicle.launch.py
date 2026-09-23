from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
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


def _make_vehicle_node(context):
    """按 launch 参数构建 vehicle 节点（规划组件为空时不覆盖 vehicle.yaml）"""
    cfg = context.launch_configurations
    overrides = {}
    for key in ('map', 'route_planner', 'ref_path_generator', 'maneuver_provider',
                'entry_portal', 'exit_portal'):
        value = cfg.get(key, '') or ''
        if value:
            overrides[key] = value
    def _as_int(key, default):
        try:
            return int(cfg.get(key, str(default)) or str(default))
        except ValueError:
            return default

    vehicle_id = _as_int('vehicle_id', 0)
    spot_index = _as_int('spot_index', 3)
    replay = _as_int('use_existing', 0) != 0

    # 注意：数值参数必须为 int（launch 传入的是字符串；节点按 int 参数声明，字符串参数会被读到默认值）
    param_overrides = {
        'vehicle_id': vehicle_id,
        'spot_index': spot_index,
        # 场景 replay 模式由 simulator 通过 use_existing:=1 传入
        'use_existing_agents': replay,
    }
    if overrides:
        param_overrides.update(overrides)
    return [
        Node(
            package='parksim',
            namespace='vehicle_%d' % vehicle_id,
            executable='vehicle_node.py',
            name='vehicle',
            parameters=[vehicle_params] + global_params + [param_overrides],
            output='screen',
            emulate_tty=True
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PARKSIM_ROOT', project_root),
        DeclareLaunchArgument('vehicle_id', default_value='0'),
        DeclareLaunchArgument('spot_index', default_value='3'),
        DeclareLaunchArgument('use_existing', default_value='0',
                              description='1 = 回放既有智能体（scenario replay 模式），0 = 普通入库/出库'),
        DeclareLaunchArgument('map', default_value='',
                              description='地图预设名（config/maps/<name>.yaml）；空 = 默认 DJI_0012'),
        DeclareLaunchArgument('route_planner', default_value='',
                              description='路由规划器覆盖：astar | dijkstra | via'),
        DeclareLaunchArgument('ref_path_generator', default_value='',
                              description='参考路径生成器覆盖：spline | linear'),
        DeclareLaunchArgument('maneuver_provider', default_value='',
                              description='机动提供者覆盖：offline | online_rs'),
        DeclareLaunchArgument('entry_portal', default_value='',
                              description='入场口（多出入口地图：P1 | P4）；空 = legacy 门顶推导'),
        DeclareLaunchArgument('exit_portal', default_value='',
                              description='离场口（多出入口地图：P1 | P2 | P3）；空 = legacy 门顶推导'),

        OpaqueFunction(function=_make_vehicle_node)
    ])
