#!/usr/bin/env python3
"""
场景初始化（scenario）加载与解析 —— random / replay / custom

用法（simulator 侧）：
    scenario, path = load_scenario(name, config_dir)
    mode = effective_mode(scenario, use_existing_agents)

- name: '' 或 'default' → config_dir/scenario.yaml（默认场景，缺文件时返回空配置=旧行为）
        '<name>'         → config_dir/scenarios/<name>.yaml
        含路径分隔符 / .yaml 后缀 → 直接作为路径（相对路径基于 config_dir）

- effective_mode: 场景文件显式指定 mode 时以其为准；
                  否则回退旧参数映射（use_existing_agents=True→replay / False→random）。

场景文件结构见 config/scenario.yaml 与 config/scenarios/*.yaml 示例。
"""

import os
import re

import logging
import pickle

import yaml

DEFAULT_SCENARIO_NAME = 'default'
VALID_MODES = ('random', 'replay', 'custom')

_log = logging.getLogger(__name__)

# schema v2（jth_b1 新有向图）冒烟校验的必需键。
_SCHEMA_V2_REQUIRED_KEYS = (
    'schema_version', 'graph', 'arcs', 'vertex_index', 'portals',
    'portal_id_by_node', 'entrance_portal_ids', 'exit_portal_ids', 'turns',
    'unreachable_entry_arcs', 'spot_entry_portals', 'spot_targets', 'density',
    'entrance_coords', 'lane_offset_mode', 'lane_half_width_m',
)


def _smoke_check_map_schema(expanded):
    """schema v2 地图的**只 warn 不抛**冒烟校验（DJI/legacy 无 schema_version → 跳过）。

    - 必需键是否齐全；
    - len(spot_targets.by_index) 是否等于 len(spots_data.spot_waypoints)。
    任何异常仅记 warning，绝不打断加载。
    """
    try:
        wgp = expanded.get('waypoints_graph_path')
        if not wgp or not os.path.isfile(wgp):
            return
        with open(wgp, 'rb') as f:
            data = pickle.load(f)
        if not isinstance(data, dict):
            return
        sv = data.get('schema_version')
        if not isinstance(sv, int) or sv < 2:
            return
        missing = [k for k in _SCHEMA_V2_REQUIRED_KEYS if k not in data]
        if missing:
            _log.warning('map schema v2 冒烟：缺少必需键 %s (%s)', missing, wgp)
        st = data.get('spot_targets')
        by_index = st.get('by_index') if isinstance(st, dict) else None
        sdp = expanded.get('spots_data_path')
        n_sw = None
        if sdp and os.path.isfile(sdp):
            try:
                with open(sdp, 'rb') as f:
                    sd = pickle.load(f)
                if isinstance(sd, dict) and sd.get('spot_waypoints') is not None:
                    n_sw = len(sd['spot_waypoints'])
            except Exception as e:  # noqa: BLE001
                _log.warning('map schema v2 冒烟：spots_data 读取失败（%s）', e)
        if by_index is not None and n_sw is not None and len(by_index) != n_sw:
            _log.warning(
                'map schema v2 冒烟：len(spot_targets.by_index)=%d != len(spot_waypoints)=%d (%s)',
                len(by_index), n_sw, wgp)
    except Exception as e:  # noqa: BLE001
        _log.warning('map schema v2 冒烟校验跳过（%s）', e)



class ScenarioError(RuntimeError):
    """场景配置错误（文件缺失 / 格式错误 / 模式非法）"""


def resolve_scenario_file(name, config_dir):
    """把场景名解析为文件路径（不做存在性检查）"""
    name = str(name or '')
    if name in ('', DEFAULT_SCENARIO_NAME):
        # 默认场景文件单独一个路径（与预设库分离）
        return os.path.join(config_dir, 'scenario.yaml')
    if name.endswith('.yaml') or os.sep in name or '/' in name:
        return name if os.path.isabs(name) else os.path.join(config_dir, name)
    return os.path.join(config_dir, 'scenarios', name + '.yaml')


def load_scenario(name, config_dir):
    """
    加载场景配置（同 time_scale 解析）。
    返回 (scenario_dict, source_path)；
    - 默认场景文件缺失 → ({}, None)（兼容旧行为，由 effective_mode 回退）
    - 命名场景缺失 / 文件非法 → 抛 ScenarioError（快速失败，避免静默用错场景）
    """
    path = resolve_scenario_file(name, config_dir)
    if not os.path.isfile(path):
        if str(name or '') in ('', DEFAULT_SCENARIO_NAME):
            return {}, None
        raise ScenarioError('scenario preset not found: %s' % path)

    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ScenarioError('scenario file must be a mapping: %s' % path)

    mode = data.get('mode', 'random')
    if mode not in VALID_MODES:
        raise ScenarioError('unknown scenario mode %r (valid: %s) in %s' % (
            mode, ', '.join(VALID_MODES), path))
    return data, path


def effective_mode(scenario, use_existing_agents):
    """
    决定生效模式：
    - 场景文件显式给定 mode → 使用之
    - 否则回退旧参数：use_existing_agents True→replay，False→random
    """
    mode = (scenario or {}).get('mode')
    if mode:
        return mode
    return 'replay' if use_existing_agents else 'random'


class MapError(RuntimeError):
    """地图配置错误（文件缺失 / 格式错误）"""


def resolve_data_scene(name, priorfiles_dir):
    """把 DJI_XXXX 数据集名解析为各数据文件路径（目录级共享 + 场景级 agents）。"""
    digits = name[len('DJI_'):]
    dlp = os.path.join(priorfiles_dir, 'data', name)
    if not os.path.exists(dlp + '_scene.json'):
        raise MapError('dataset not found: %s_scene.json' % dlp)
    out = {
        'name': name,
        'dlp_path': dlp,
        'spots_data_path': os.path.join(priorfiles_dir, 'spots_data.pickle'),
        'waypoints_graph_path': os.path.join(priorfiles_dir, 'waypoints_graph.pickle'),
        'parking_maneuvers_path': os.path.join(priorfiles_dir, 'parking_maneuvers.pickle'),
    }
    agents = os.path.join(priorfiles_dir, 'agents_data_%s.pickle' % digits)
    if os.path.exists(agents):
        out['agents_data_path'] = agents
    return out


def _expand_map_paths(data, root, priorfiles, base_dir):
    """展开 ${PARKSIM_ROOT}/${PARKSIM_PRIORFILES}；地图目录内相对文件路径基于 base_dir 展开。"""
    file_keys = ('spots_data_path', 'waypoints_graph_path', 'parking_maneuvers_path',
                 'agents_data_path', 'dlp_path', 'layout', 'base_map', 'spots_draft')
    expanded = {}
    for key, value in data.items():
        if isinstance(value, str):
            value = value.replace('${PARKSIM_ROOT}', root).replace('$PARKSIM_ROOT', root)
            value = value.replace('${PARKSIM_PRIORFILES}', priorfiles).replace('$PARKSIM_PRIORFILES', priorfiles)
            if base_dir and key in file_keys and value and ('${' not in value) and not os.path.isabs(value):
                value = os.path.join(base_dir, value)
        expanded[key] = value
    return expanded


def load_map(name, config_dir):
    """
    加载"场地数据 / 地图"配置，返回路径已展开的 dict。

    三种形式：
    - config/maps/<name>.yaml 预设文件（自定义地图）
    - priorFiles/maps/<name>/map.yaml 地图目录（地图扩展框架，见 docs/map_extension.md）
    - DJI_XXXX 数据集名 → 直接从 priorFiles 推导（dlp 数据集 + agents 数据文件）

    name 为空 → 默认 'DJI_0012'；均不匹配 → 抛 MapError。
    """
    name = str(name or '').strip() or 'DJI_0012'
    root = os.environ.get('PARKSIM_ROOT') or os.path.abspath(os.path.join(config_dir, '..', '..', '..'))
    priorfiles = os.path.join(root, 'python', 'parksim', 'priorFiles')
    path = os.path.join(config_dir, 'maps', name + '.yaml')
    if os.path.isfile(path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise MapError('map file must be a mapping: %s' % path)
        expanded = _expand_map_paths(data, root, priorfiles, None)
        expanded.setdefault('name', name)
        _smoke_check_map_schema(expanded)
        return expanded
    map_dir_yaml = os.path.join(priorfiles, 'maps', name, 'map.yaml')
    if os.path.isfile(map_dir_yaml):
        with open(map_dir_yaml, 'r') as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise MapError('map file must be a mapping: %s' % map_dir_yaml)
        expanded = _expand_map_paths(data, root, priorfiles, os.path.dirname(map_dir_yaml))
        expanded.setdefault('name', name)
        _smoke_check_map_schema(expanded)
        return expanded
    if re.match(r'^DJI_\d+$', name):
        return resolve_data_scene(name, priorfiles)
    raise MapError('map/dataset not found: %s (or DJI_XXXX data scene)' % path)
