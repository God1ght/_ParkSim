#!/usr/bin/env python3
"""
build_agent_data.py — 批量生成 agents_data_XXXX.pickle（经验初始化回放数据）

用途：
  从 DLP 数据集（priorFiles/data/DJI_XXXX_*.json）提取车辆时序，生成仿真回放
  所需的智能体任务档案（与既有 agents_data_0012.pickle 同格式）。

来源：移植自
  - deps/dlp-dataset/raw-data-processing/generate_agent_data.py（时序提取）
  - parksim/utils/generate_profiles.py（任务档案生成；0012 专属后处理仅对 0012 生效）

键约定：与原始脚本一致 —— 键 = scene['agents'] 列表中的序号（跳过行人等非车辆）。
        例如 0012 为 [0..7, 20, 21, 22, 24, 26, 49, ...] 共 32 个。

用法：
  python build_agent_data.py                    # 处理所有场景（跳过已有输出）
  python build_agent_data.py 0001 0003          # 只处理指定场景
  python build_agent_data.py --force 0001       # 覆盖已有输出
  python build_agent_data.py --out-dir /tmp/x   # 输出到指定目录（测试用）
"""
import argparse
import os
import pickle
import sys
import time
from pathlib import Path

import numpy as np
from dlp.dataset import Dataset

PARKED_DEADBAND = 2        # 距车位多远内视为"在该车位"
MOVING_DEADBAND = 0.0001   # 速度阈值
IDLE_MIN_TIME = 15         # 静止超过该秒数视为 IDLE
SKIP_TYPES = {'Pedestrian', 'Undefined', 'Bicycle', 'Motorcycle'}


def load_spots(prior_dir):
    with open(os.path.join(prior_dir, 'spots_data.pickle'), 'rb') as f:
        return pickle.load(f)


def extract_raw(ds, spots):
    """提取全部车辆时序数据，键 = scene['agents'] 序号（与原始脚本一致）。"""
    scene = ds.get('scene', ds.list_scenes()[0])
    spot_xy = np.asarray(spots['parking_spaces'], dtype=float)
    spot_cache, frame_cache = {}, {}
    agents = {}
    for i, agent_token in enumerate(scene['agents']):
        agent = ds.get('agent', agent_token)
        if agent.get('type') in SKIP_TYPES:
            continue
        instances = ds.get_agent_instances(agent_token)
        if len(instances) == 0:
            continue
        t, v, heading, coords, closest, dist = [], [], [], [], [], []
        for inst in instances:
            v.append(inst['speed'])
            heading.append(inst['heading'])
            coords.append(list(inst['coords']))
            ft = inst['frame_token']
            if ft in frame_cache:
                t.append(frame_cache[ft])
            else:
                val = ds.get('frame', ft)['timestamp']
                t.append(val)
                frame_cache[ft] = val
            key = tuple(inst['coords'])
            if key in spot_cache:
                cs, dd = spot_cache[key]
            else:
                c = np.asarray(inst['coords'], dtype=float)
                cs = int(np.argmin(np.linalg.norm(spot_xy - c, axis=1)))
                dd = float(np.linalg.norm(c - spot_xy[cs]))
                spot_cache[key] = (cs, dd)
            closest.append(cs)
            dist.append(dd)
        agents[i] = {
            'start_time': float(t[0]),
            'v': v, 't': t, 'heading': heading, 'coords': coords,
            'closest_spot': closest, 'dist_to_closest_spot': dist,
            'size': list(agent['size']),
        }
    return agents


def make_profile(agent, number, spots, agent_key):
    """移植 generate_profiles.py 的任务档案逻辑；返回 None 表示无效（跳过）。"""
    json_dict = {'task_profile': []}
    n = len(agent['v'])

    # 确定智能体运行的起始时间（第一段速度 > 阈值处）
    spawn_index = -1
    i = 0
    while spawn_index == -1 and i < n:
        if agent['v'][i] >= MOVING_DEADBAND:
            spawn_index = i
            json_dict['init_time'] = agent['t'][i]
        else:
            i += 1
    if spawn_index == -1:
        return None  # 全程静止

    # 初始位置（泊位内 / 自由坐标）与方向
    prev = max(spawn_index - 1, 0)
    if agent['dist_to_closest_spot'][prev] < PARKED_DEADBAND:
        json_dict['init_spot'] = agent['closest_spot'][prev]
        json_dict['init_heading'] = np.pi / 2 if agent['heading'][prev] < np.pi else 3 * np.pi / 2
    else:
        json_dict['init_coords'] = list(agent['coords'][0])
        json_dict['init_heading'] = agent['heading'][prev]
    json_dict['init_v'] = agent['v'][0]
    json_dict['length'] = agent['size'][0]
    json_dict['width'] = agent['size'][1]

    sec_start = spawn_index
    zero_sec_start = -1
    sec_max_speed = -1
    total_non_idle_time = 0

    def add_non_idle_section(end_step, max_speed):
        if agent['dist_to_closest_spot'][end_step] < PARKED_DEADBAND:
            spot = agent['closest_spot'][end_step]
            json_dict['task_profile'].append(
                {'name': 'CRUISE', 'v_cruise': max_speed, 'target_spot_index': spot})
            json_dict['task_profile'].append(
                {'name': 'PARK', 'target_spot_index': spot})
        else:
            json_dict['task_profile'].append(
                {'name': 'CRUISE', 'v_cruise': max_speed,
                 'target_coords': list(agent['coords'][end_step])})

    while i < n:
        if agent['v'][i] < MOVING_DEADBAND:  # 静止
            if zero_sec_start == -1:
                zero_sec_start = i
        else:
            if zero_sec_start != -1:  # 静止段结束
                if agent['t'][i] - agent['t'][zero_sec_start] >= IDLE_MIN_TIME:
                    # 长时间静止 → IDLE 段
                    if agent['dist_to_closest_spot'][max(sec_start - 1, 0)] < PARKED_DEADBAND:
                        json_dict['task_profile'].append(
                            {'name': 'UNPARK',
                             'target_spot_index': agent['closest_spot'][max(sec_start - 1, 0)]})
                    if zero_sec_start - sec_start > 0:
                        add_non_idle_section(zero_sec_start - 1, sec_max_speed)
                        total_non_idle_time += agent['t'][zero_sec_start - 1] - agent['t'][sec_start]
                    json_dict['task_profile'].append(
                        {'name': 'IDLE', 'duration': agent['t'][i] - agent['t'][zero_sec_start]})
                    sec_start = i + 1
                    sec_max_speed = -1
            else:
                sec_max_speed = max(sec_max_speed, agent['v'][i])
            zero_sec_start = -1
        i += 1

    # 收尾段
    if zero_sec_start != -1:  # 以静止结束
        if agent['t'][i - 1] - agent['t'][zero_sec_start] >= IDLE_MIN_TIME:
            if agent['dist_to_closest_spot'][max(sec_start - 1, 0)] < PARKED_DEADBAND:
                json_dict['task_profile'].append(
                    {'name': 'UNPARK',
                     'target_spot_index': agent['closest_spot'][max(sec_start - 1, 0)]})
            if zero_sec_start - sec_start > 0:
                add_non_idle_section(zero_sec_start - 1, sec_max_speed)
                total_non_idle_time += agent['t'][zero_sec_start - 1] - agent['t'][sec_start]
            json_dict['task_profile'].append(
                {'name': 'IDLE', 'duration': agent['t'][i - 1] - agent['t'][zero_sec_start]})
        else:
            add_non_idle_section(i - 1, sec_max_speed)
            total_non_idle_time += agent['t'][i - 1] - agent['t'][sec_start]
    else:
        if agent['dist_to_closest_spot'][max(sec_start - 1, 0)] < PARKED_DEADBAND:
            json_dict['task_profile'].append(
                {'name': 'UNPARK',
                 'target_spot_index': agent['closest_spot'][max(sec_start - 1, 0)]})
        add_non_idle_section(i - 1, sec_max_speed)
        total_non_idle_time += agent['t'][i - 1] - agent['t'][sec_start]

    # 去掉末尾的 IDLE 段
    if json_dict['task_profile'] and json_dict['task_profile'][-1]['name'] == 'IDLE':
        json_dict['task_profile'].pop()

    # 中途泊入调整（首个 CRUISE→PARK 同车位时，把起点改到车道中线附近）
    tp = json_dict['task_profile']
    if (len(tp) >= 2 and tp[0]['name'] == 'CRUISE' and tp[1]['name'] == 'PARK'
            and tp[0].get('target_spot_index') is not None
            and tp[0].get('target_spot_index') == tp[1].get('target_spot_index')
            and 'init_coords' in json_dict):
        tgt_spot = tp[0]['target_spot_index']
        space_coords = spots['parking_spaces'][tgt_spot]
        north_ranges = spots['north_spot_idx_ranges']
        if (abs(json_dict['init_coords'][0] - space_coords[0]) <= 5
                and abs(json_dict['init_coords'][1] - space_coords[1]) <= 12):
            north_spot = any(rg[0] <= tgt_spot <= rg[1] for rg in north_ranges)
            face_right = abs(json_dict['init_heading']) <= np.pi / 2
            if north_spot and face_right:
                new_x, new_y = space_coords[0] - 4, space_coords[1] - 8
            elif north_spot and not face_right:
                new_x, new_y = space_coords[0] + 4, space_coords[1] - 4.5
            elif not north_spot and face_right:
                new_x, new_y = space_coords[0] - 4, space_coords[1] + 4.5
            else:
                new_x, new_y = space_coords[0] + 4, space_coords[1] + 8
            json_dict['init_coords'] = [float(new_x), float(new_y)]
            json_dict['init_heading'] = 0.0 if face_right else float(np.pi)
            tp.pop(0)

    # 0012 专属后处理 hack（保持与既有 agents_data_0012.pickle 完全一致）
    if number == '0012' and agent_key == 16:
        final_loc = json_dict['task_profile'][-1]['target_coords']
        json_dict['task_profile'] = json_dict['task_profile'][:-2]
        json_dict['task_profile'][-1]['target_coords'] = final_loc
    if number == '0012' and agent_key == 23:  # 转向 60 度并左移
        json_dict['init_heading'] -= np.pi / 3
        json_dict['init_coords'][0] -= 3
    # 0012 agent 21：原始代码为无操作（pass）

    if len(json_dict['task_profile']) == 0:
        return None
    return json_dict


def process_scene(number, data_dir, prior_dir, out_dir, force):
    out_path = os.path.join(out_dir, 'agents_data_%s.pickle' % number)
    if os.path.exists(out_path) and not force:
        print('[%s] 已存在，跳过：%s' % (number, out_path))
        return 'skip'
    prefix = os.path.join(data_dir, 'DJI_%s' % number)
    if not os.path.exists(prefix + '_scene.json'):
        print('[%s] 数据不存在（%s_scene.json），跳过' % (number, prefix))
        return 'missing'
    t0 = time.time()
    ds = Dataset()
    ds.load(prefix)
    spots = load_spots(prior_dir)
    raw = extract_raw(ds, spots)
    t1 = time.time()
    final, skipped = {}, 0
    for key, agent in raw.items():
        prof = make_profile(agent, number, spots, key)
        if prof is None:
            skipped += 1
            continue
        final[key] = prof
    t2 = time.time()
    if not final:
        print('[%s] 无有效智能体，跳过' % number)
        return 'empty'
    tmp_path = out_path + '.tmp'
    with open(tmp_path, 'wb') as f:
        pickle.dump(final, f)
    os.replace(tmp_path, out_path)
    print('[%s] OK：车辆 %d（跳过无效 %d）| 载入 %.1fs + 处理 %.1fs → %s'
          % (number, len(final), skipped, t1 - t0, t2 - t1, out_path))
    return 'ok'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('numbers', nargs='*', help='场景编号（如 0001），缺省 = 全部')
    ap.add_argument('--force', action='store_true', help='覆盖已有输出')
    ap.add_argument('--data-dir', default=None, help='数据目录（含 DJI_XXXX_*.json）')
    ap.add_argument('--prior-dir', default=None, help='priorFiles 目录（含 spots_data.pickle）')
    ap.add_argument('--out-dir', default=None, help='输出目录（默认 = priorFiles）')
    args = ap.parse_args()

    root = os.environ.get('PARKSIM_ROOT') or str(Path(__file__).resolve().parents[1])
    prior_dir = args.prior_dir or os.path.join(root, 'python', 'parksim', 'priorFiles')
    data_dir = args.data_dir or os.path.join(prior_dir, 'data')
    out_dir = args.out_dir or prior_dir

    if args.numbers:
        numbers = [str(n).zfill(4) for n in args.numbers]
    else:
        numbers = sorted(fn[len('DJI_'):-len('_scene.json')]
                         for fn in os.listdir(data_dir)
                         if fn.startswith('DJI_') and fn.endswith('_scene.json'))

    print('场景列表（%d）：%s' % (len(numbers), ' '.join(numbers)))
    t_all = time.time()
    stats = {'ok': 0, 'skip': 0, 'missing': 0, 'empty': 0}
    for number in numbers:
        try:
            stats[process_scene(number, data_dir, prior_dir, out_dir, args.force)] += 1
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print('[%s] 失败：%s' % (number, exc))
    print('全部完成：%.1fs | ok=%d skip=%d missing=%d empty=%d'
          % (time.time() - t_all, stats['ok'], stats['skip'], stats['missing'], stats['empty']))


if __name__ == '__main__':
    main()
