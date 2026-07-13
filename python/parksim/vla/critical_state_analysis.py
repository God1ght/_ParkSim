"""Critical-state analysis for ParkSim-Qwen-VLA CSV/JSON outputs."""

from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


def _read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _write_csv(path: Path, rows: Iterable[Dict[str, Any]], fields: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _num(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key)
    if value in (None, "", "nan", "None"):
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _pct(delta: float, baseline: float) -> str:
    if abs(baseline) < 1e-9:
        return ""
    return f"{delta / baseline * 100.0:.1f}%"


def _classify(qwen: Dict[str, str], baseline: Dict[str, str]) -> tuple[str, str]:
    objective_delta = _num(qwen, "objective_score") - _num(baseline, "objective_score")
    path_delta = _num(qwen, "fleet_total_automated_path_length") - _num(baseline, "fleet_total_automated_path_length")
    wait_delta = _num(qwen, "fleet_total_automated_waiting_time") - _num(baseline, "fleet_total_automated_waiting_time")
    near_delta = _num(qwen, "system_near_miss_event_count") - _num(baseline, "system_near_miss_event_count")
    collision_delta = _num(qwen, "system_collision_proxy_event_count") - _num(baseline, "system_collision_proxy_event_count")
    conflict_delta = _num(qwen, "trajectory_conflict_event_count") - _num(baseline, "trajectory_conflict_event_count")
    if objective_delta <= 0:
        return "system_objective_gain", "Retain the successful fleet bundle as a reproducible prior for matched scenes."
    if collision_delta > 3 or near_delta > 3:
        return "near_miss_or_collision_proxy_increase", "Increase bottleneck time-window risk weight and expose min distance, hidden intent, and yield cost to Qwen."
    if conflict_delta > 10:
        return "trajectory_conflict_accumulation", "Use fleet-level conflict matrices rather than independent shortest path candidates."
    if wait_delta > 5:
        return "waiting_recovery_cost_increase", "Add waiting age, blocking vehicle id, and replan deadline to the fleet state."
    if path_delta < 0:
        return "short_path_not_system_optimal", "Penalize short routes that traverse high-risk bottlenecks."
    return "mixed_objective_degradation", "Inspect per-window metrics and video frames to isolate the degradation window."


def _paired_drivers(rows: List[Dict[str, str]], baseline_agent: str, target_agent: str) -> List[Dict[str, Any]]:
    by_pair: Dict[str, Dict[str, Dict[str, str]]] = defaultdict(dict)
    for row in rows:
        key = row.get("pair_key") or row.get("scenario_id") or row.get("benchmark_id") or "unknown"
        by_pair[key][row.get("agent_type", "")] = row
    output = []
    for pair_key, agents in sorted(by_pair.items()):
        if baseline_agent not in agents or target_agent not in agents:
            continue
        base = agents[baseline_agent]
        qwen = agents[target_agent]
        reason, suggestion = _classify(qwen, base)
        obj_delta = _num(qwen, "objective_score") - _num(base, "objective_score")
        output.append({
            "pair_key": pair_key,
            "seed": qwen.get("seed", ""),
            "qwen_objective": f"{_num(qwen, 'objective_score'):.6f}",
            "baseline_objective": f"{_num(base, 'objective_score'):.6f}",
            "objective_delta_qwen_minus_baseline": f"{obj_delta:.6f}",
            "objective_delta_percent": _pct(obj_delta, _num(base, "objective_score")),
            "path_delta_m": f"{_num(qwen, 'fleet_total_automated_path_length') - _num(base, 'fleet_total_automated_path_length'):.6f}",
            "waiting_delta_s": f"{_num(qwen, 'fleet_total_automated_waiting_time') - _num(base, 'fleet_total_automated_waiting_time'):.6f}",
            "near_miss_delta": f"{_num(qwen, 'system_near_miss_event_count') - _num(base, 'system_near_miss_event_count'):.6f}",
            "collision_proxy_delta": f"{_num(qwen, 'system_collision_proxy_event_count') - _num(base, 'system_collision_proxy_event_count'):.6f}",
            "trajectory_conflict_delta": f"{_num(qwen, 'trajectory_conflict_event_count') - _num(base, 'trajectory_conflict_event_count'):.6f}",
            "traffic_scheduled_count": f"{_num(qwen, 'traffic_scheduled_count'):.0f}",
            "traffic_spawned_count": f"{_num(qwen, 'traffic_spawned_count'):.0f}",
            "traffic_delayed_count": f"{_num(qwen, 'traffic_delayed_count'):.0f}",
            "hidden_intent_event_count": f"{_num(qwen, 'traffic_hidden_event_count'):.0f}",
            "critical_driver": reason,
            "optimization_suggestion": suggestion,
        })
    return output


def _jsonl(path: Path):
    with path.open() as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def _feature(decision: Dict[str, Any], key: str) -> float:
    features = ((decision.get("action_bundle") or {}).get("features") or {})
    try:
        return float(features.get(key) or 0.0)
    except (TypeError, ValueError):
        return 0.0


EVENT_LABELS_ZH = {
    "duplicate_target_proposal": "初始提案包含重复目标泊位",
    "feedback_repair_gain": "反馈降低了硬约束或路径冲突问题",
    "feedback_repair_no_gain": "反馈修正未减少已识别问题",
    "fallback_or_duplicate_target": "执行使用回退或重复目标恢复",
    "high_conflict_action_executed": "执行动作具有较高预测冲突风险",
    "shield_rejection": "安全盾拒绝模型动作",
    "fleet_synchronized_wait": "多辆自动驾驶车辆同步等待",
    "decision_barrier_failure": "同步决策栅栏失败",
    "high_intent_uncertainty": "高人工意图不确定性与风险决策共现",
    "traffic_demand_delayed": "交通需求延迟释放",
    "traffic_demand_skipped": "交通需求未被服务",
    "restored_obstacle_exit": "原场景占用车辆恢复为离场任务",
    "blocked_wait_interval": "车辆受阻持续等待",
    "yield_braking_interval": "车辆持续让行或制动",
    "deadlock_release": "触发死锁恢复",
}

EVENT_SUGGESTIONS = {
    "duplicate_target_proposal": "加强车队级泊位互斥图和候选目标排序。",
    "feedback_repair_gain": "保留该 critic 反馈模板，并在相同冲突拓扑下复核执行收益。",
    "feedback_repair_no_gain": "收紧反馈问题定位，要求修正动作显式对应未解决的车辆和冲突边。",
    "fallback_or_duplicate_target": "检查模型输出合法性、候选覆盖和回退后的任务恢复。",
    "high_conflict_action_executed": "提高瓶颈时间窗与人工意图不确定性的风险权重。",
    "shield_rejection": "检查 Qwen 提案为何违反硬约束，避免把 shield 回退误认为模型增益。",
    "fleet_synchronized_wait": "加入等待年龄和瓶颈通行优先级，避免全车队同时保守等待。",
    "decision_barrier_failure": "该运行不得进入性能聚合，应先修复 ACK 覆盖或仿真时刻一致性。",
    "high_intent_uncertainty": "增加基于历史轨迹的意图置信度和保守时间窗，但不泄漏真值路径。",
}

WINDOW_MECHANISM_LABELS_ZH = {
    "decision_integrity_failure": "同步决策完整性失败",
    "bottleneck_risk_underestimation": "瓶颈或隐藏意图风险低估",
    "late_shield_or_fallback_recovery": "安全盾或回退后的恢复代价",
    "over_conservative_fleet_wait": "车队过度保守同步等待",
    "traffic_release_or_capacity_blocking": "需求释放或容量阻塞",
    "assignment_or_route_inefficiency": "泊位分配或路径选择低效",
    "mixed_window_degradation": "多因素窗口退化",
    "target_gain_or_neutral": "目标方法窗口改善或无退化",
}

WINDOW_MECHANISM_SUGGESTIONS = {
    "decision_integrity_failure": "先修复暂停确认、ACK 覆盖和仿真时刻一致性；该窗口及运行不得进入性能结论。",
    "bottleneck_risk_underestimation": "提高瓶颈时间窗、最小距离和隐藏意图熵的候选代价，并要求 critic 指明冲突车辆对。",
    "late_shield_or_fallback_recovery": "把 shield 拒绝原因和 fallback 后恢复期限反馈给下一轮 Qwen，避免重复生成同类非法动作。",
    "over_conservative_fleet_wait": "加入等待年龄、阻塞车辆和通行优先级，限制多车同时 WAIT 并设置公平性释放条件。",
    "traffic_release_or_capacity_blocking": "联合优化入口通行、离场优先级和泊位周转，避免已释放需求长期占用入口或可离场泊位。",
    "assignment_or_route_inefficiency": "提高系统路径代价与后续瓶颈占用权重，不仅按目标泊位距离排序候选 bundle。",
    "mixed_window_degradation": "复核该窗口的决策、交通和轨迹事件序列，再调整候选代价；不从单一共现事件作因果结论。",
    "target_gain_or_neutral": "保留该窗口作为成功对照，并检查改善是否跨种子和负载层稳定。",
}


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _relative_case(path: Path, root: Optional[Path]) -> str:
    if root is not None:
        try:
            return "/".join(path.parent.resolve().relative_to(root.resolve()).parts)
        except ValueError:
            pass
    return "/".join(path.parent.parts[-3:])


def _discover_epoch_paths(suite_dir: Optional[Path], video_dir: Optional[Path]) -> List[Path]:
    roots: List[Path] = []
    if suite_dir and suite_dir.exists():
        roots.append(suite_dir)
        manifest = suite_dir / "benchmarks.jsonl"
        if manifest.exists():
            for item in _jsonl(manifest):
                benchmark_dir = Path(str(item.get("benchmark_dir", "")))
                if benchmark_dir.exists():
                    roots.append(benchmark_dir)
    if video_dir and video_dir.exists():
        roots.append(video_dir)
    paths = {
        path.resolve()
        for root in roots
        for path in root.glob("**/fleet_epochs.jsonl")
        if path.is_file()
    }
    return sorted(paths)


def _nearest_traffic_event(events: Sequence[Dict[str, Any]], sim_time: float) -> str:
    if not events:
        return ""
    nearest = min(events, key=lambda row: abs(_as_float(row.get("sim_time", row.get("time"))) - sim_time))
    event_time = _as_float(nearest.get("sim_time", nearest.get("time")))
    if abs(event_time - sim_time) > 5.0:
        return ""
    return "%s:%s:%s@%.1fs" % (
        nearest.get("event_id", ""),
        nearest.get("status", ""),
        nearest.get("event_type", ""),
        event_time,
    )


def _visual_lookup(epoch_path: Path, epoch: Dict[str, Any]) -> str:
    sim_time = _as_float(epoch.get("sim_time"))
    bev = epoch.get("fleet_bev_path")
    if bev:
        return "%s @ sim_time=%.1fs" % (bev, sim_time)
    video = epoch_path.parent.parent / "rule_vs_qwen_vla.mp4"
    return "%s @ sim_time=%.1fs" % (video, sim_time)


def _critical_events(epoch_paths: Sequence[Path], root: Optional[Path]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    traffic_cache: Dict[Path, List[Dict[str, Any]]] = {}
    for path in epoch_paths:
        run_dir = path.parent
        traffic_path = run_dir / "logs" / "traffic_events.jsonl"
        traffic_cache[run_dir] = list(_jsonl(traffic_path)) if traffic_path.exists() else []
        case_name = _relative_case(path, root)
        scenario = run_dir.parent.name
        agent_type = run_dir.name
        seed_match = re.search(r"_seed(\d+)", scenario)
        for epoch in _jsonl(path):
            decisions = [row for row in epoch.get("fleet_decisions") or [] if isinstance(row, dict)]
            pre = epoch.get("pre_feedback_critique") or {}
            post = epoch.get("post_feedback_critique") or {}
            pre_hard = int(_as_float(pre.get("hard_violation_count")))
            post_hard = int(_as_float(post.get("hard_violation_count")))
            pre_route = int(_as_float(pre.get("route_conflict_count")))
            post_route = int(_as_float(post.get("route_conflict_count")))
            duplicate_targets = int(_as_float(pre.get("duplicate_target_count")))
            rejected = [row for row in decisions if row.get("shield_ok") is False]
            fallback = [
                row for row in decisions
                if row.get("used_fallback") or "duplicate" in str(row.get("reason", "")).lower()
            ]
            risky = [
                row for row in decisions
                if _feature(row, "conflict_risk") >= 0.8
                or _feature(row, "conflict_vehicle_count") >= 3
            ]
            all_wait = decisions if len(decisions) >= 2 and all(
                str(row.get("action_id", "")).startswith("wait") for row in decisions
            ) else []
            beliefs = (((epoch.get("fleet_context") or {}).get("state") or {}).get("human_intent_beliefs") or [])
            max_entropy = max((_as_float(row.get("normalized_entropy")) for row in beliefs if isinstance(row, dict)), default=0.0)
            barrier_status = str(epoch.get("decision_barrier_status", ""))
            checks: List[Tuple[str, List[Dict[str, Any]], str]] = []
            if duplicate_targets:
                checks.append(("duplicate_target_proposal", decisions, "high"))
            if bool(epoch.get("repair_attempted")):
                if post_hard + post_route < pre_hard + pre_route:
                    checks.append(("feedback_repair_gain", decisions, "medium"))
                else:
                    checks.append(("feedback_repair_no_gain", decisions, "medium"))
            if fallback:
                checks.append(("fallback_or_duplicate_target", fallback, "high"))
            if risky:
                checks.append(("high_conflict_action_executed", risky, "high"))
            if rejected:
                checks.append(("shield_rejection", rejected, "high"))
            if all_wait:
                checks.append(("fleet_synchronized_wait", all_wait, "medium"))
            if barrier_status and barrier_status not in ("applied", "no_action_required"):
                checks.append(("decision_barrier_failure", decisions, "critical"))
            if max_entropy >= 0.8 and (risky or rejected or all_wait):
                checks.append(("high_intent_uncertainty", risky or rejected or all_wait, "medium"))
            sim_time = _as_float(epoch.get("sim_time"))
            for event_type, selected, severity in checks:
                selected = selected or decisions
                risks = [_feature(row, "conflict_risk") for row in selected]
                conflict_counts = [_feature(row, "conflict_vehicle_count") for row in selected]
                rows.append({
                    "case": case_name,
                    "scenario": scenario,
                    "agent_type": agent_type,
                    "seed": seed_match.group(1) if seed_match else "",
                    "sim_time": f"{sim_time:.3f}",
                    "fleet_epoch": epoch.get("epoch_id", ""),
                    "event_type": event_type,
                    "event_type_zh": EVENT_LABELS_ZH.get(event_type, event_type),
                    "severity": severity,
                    "trigger_type": epoch.get("trigger_type", ""),
                    "trigger_reasons": json.dumps(epoch.get("trigger_reasons") or {}, sort_keys=True),
                    "vehicle_ids": ",".join(str(row.get("vehicle_id", "")) for row in selected),
                    "action_ids": ",".join(str(row.get("action_id", "")) for row in selected),
                    "target_spots": ",".join(str(row.get("target_spot_index", "")) for row in selected),
                    "pre_hard_violation_count": pre_hard,
                    "post_hard_violation_count": post_hard,
                    "pre_route_conflict_count": pre_route,
                    "post_route_conflict_count": post_route,
                    "shield_rejection_count": len(rejected),
                    "fallback_count": sum(1 for row in selected if row.get("used_fallback")),
                    "max_conflict_risk": f"{max(risks, default=0.0):.3f}",
                    "max_conflict_vehicle_count": f"{max(conflict_counts, default=0.0):.0f}",
                    "max_human_intent_entropy": f"{max_entropy:.3f}",
                    "decision_barrier_status": barrier_status,
                    "linked_traffic_event": _nearest_traffic_event(traffic_cache[run_dir], sim_time),
                    "optimization_suggestion": EVENT_SUGGESTIONS.get(event_type, "Review the synchronized fleet state and executed bundle."),
                    "video_lookup": _visual_lookup(path, epoch),
                })
    return rows


def _critical_traffic_events(run_dirs: Sequence[Path], root: Optional[Path]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for run_dir in run_dirs:
        path = run_dir / "logs" / "traffic_events.jsonl"
        if not path.exists():
            continue
        case_name = _relative_case(run_dir / "fleet_epochs.jsonl", root)
        for event in _jsonl(path):
            status = str(event.get("status", ""))
            source = str(event.get("source", ""))
            if status not in ("delayed", "skipped") and source != "static_obstacle_restore":
                continue
            if source == "static_obstacle_restore":
                event_type = "restored_obstacle_exit"
            elif status == "delayed":
                event_type = "traffic_demand_delayed"
            else:
                event_type = "traffic_demand_skipped"
            rows.append({
                "case": case_name,
                "agent_type": run_dir.name,
                "sim_time": f"{_as_float(event.get('sim_time', event.get('time'))):.3f}",
                "event_id": event.get("event_id", ""),
                "event_type": event_type,
                "event_type_zh": EVENT_LABELS_ZH[event_type],
                "demand_type": event.get("event_type", ""),
                "source": source,
                "status": status,
                "reason": event.get("reason", ""),
                "vehicle_id": event.get("vehicle_id", ""),
                "spot_index": event.get("spot_index", ""),
                "intent_hidden": not bool(event.get("intent_observable", True)),
                "active_vehicle_count": event.get("active_vehicle_count", ""),
            })
    return rows


def _trajectory_event(row: Dict[str, Any]) -> str:
    task = str(row.get("task", ""))
    if int(_as_float(row.get("waiting_for"))) != 0:
        return "blocked_wait_interval"
    if task != "IDLE" and bool(row.get("is_braking")) and abs(_as_float(row.get("speed"))) < 0.2:
        return "yield_braking_interval"
    return ""


def _critical_trajectory_intervals(
    run_dirs: Sequence[Path],
    root: Optional[Path],
    minimum_duration: float,
) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    for run_dir in run_dirs:
        case_name = _relative_case(run_dir / "fleet_epochs.jsonl", root)
        for trace_path in sorted((run_dir / "logs").glob("vehicle_*_trace.jsonl")):
            trace = sorted(_jsonl(trace_path), key=lambda row: _as_float(row.get("sim_time", row.get("time"))))
            active: List[Dict[str, Any]] = []
            active_type = ""
            previous_time: Optional[float] = None
            previous_deadlock_count = 0

            def flush() -> None:
                nonlocal active, active_type
                if not active:
                    return
                start = _as_float(active[0].get("sim_time", active[0].get("time")))
                end = _as_float(active[-1].get("sim_time", active[-1].get("time")))
                duration = max(0.0, end - start)
                if duration + 1e-9 >= minimum_duration:
                    blockers = sorted({int(_as_float(row.get("waiting_for"))) for row in active if int(_as_float(row.get("waiting_for"))) != 0})
                    output.append({
                        "case": case_name,
                        "agent_type": run_dir.name,
                        "vehicle_id": active[0].get("vehicle_id", ""),
                        "vehicle_role": active[0].get("vehicle_role", ""),
                        "intent_hidden": not bool(active[0].get("intent_observable", True)),
                        "event_type": active_type,
                        "event_type_zh": EVENT_LABELS_ZH[active_type],
                        "task": active[0].get("task", ""),
                        "start_sim_time": f"{start:.3f}",
                        "end_sim_time": f"{end:.3f}",
                        "duration_s": f"{duration:.3f}",
                        "blocking_vehicle_ids": ",".join(str(value) for value in blockers),
                        "mean_speed_mps": f"{sum(abs(_as_float(row.get('speed'))) for row in active) / len(active):.3f}",
                        "trace_path": str(trace_path),
                    })
                active = []
                active_type = ""

            for row in trace:
                sim_time = _as_float(row.get("sim_time", row.get("time")))
                deadlock_count = int(_as_float(row.get("deadlock_release_count")))
                if deadlock_count > previous_deadlock_count:
                    output.append({
                        "case": case_name,
                        "agent_type": run_dir.name,
                        "vehicle_id": row.get("vehicle_id", ""),
                        "vehicle_role": row.get("vehicle_role", ""),
                        "intent_hidden": not bool(row.get("intent_observable", True)),
                        "event_type": "deadlock_release",
                        "event_type_zh": EVENT_LABELS_ZH["deadlock_release"],
                        "task": row.get("task", ""),
                        "start_sim_time": f"{sim_time:.3f}",
                        "end_sim_time": f"{sim_time:.3f}",
                        "duration_s": "0.000",
                        "blocking_vehicle_ids": row.get("waiting_for", ""),
                        "mean_speed_mps": f"{abs(_as_float(row.get('speed'))):.3f}",
                        "trace_path": str(trace_path),
                    })
                previous_deadlock_count = deadlock_count
                event_type = _trajectory_event(row)
                contiguous = previous_time is None or sim_time - previous_time <= 1.5
                if event_type and event_type == active_type and contiguous:
                    active.append(row)
                else:
                    flush()
                    if event_type:
                        active_type = event_type
                        active = [row]
                previous_time = sim_time
            flush()
    return output


def _row_matches_run(row: Dict[str, Any], benchmark_id: str, scenario: str, agent_type: str) -> bool:
    if str(row.get("agent_type", "")) != agent_type:
        return False
    case_parts = str(row.get("case", "")).split("/")
    if benchmark_id and benchmark_id not in case_parts:
        return False
    explicit = str(row.get("scenario", ""))
    if explicit:
        return explicit == scenario
    return scenario in case_parts


def _events_in_window(
    rows: Sequence[Dict[str, Any]],
    benchmark_id: str,
    scenario: str,
    agent_type: str,
    start: float,
    end: float,
    interval_rows: bool = False,
) -> List[Dict[str, Any]]:
    selected: List[Dict[str, Any]] = []
    for row in rows:
        if not _row_matches_run(row, benchmark_id, scenario, agent_type):
            continue
        if interval_rows:
            event_start = _as_float(row.get("start_sim_time"))
            event_end = _as_float(row.get("end_sim_time"), event_start)
            if event_start < end and event_end >= start:
                selected.append(row)
        else:
            sim_time = _as_float(row.get("sim_time"))
            if start <= sim_time < end:
                selected.append(row)
    return selected


def _mechanism_class(window: Dict[str, str], excess: Counter) -> str:
    if _num(window, "delta_decision_barrier_failure_count") > 0:
        return "decision_integrity_failure"
    safety_worse = (
        _num(window, "delta_system_collision_proxy_event_count") > 0
        or _num(window, "delta_system_near_miss_event_count") > 0
        or _num(window, "delta_trajectory_conflict_event_count") > 0
        or _num(window, "delta_mixed_intent_conflict_event_count") > 0
    )
    if safety_worse and (
        excess["high_conflict_action_executed"] > 0
        or excess["high_intent_uncertainty"] > 0
    ):
        return "bottleneck_risk_underestimation"
    if safety_worse and (
        excess["shield_rejection"] > 0
        or excess["fallback_or_duplicate_target"] > 0
    ):
        return "late_shield_or_fallback_recovery"
    service_worse = (
        _num(window, "delta_av_completed_count") < 0
        or _num(window, "delta_av_cumulative_backlog_count") > 0
        or _num(window, "delta_av_waiting_time_s") > 0
    )
    if service_worse and (
        excess["fleet_synchronized_wait"] > 0
        or excess["blocked_wait_interval"] > 0
        or excess["yield_braking_interval"] > 0
    ):
        return "over_conservative_fleet_wait"
    if service_worse and (
        excess["traffic_demand_delayed"] > 0
        or excess["traffic_demand_skipped"] > 0
    ):
        return "traffic_release_or_capacity_blocking"
    if _num(window, "delta_av_path_length_m") > 0:
        return "assignment_or_route_inefficiency"
    if _num(window, "degradation_score") <= 0:
        return "target_gain_or_neutral"
    return "mixed_window_degradation"


def _window_mechanism_evidence(
    windows: Sequence[Dict[str, str]],
    events: Sequence[Dict[str, Any]],
    traffic_events: Sequence[Dict[str, Any]],
    trajectory_intervals: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    for window in windows:
        scenario = str(window.get("scenario_id", ""))
        benchmark_id = str(window.get("benchmark_id", ""))
        baseline_agent = str(window.get("baseline_agent", ""))
        target_agent = str(window.get("target_agent", ""))
        if not scenario or not baseline_agent or not target_agent:
            continue
        start = _num(window, "window_start_s")
        end = _num(window, "window_end_s")
        target_rows = (
            _events_in_window(events, benchmark_id, scenario, target_agent, start, end)
            + _events_in_window(traffic_events, benchmark_id, scenario, target_agent, start, end)
            + _events_in_window(trajectory_intervals, benchmark_id, scenario, target_agent, start, end, interval_rows=True)
        )
        baseline_rows = (
            _events_in_window(events, benchmark_id, scenario, baseline_agent, start, end)
            + _events_in_window(traffic_events, benchmark_id, scenario, baseline_agent, start, end)
            + _events_in_window(trajectory_intervals, benchmark_id, scenario, baseline_agent, start, end, interval_rows=True)
        )
        target_counts = Counter(str(row.get("event_type", "")) for row in target_rows if row.get("event_type"))
        baseline_counts = Counter(str(row.get("event_type", "")) for row in baseline_rows if row.get("event_type"))
        deltas = Counter({
            key: target_counts[key] - baseline_counts[key]
            for key in set(target_counts) | set(baseline_counts)
        })
        excess = Counter({key: value for key, value in deltas.items() if value > 0})
        mechanism = _mechanism_class(window, excess)
        samples = []
        for row in sorted(
            target_rows,
            key=lambda item: _as_float(item.get("sim_time", item.get("start_sim_time"))),
        )[:12]:
            samples.append({
                "event_type": row.get("event_type", ""),
                "event_type_zh": row.get("event_type_zh", ""),
                "sim_time": row.get("sim_time", row.get("start_sim_time", "")),
                "vehicle_ids": row.get("vehicle_ids", row.get("vehicle_id", "")),
                "action_ids": row.get("action_ids", ""),
                "duration_s": row.get("duration_s", ""),
                "video_lookup": row.get("video_lookup", ""),
            })
        visual_lookups = list(dict.fromkeys(
            str(row.get("video_lookup"))
            for row in target_rows
            if row.get("video_lookup")
        ))
        output.append({
            "pair_key": window.get("pair_key", ""),
            "benchmark_id": benchmark_id,
            "scenario_id": scenario,
            "seed": window.get("seed", ""),
            "background_mode": window.get("background_mode", ""),
            "window_id": window.get("window_id", ""),
            "window_start_s": window.get("window_start_s", ""),
            "window_end_s": window.get("window_end_s", ""),
            "baseline_agent": baseline_agent,
            "target_agent": target_agent,
            "degradation_rank": window.get("degradation_rank", ""),
            "degradation_score": window.get("degradation_score", ""),
            "delta_av_completed_count": window.get("delta_av_completed_count", ""),
            "delta_av_cumulative_backlog_count": window.get("delta_av_cumulative_backlog_count", ""),
            "delta_av_path_length_m": window.get("delta_av_path_length_m", ""),
            "delta_av_waiting_time_s": window.get("delta_av_waiting_time_s", ""),
            "delta_system_near_miss_event_count": window.get("delta_system_near_miss_event_count", ""),
            "delta_system_collision_proxy_event_count": window.get("delta_system_collision_proxy_event_count", ""),
            "delta_trajectory_conflict_event_count": window.get("delta_trajectory_conflict_event_count", ""),
            "delta_mixed_intent_conflict_event_count": window.get("delta_mixed_intent_conflict_event_count", ""),
            "delta_traffic_delayed_count": window.get("delta_traffic_delayed_count", ""),
            "delta_shield_rejection_count": window.get("delta_shield_rejection_count", ""),
            "delta_qwen_fallback_count": window.get("delta_qwen_fallback_count", ""),
            "mechanism_class": mechanism,
            "mechanism_class_zh": WINDOW_MECHANISM_LABELS_ZH[mechanism],
            "target_event_count": len(target_rows),
            "baseline_event_count": len(baseline_rows),
            "target_event_counts_json": json.dumps(target_counts, ensure_ascii=False, sort_keys=True),
            "baseline_event_counts_json": json.dumps(baseline_counts, ensure_ascii=False, sort_keys=True),
            "excess_event_counts_json": json.dumps(excess, ensure_ascii=False, sort_keys=True),
            "target_event_samples_json": json.dumps(samples, ensure_ascii=False, sort_keys=True),
            "target_visual_lookups": " | ".join(visual_lookups[:4]),
            "optimization_suggestion": WINDOW_MECHANISM_SUGGESTIONS[mechanism],
            "evidence_interpretation": "diagnostic association; not causal proof",
        })
    return output


def _write_summary(
    path: Path,
    drivers: List[Dict[str, Any]],
    events: List[Dict[str, Any]],
    traffic_events: List[Dict[str, Any]],
    trajectory_intervals: List[Dict[str, Any]],
    window_mechanisms: Optional[List[Dict[str, Any]]] = None,
) -> None:
    window_mechanisms = window_mechanisms or []
    driver_counts = Counter(row["critical_driver"] for row in drivers)
    event_counts = Counter(row["event_type"] for row in events)
    traffic_counts = Counter(row["event_type"] for row in traffic_events)
    interval_counts = Counter(row["event_type"] for row in trajectory_intervals)
    lines = [
        "# 关键状态分析",
        "",
        "本报告由冻结的 CSV/JSON 日志生成，不会自动修改 TeX 或论文结论。",
        "",
        "## 指标差异驱动",
    ]
    for key, count in driver_counts.most_common():
        lines.append(f"- {key}: {count}")
    if not driver_counts:
        lines.append("- 当前没有可配对的基线与目标方法行")
    lines.extend(["", "## 关键窗口机制证据"])
    for row in sorted(window_mechanisms, key=lambda item: _as_float(item.get("degradation_rank"), 1e9))[:20]:
        lines.append(
            "- rank %s, %s, %.1f--%.1f s: %s; target/base events=%s/%s; %s"
            % (
                row.get("degradation_rank", ""),
                row.get("scenario_id", ""),
                _as_float(row.get("window_start_s")),
                _as_float(row.get("window_end_s")),
                row.get("mechanism_class_zh", ""),
                row.get("target_event_count", 0),
                row.get("baseline_event_count", 0),
                row.get("optimization_suggestion", ""),
            )
        )
    if not window_mechanisms:
        lines.append("- 当前没有 window-level 配对数据")
    lines.extend(["", "## 决策级关键事件"])
    for key, count in event_counts.most_common():
        lines.append(f"- {EVENT_LABELS_ZH.get(key, key)} ({key}): {count}")
    if not event_counts:
        lines.append("- 当前规则未检测到决策级关键事件")
    lines.extend(["", "## 交通需求关键事件"])
    for key, count in traffic_counts.most_common():
        lines.append(f"- {EVENT_LABELS_ZH.get(key, key)} ({key}): {count}")
    if not traffic_counts:
        lines.append("- 无延迟、跳过或恢复离场车辆事件")
    lines.extend(["", "## 轨迹持续状态"])
    for key, count in interval_counts.most_common():
        lines.append(f"- {EVENT_LABELS_ZH.get(key, key)} ({key}): {count}")
    if not interval_counts:
        lines.append("- 无超过阈值的持续等待/制动或死锁恢复")
    lines.extend([
        "",
        "## 字段定义",
        "- `sim_time`：与仿真固定步长一致的事件时刻，不使用 Qwen 墙钟响应时间。",
        "- `linked_traffic_event`：关键决策前后 5 s 内最近的需求释放、延迟或离场事件。",
        "- `visual_lookup`/`video_lookup`：按同一仿真时刻检索 BEV、视频或动图片段。",
        "- `duration_s`：车辆持续受阻、让行或制动的仿真时长。",
        "- `mechanism_class`：窗口差异与同窗事件的诊断性关联，不构成因果证明。",
    ])
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate critical-state CSVs from ParkSim VLA reports.")
    parser.add_argument("--report-dir", required=True, type=Path)
    parser.add_argument("--suite-dir", type=Path)
    parser.add_argument("--video-dir", type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--baseline-agent", default="rule_based")
    parser.add_argument("--target-agent", default="qwen_vla")
    parser.add_argument("--minimum-interval-seconds", type=float, default=5.0)
    args = parser.parse_args()
    report_rows = _read_csv(args.report_dir / "paper_rows.csv")
    drivers = _paired_drivers(report_rows, args.baseline_agent, args.target_agent)
    epoch_paths = _discover_epoch_paths(args.suite_dir, args.video_dir)
    root = args.suite_dir or args.video_dir
    events = _critical_events(epoch_paths, root)
    run_dirs = sorted({path.parent for path in epoch_paths})
    traffic_events = _critical_traffic_events(run_dirs, root)
    trajectory_intervals = _critical_trajectory_intervals(
        run_dirs,
        root,
        max(0.0, args.minimum_interval_seconds),
    )
    window_rows = _read_csv(args.out_dir / "critical_window_rank.csv")
    window_mechanisms = _window_mechanism_evidence(
        window_rows,
        events,
        traffic_events,
        trajectory_intervals,
    )
    args.out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(args.out_dir / "critical_metric_drivers.csv", drivers, [
        "pair_key", "seed", "qwen_objective", "baseline_objective",
        "objective_delta_qwen_minus_baseline", "objective_delta_percent", "path_delta_m",
        "waiting_delta_s", "near_miss_delta", "collision_proxy_delta",
        "trajectory_conflict_delta", "traffic_scheduled_count", "traffic_spawned_count",
        "traffic_delayed_count", "hidden_intent_event_count", "critical_driver",
        "optimization_suggestion",
    ])
    _write_csv(args.out_dir / "critical_state_events.csv", events, [
        "case", "scenario", "agent_type", "seed", "sim_time", "fleet_epoch",
        "event_type", "event_type_zh", "severity", "trigger_type", "trigger_reasons",
        "vehicle_ids", "action_ids", "target_spots", "pre_hard_violation_count",
        "post_hard_violation_count", "pre_route_conflict_count", "post_route_conflict_count",
        "shield_rejection_count", "fallback_count", "max_conflict_risk",
        "max_conflict_vehicle_count", "max_human_intent_entropy", "decision_barrier_status",
        "linked_traffic_event", "optimization_suggestion", "video_lookup",
    ])
    _write_csv(args.out_dir / "critical_traffic_events.csv", traffic_events, [
        "case", "agent_type", "sim_time", "event_id", "event_type", "event_type_zh",
        "demand_type", "source", "status", "reason", "vehicle_id", "spot_index",
        "intent_hidden", "active_vehicle_count",
    ])
    _write_csv(args.out_dir / "critical_trajectory_intervals.csv", trajectory_intervals, [
        "case", "agent_type", "vehicle_id", "vehicle_role", "intent_hidden",
        "event_type", "event_type_zh", "task", "start_sim_time", "end_sim_time",
        "duration_s", "blocking_vehicle_ids", "mean_speed_mps", "trace_path",
    ])
    _write_csv(args.out_dir / "critical_window_mechanism_evidence.csv", window_mechanisms, [
        "pair_key", "benchmark_id", "scenario_id", "seed", "background_mode",
        "window_id", "window_start_s", "window_end_s", "baseline_agent", "target_agent",
        "degradation_rank", "degradation_score", "delta_av_completed_count",
        "delta_av_cumulative_backlog_count", "delta_av_path_length_m", "delta_av_waiting_time_s",
        "delta_system_near_miss_event_count", "delta_system_collision_proxy_event_count",
        "delta_trajectory_conflict_event_count", "delta_mixed_intent_conflict_event_count",
        "delta_traffic_delayed_count", "delta_shield_rejection_count", "delta_qwen_fallback_count",
        "mechanism_class", "mechanism_class_zh", "target_event_count", "baseline_event_count",
        "target_event_counts_json", "baseline_event_counts_json", "excess_event_counts_json",
        "target_event_samples_json", "target_visual_lookups", "optimization_suggestion",
        "evidence_interpretation",
    ])
    _write_summary(
        args.out_dir / "critical_state_analysis.md",
        drivers,
        events,
        traffic_events,
        trajectory_intervals,
        window_mechanisms,
    )
    print(
        "critical_state_analysis=%s drivers=%d decision_events=%d traffic_events=%d trajectory_intervals=%d window_mechanisms=%d"
        % (args.out_dir, len(drivers), len(events), len(traffic_events), len(trajectory_intervals), len(window_mechanisms))
    )


if __name__ == "__main__":
    main()
