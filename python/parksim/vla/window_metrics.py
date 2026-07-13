"""Simulation-time window metrics for long-horizon ParkSim fleet experiments."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from parksim.vla.safety_metrics import (
    _empty_pairwise_metrics,
    _is_automated_vehicle,
    _pairwise_system_metrics,
    load_jsonl,
    trace_integrity_metrics,
)


WINDOW_METRICS = [
    "av_released_count",
    "av_completed_count",
    "av_cumulative_backlog_count",
    "av_cumulative_service_rate",
    "av_throughput_per_sim_hour",
    "av_path_length_m",
    "av_waiting_time_s",
    "av_idle_time_s",
    "av_non_idle_time_s",
    "av_exposure_vehicle_hours",
    "system_min_distance_m",
    "system_near_miss_event_count",
    "system_collision_proxy_event_count",
    "trajectory_conflict_event_count",
    "mixed_intent_conflict_event_count",
    "traffic_delayed_count",
    "traffic_skipped_count",
    "fleet_epoch_count",
    "fleet_event_trigger_epoch_count",
    "fleet_watchdog_epoch_count",
    "fleet_actionable_epoch_count",
    "shield_rejection_count",
    "qwen_fallback_count",
    "feedback_repair_attempt_count",
    "feedback_repair_success_count",
    "decision_barrier_failure_count",
    "window_operating_cost",
]

METRIC_DEFINITIONS_ZH = {
    "av_released_count": "窗口内首次出现的自动驾驶车辆需求数。",
    "av_completed_count": "窗口内完成进场或离场任务的自动驾驶车辆数。",
    "av_cumulative_released_count": "截至窗口末已释放的自动驾驶需求累计数。",
    "av_cumulative_completed_count": "截至窗口末已完成的自动驾驶任务累计数。",
    "av_cumulative_backlog_count": "截至窗口末已释放但尚未完成的自动驾驶任务数。",
    "av_cumulative_service_rate": "截至窗口末累计完成数与累计释放数之比。",
    "av_throughput_per_sim_hour": "窗口完成数按 3600 s 归一化后的吞吐量。",
    "active_av_count": "窗口内存在轨迹暴露的自动驾驶车辆数。",
    "active_human_vehicle_count": "窗口内存在轨迹暴露的人工驾驶车辆数。",
    "hidden_intent_human_vehicle_count": "窗口内目标泊位和意向路径不可观测的人工驾驶车辆数。",
    "av_path_length_m": "窗口内全部自动驾驶车辆轨迹段长度之和。",
    "av_waiting_time_s": "窗口内 waiting_for 非零的自动驾驶车辆累计仿真时长。",
    "av_idle_time_s": "窗口内自动驾驶车辆处于 IDLE 状态的累计仿真时长。",
    "av_non_idle_time_s": "窗口内自动驾驶车辆非 IDLE 状态累计仿真时长。",
    "av_exposure_vehicle_hours": "窗口内自动驾驶车辆暴露时长总和，以车小时计。",
    "system_min_distance_m": "窗口内任意车辆对的最小平面距离。",
    "system_near_miss_event_count": "窗口内车辆最小距离低于近失阈值的独立事件数。",
    "system_collision_proxy_event_count": "窗口内车辆占用距离低于碰撞代理阈值的独立事件数。",
    "trajectory_conflict_event_count": "窗口内预测轨迹冲突事件数。",
    "mixed_intent_conflict_event_count": "窗口内涉及人工隐藏意图车辆的冲突事件数。",
    "traffic_delayed_count": "窗口内因入口、泊位或并发容量约束而延迟的到离场需求记录数。",
    "traffic_skipped_count": "窗口内因无合法泊位等原因被跳过的到离场需求数。",
    "fleet_epoch_count": "窗口内全部云端车队决策轮次。",
    "fleet_event_trigger_epoch_count": "窗口内由状态变化或冲突事件触发的云端决策轮次。",
    "fleet_watchdog_epoch_count": "窗口内由仿真时间看门狗触发的云端决策轮次。",
    "fleet_actionable_epoch_count": "窗口内至少生成一项可执行 AV 动作的车队决策轮次。",
    "shield_rejection_count": "窗口内被系统级 Safety Shield 拒绝的动作数。",
    "qwen_fallback_count": "窗口内 Qwen 决策使用确定性回退的动作数。",
    "feedback_repair_attempt_count": "窗口内外部反馈 critic 请求修正 Qwen 初始方案的轮次。",
    "feedback_repair_success_count": "窗口内反馈后硬约束或路径冲突问题减少的修正轮次。",
    "decision_barrier_failure_count": "窗口内未满足同一仿真时刻全车确认条件的决策轮次。",
    "window_operating_cost": "不含未完成惩罚的窗口诊断代价，仅用于定位退化窗口。",
}


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return default


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _discover_benchmarks(suite_dir: Path) -> List[Path]:
    paths: List[Path] = []
    manifest = suite_dir / "benchmarks.jsonl"
    if manifest.exists():
        for item in load_jsonl(manifest):
            path = Path(str(item.get("benchmark_dir", "")))
            if not path.is_absolute():
                candidate = (suite_dir / path).resolve()
                path = candidate if candidate.exists() else path.resolve()
            if path.exists():
                paths.append(path)
    if (suite_dir / "episodes.jsonl").exists():
        paths.append(suite_dir)
    return sorted(set(paths))


def _discover_runs(suite_dir: Path) -> List[Tuple[Path, Dict[str, Any], Path]]:
    output: List[Tuple[Path, Dict[str, Any], Path]] = []
    for benchmark in _discover_benchmarks(suite_dir):
        for episode in load_jsonl(benchmark / "episodes.jsonl"):
            scenario_id = str(episode.get("scenario_id", ""))
            agent_type = str(episode.get("agent_type", ""))
            run_dir = benchmark / "episodes" / scenario_id / agent_type
            if run_dir.exists():
                output.append((benchmark, episode, run_dir))
    return output


def _trace_time(row: Dict[str, Any]) -> float:
    return _float(row.get("sim_time", row.get("time")))


def _trace_overlaps(rows: Sequence[Dict[str, Any]], start: float, end: float) -> bool:
    return bool(rows) and _trace_time(rows[0]) < end and _trace_time(rows[-1]) >= start


def _trace_window_metrics(rows: Sequence[Dict[str, Any]], start: float, end: float) -> Dict[str, float]:
    path_length = 0.0
    waiting = 0.0
    idle = 0.0
    non_idle = 0.0
    exposure = 0.0
    for left, right in zip(rows, rows[1:]):
        t0 = _trace_time(left)
        t1 = _trace_time(right)
        if t1 <= t0:
            continue
        overlap = max(0.0, min(t1, end) - max(t0, start))
        if overlap <= 0.0:
            continue
        ratio = overlap / (t1 - t0)
        dx = _float(right.get("x")) - _float(left.get("x"))
        dy = _float(right.get("y")) - _float(left.get("y"))
        path_length += math.hypot(dx, dy) * ratio
        exposure += overlap
        if int(_float(left.get("waiting_for"))) != 0:
            waiting += overlap
        if str(left.get("task", "")) == "IDLE":
            idle += overlap
        else:
            non_idle += overlap
    return {
        "path_length_m": path_length,
        "waiting_time_s": waiting,
        "idle_time_s": idle,
        "non_idle_time_s": non_idle,
        "exposure_time_s": exposure,
    }


def _clip_trace(rows: Sequence[Dict[str, Any]], start: float, end: float) -> List[Dict[str, Any]]:
    # Half-open windows prevent a boundary conflict from being counted twice.
    clipped = [row for row in rows if start <= _trace_time(row) < end]
    return clipped


def _summary_completion_time(summary: Dict[str, Any], rows: Sequence[Dict[str, Any]]) -> Optional[float]:
    if summary.get("completed") is not True:
        return None
    value = summary.get("total_time")
    if value is not None:
        return _float(value)
    if rows and rows[-1].get("is_final") is True:
        return _trace_time(rows[-1])
    return None


def _window_safety_metrics(
    traces: Sequence[Tuple[Path, List[Dict[str, Any]]]],
    roles: Dict[str, str],
    start: float,
    end: float,
) -> Dict[str, Any]:
    clipped: List[Tuple[Path, List[Dict[str, Any]]]] = []
    for path, rows in traces:
        subset = _clip_trace(rows, start, end)
        if len(subset) >= 2:
            clipped.append((path, subset))
    if not clipped or not trace_integrity_metrics(clipped)["trace_integrity_ok"]:
        return _empty_pairwise_metrics()
    return _pairwise_system_metrics(
        clipped,
        roles,
        near_miss_radius=3.0,
        collision_radius=1.5,
        intent_conflict_radius=6.0,
        ttc_horizon=4.0,
        max_time_gap=0.25,
    )


def _run_horizon(log_dir: Path, traces: Sequence[Tuple[Path, List[Dict[str, Any]]]]) -> float:
    schedule = _read_json(log_dir / "traffic_schedule.json", {}) or {}
    horizon = _float(schedule.get("long_horizon_duration"))
    if horizon > 0.0:
        return horizon
    return max((_trace_time(rows[-1]) for _, rows in traces if rows), default=0.0)


def collect_window_rows(suite_dir: Path, window_seconds: float = 300.0) -> List[Dict[str, Any]]:
    window_seconds = max(1.0, float(window_seconds))
    output: List[Dict[str, Any]] = []
    for benchmark, episode, run_dir in _discover_runs(suite_dir):
        log_dir = run_dir / "logs"
        traces: List[Tuple[Path, List[Dict[str, Any]]]] = []
        roles: Dict[str, str] = {}
        agent_types: Dict[str, str] = {}
        intent_observable: Dict[str, bool] = {}
        trace_by_id: Dict[str, List[Dict[str, Any]]] = {}
        summaries: Dict[str, Dict[str, Any]] = {}
        for path in sorted(log_dir.glob("vehicle_*_trace.jsonl")):
            rows = sorted(load_jsonl(path), key=_trace_time)
            if not rows:
                continue
            traces.append((path, rows))
            vehicle_id = str(rows[0].get("vehicle_id", ""))
            if not vehicle_id:
                continue
            trace_by_id[vehicle_id] = rows
            roles[vehicle_id] = str(rows[0].get("vehicle_role", "unknown"))
            agent_types[vehicle_id] = str(rows[0].get("agent_type", "unknown"))
            intent_observable[vehicle_id] = bool(rows[0].get("intent_observable", True))
        for path in sorted(log_dir.glob("vehicle_*_summary.json")):
            summary = _read_json(path, {}) or {}
            vehicle_id = str(summary.get("vehicle_id", ""))
            if vehicle_id:
                summaries[vehicle_id] = summary
                roles.setdefault(vehicle_id, str(summary.get("vehicle_role", "unknown")))
                agent_types.setdefault(vehicle_id, str(summary.get("agent_type", "unknown")))
                intent_observable.setdefault(vehicle_id, bool(summary.get("intent_observable", True)))
        automated_ids = {
            vehicle_id
            for vehicle_id in trace_by_id
            if _is_automated_vehicle(roles.get(vehicle_id, ""), agent_types.get(vehicle_id, ""))
        }
        traffic_events = load_jsonl(log_dir / "traffic_events.jsonl")
        fleet_epochs = load_jsonl(run_dir / "fleet_epochs.jsonl")
        horizon = _run_horizon(log_dir, traces)
        windows = int(math.ceil(horizon / window_seconds)) if horizon > 0.0 else 0
        cumulative_released = 0
        cumulative_completed = 0
        for window_id in range(windows):
            start = window_id * window_seconds
            end = min(horizon, start + window_seconds)
            include_end = window_id == windows - 1
            duration = max(0.0, end - start)
            released_ids = {
                vehicle_id
                for vehicle_id in automated_ids
                if start <= _trace_time(trace_by_id[vehicle_id][0]) < end
            }
            completed_ids = {
                vehicle_id
                for vehicle_id in automated_ids
                if (
                    (completion_time := _summary_completion_time(
                        summaries.get(vehicle_id, {}), trace_by_id.get(vehicle_id, [])
                    )) is not None
                    and start <= completion_time
                    and (completion_time < end or (include_end and completion_time <= end))
                )
            }
            cumulative_released += len(released_ids)
            cumulative_completed += len(completed_ids)
            av_path = av_wait = av_idle = av_non_idle = av_exposure = 0.0
            active_av = 0
            active_human = 0
            hidden_human = 0
            for vehicle_id, rows in trace_by_id.items():
                if not _trace_overlaps(rows, start, end):
                    continue
                values = _trace_window_metrics(rows, start, end)
                if vehicle_id in automated_ids:
                    active_av += 1
                    av_path += values["path_length_m"]
                    av_wait += values["waiting_time_s"]
                    av_idle += values["idle_time_s"]
                    av_non_idle += values["non_idle_time_s"]
                    av_exposure += values["exposure_time_s"]
                else:
                    active_human += 1
                    if not intent_observable.get(vehicle_id, True):
                        hidden_human += 1
            safety = _window_safety_metrics(traces, roles, start, end)
            epochs = [row for row in fleet_epochs if start <= _float(row.get("sim_time")) < end]
            decisions = [
                decision
                for epoch in epochs
                for decision in epoch.get("fleet_decisions") or []
                if isinstance(decision, dict)
            ]
            delayed = sum(
                1 for row in traffic_events
                if start <= _float(row.get("sim_time", row.get("time"))) < end
                and row.get("status") == "delayed"
            )
            skipped = sum(
                1 for row in traffic_events
                if start <= _float(row.get("sim_time", row.get("time"))) < end
                and row.get("status") == "skipped"
            )
            near = int(_float(safety.get("system_near_miss_event_count")))
            collision = int(_float(safety.get("system_collision_proxy_event_count")))
            conflicts = int(_float(safety.get("trajectory_conflict_event_count")))
            mixed_conflicts = int(_float(safety.get("mixed_intent_conflict_event_count")))
            shield_rejections = sum(1 for row in decisions if row.get("shield_ok") is False)
            fallbacks = sum(1 for row in decisions if row.get("used_fallback"))
            operating_cost = (
                av_path
                + 0.2 * av_non_idle
                + 0.1 * av_idle
                + 50.0 * collision
                + 10.0 * near
                + 5.0 * conflicts
                + 5.0 * mixed_conflicts
                + 5.0 * shield_rejections
                + 2.0 * fallbacks
            )
            output.append({
                "pair_key": "%s|%s|window_%02d" % (benchmark.name, episode.get("scenario_id", ""), window_id),
                "benchmark_id": benchmark.name,
                "scenario_id": episode.get("scenario_id", ""),
                "agent_type": episode.get("agent_type", run_dir.name),
                "seed": episode.get("seed", ""),
                "background_mode": episode.get("background_mode", ""),
                "window_id": window_id,
                "time_basis": "simulator_time",
                "qwen_wall_clock_latency_in_performance_metrics": False,
                "window_start_s": f"{start:.3f}",
                "window_end_s": f"{end:.3f}",
                "window_duration_s": f"{duration:.3f}",
                "av_released_count": len(released_ids),
                "av_completed_count": len(completed_ids),
                "av_cumulative_released_count": cumulative_released,
                "av_cumulative_completed_count": cumulative_completed,
                "av_cumulative_backlog_count": max(0, cumulative_released - cumulative_completed),
                "av_cumulative_service_rate": cumulative_completed / cumulative_released if cumulative_released else 0.0,
                "av_throughput_per_sim_hour": len(completed_ids) * 3600.0 / duration if duration else 0.0,
                "active_av_count": active_av,
                "active_human_vehicle_count": active_human,
                "hidden_intent_human_vehicle_count": hidden_human,
                "av_path_length_m": round(av_path, 6),
                "av_waiting_time_s": round(av_wait, 6),
                "av_idle_time_s": round(av_idle, 6),
                "av_non_idle_time_s": round(av_non_idle, 6),
                "av_exposure_vehicle_hours": av_exposure / 3600.0,
                "system_min_distance_m": safety.get("system_min_distance_m"),
                "system_near_miss_event_count": near,
                "system_collision_proxy_event_count": collision,
                "trajectory_conflict_event_count": conflicts,
                "mixed_intent_conflict_event_count": mixed_conflicts,
                "traffic_delayed_count": delayed,
                "traffic_skipped_count": skipped,
                "fleet_epoch_count": len(epochs),
                "fleet_event_trigger_epoch_count": sum(1 for row in epochs if row.get("trigger_type") == "event"),
                "fleet_watchdog_epoch_count": sum(1 for row in epochs if row.get("trigger_type") == "watchdog"),
                "fleet_actionable_epoch_count": sum(1 for row in epochs if row.get("fleet_decisions")),
                "shield_rejection_count": shield_rejections,
                "qwen_fallback_count": fallbacks,
                "feedback_repair_attempt_count": sum(1 for row in epochs if row.get("repair_attempted")),
                "feedback_repair_success_count": sum(1 for row in epochs if row.get("repair_success")),
                "decision_barrier_failure_count": sum(
                    1 for row in epochs
                    if row.get("decision_barrier_status") not in (None, "", "applied", "no_action_required")
                ),
                "window_operating_cost": round(operating_cost, 6),
                "run_dir": str(run_dir),
            })
    return output


def paired_window_deltas(
    rows: Sequence[Dict[str, Any]],
    baseline_agent: str,
    target_agent: str,
) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, str, int], Dict[str, Dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        key = (str(row.get("benchmark_id")), str(row.get("scenario_id")), int(row.get("window_id", 0)))
        grouped[key][str(row.get("agent_type"))] = row
    output: List[Dict[str, Any]] = []
    for key, agents in sorted(grouped.items()):
        if baseline_agent not in agents or target_agent not in agents:
            continue
        baseline = agents[baseline_agent]
        target = agents[target_agent]
        item: Dict[str, Any] = {
            "pair_key": target.get("pair_key", ""),
            "benchmark_id": key[0],
            "scenario_id": key[1],
            "window_id": key[2],
            "window_start_s": target.get("window_start_s", ""),
            "window_end_s": target.get("window_end_s", ""),
            "seed": target.get("seed", ""),
            "background_mode": target.get("background_mode", ""),
            "baseline_agent": baseline_agent,
            "target_agent": target_agent,
        }
        for metric in WINDOW_METRICS:
            base_value = _float(baseline.get(metric))
            target_value = _float(target.get(metric))
            item["baseline_" + metric] = base_value
            item["target_" + metric] = target_value
            item["delta_" + metric] = target_value - base_value
        item["degradation_score"] = round(
            250.0 * max(0.0, -_float(item.get("delta_av_completed_count")))
            + 100.0 * max(0.0, _float(item.get("delta_av_cumulative_backlog_count")))
            + max(0.0, _float(item.get("delta_av_path_length_m")))
            + max(0.0, _float(item.get("delta_av_waiting_time_s")))
            + 50.0 * max(0.0, _float(item.get("delta_system_collision_proxy_event_count")))
            + 10.0 * max(0.0, _float(item.get("delta_system_near_miss_event_count")))
            + 5.0 * max(0.0, _float(item.get("delta_trajectory_conflict_event_count")))
            + 5.0 * max(0.0, _float(item.get("delta_mixed_intent_conflict_event_count")))
            + 20.0 * max(0.0, _float(item.get("delta_traffic_delayed_count")))
            + 20.0 * max(0.0, _float(item.get("delta_traffic_skipped_count")))
            + 5.0 * max(0.0, _float(item.get("delta_shield_rejection_count")))
            + 5.0 * max(0.0, _float(item.get("delta_qwen_fallback_count")))
            + 1000.0 * max(0.0, _float(item.get("delta_decision_barrier_failure_count"))),
            6,
        )
        output.append(item)
    return output


def write_outputs(
    suite_dir: Path,
    out_dir: Path,
    baseline_agent: str,
    target_agent: str,
    window_seconds: float,
) -> Dict[str, Any]:
    rows = collect_window_rows(suite_dir, window_seconds)
    deltas = paired_window_deltas(rows, baseline_agent, target_agent)
    ranked = sorted(deltas, key=lambda row: (-_float(row.get("degradation_score")), str(row.get("pair_key"))))
    for rank, row in enumerate(ranked, 1):
        row["degradation_rank"] = rank
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(out_dir / "window_metrics.csv", rows)
    _write_csv(out_dir / "window_paired_deltas.csv", deltas)
    _write_csv(out_dir / "critical_window_rank.csv", ranked)
    manifest = {
        "type": "parksim_vla_simulation_time_window_metrics",
        "suite_dir": str(suite_dir.resolve()),
        "window_seconds": float(window_seconds),
        "baseline_agent": baseline_agent,
        "target_agent": target_agent,
        "row_count": len(rows),
        "paired_window_count": len(deltas),
        "metric_definitions_zh": METRIC_DEFINITIONS_ZH,
        "latency_policy": "Qwen wall-clock response latency is excluded; all windows use simulator time.",
        "degradation_rank_usage": "diagnostic event localization only; not a statistical significance test",
        "degradation_score_definition_zh": (
            "仅用于关键窗口定位的预冻结加权分数：任务完成损失、积压、路径、等待、安全事件、"
            "需求延迟/跳过、shield/fallback 与决策栅栏失败的目标方法减基线正向退化量加权和。"
        ),
        "outputs": [
            "window_metrics.csv",
            "window_paired_deltas.csv",
            "critical_window_rank.csv",
        ],
    }
    (out_dir / "window_metrics_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate fixed simulation-time window metrics for ParkSim VLA suites.")
    parser.add_argument("--suite-dir", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--window-seconds", type=float, default=300.0)
    parser.add_argument("--baseline-agent", default="rule_based")
    parser.add_argument("--target-agent", default="mllm_external_feedback")
    args = parser.parse_args()
    manifest = write_outputs(
        args.suite_dir,
        args.out_dir,
        args.baseline_agent,
        args.target_agent,
        args.window_seconds,
    )
    print(
        "window_metrics=%s rows=%d paired_windows=%d"
        % (args.out_dir, manifest["row_count"], manifest["paired_window_count"])
    )


if __name__ == "__main__":
    main()
