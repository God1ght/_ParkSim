import argparse
import csv
import json
import math
import random
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

DEFAULT_METRICS = [
    "completed",
    "automated_vehicle_completion_rate",
    "cloud_served_vehicle_completion_rate",
    "automated_demand_service_rate",
    "automated_throughput_per_sim_hour",
    "automated_demand_completed_count",
    "automated_demand_backlog_count",
    "system_near_miss_events_per_100_vehicle_km",
    "system_collision_proxy_events_per_100_vehicle_km",
    "trajectory_conflicts_per_100_vehicle_km",
    "feedback_repair_success_rate",
    "feedback_hard_violation_reduction_rate",
    "feedback_route_conflict_reduction_rate",
    "objective_score",
    "fleet_total_automated_path_length",
    "fleet_total_automated_waiting_time",
    "fleet_mean_automated_path_length",
    "fleet_mean_automated_waiting_time",
    "fleet_total_automated_non_idle_time",
    "fleet_mean_automated_total_time",
    "path_length",
    "total_non_idle_time",
    "idle_time",
    "near_miss_event_count",
    "collision_proxy_event_count",
    "system_near_miss_event_count",
    "system_collision_proxy_event_count",
    "trajectory_conflict_event_count",
    "mixed_intent_conflict_event_count",
    "traffic_spawned_count",
    "automated_vehicle_count",
    "cloud_served_vehicle_count",
    "human_like_vehicle_count",
    "cloud_fleet_decision_count",
    "cloud_fleet_vehicle_decision_count",
    "cloud_fleet_missing_vehicle_decision_count",
    "cloud_fleet_event_trigger_epoch_count",
    "cloud_fleet_watchdog_epoch_count",
    "cloud_fleet_actionable_epoch_count",
    "cloud_fleet_empty_epoch_count",
    "hidden_intent_vehicle_count",
    "unsafe_occupancy_action_count",
    "target_mismatch_decision_count",
    "missing_reason_code_count",
    "shield_rejection_count",
    "qwen_fallback_count",
    "decision_barrier_ack_coverage_rate",
    "decision_barrier_failure_count",
    "decision_barrier_sim_time_mismatch_count",
    "trace_integrity_ok",
    "trace_integrity_invalid_file_count",
    "trace_identity_conflict_count",
    "trace_time_regression_count",
    "trace_wall_time_regression_count",
    "trace_kinematic_jump_count",
]

HIGHER_IS_BETTER_METRICS = {
    "automated_vehicle_completion_rate",
    "cloud_served_vehicle_completion_rate",
    "automated_demand_service_rate",
    "automated_throughput_per_sim_hour",
    "automated_demand_completed_count",
    "feedback_repair_success_rate",
    "feedback_hard_violation_reduction_rate",
    "feedback_quality_warning_reduction_rate",
    "feedback_route_conflict_reduction_rate",
    "min_other_distance_m",
    "min_ttc_s",
    "system_min_distance_m",
    "trace_integrity_ok",
    "decision_barrier_ack_coverage_rate",
}

STRATIFY_FIELDS = [
    "background_mode",
    "density_label",
    "spawn_profile",
]

MARKDOWN_TEST_METRICS = [
    "automated_demand_service_rate",
    "automated_throughput_per_sim_hour",
    "automated_demand_backlog_count",
    "fleet_mean_automated_total_time",
    "fleet_mean_automated_waiting_time",
    "fleet_mean_automated_path_length",
    "system_near_miss_events_per_100_vehicle_km",
    "system_collision_proxy_events_per_100_vehicle_km",
    "trajectory_conflicts_per_100_vehicle_km",
    "objective_score",
    "path_length",
    "total_non_idle_time",
    "near_miss_event_count",
    "collision_proxy_event_count",
    "system_near_miss_event_count",
    "system_collision_proxy_event_count",
    "trajectory_conflict_event_count",
    "mixed_intent_conflict_event_count",
    "unsafe_occupancy_action_count",
]


def _load_json(path: Path) -> Any:
    with path.open() as f:
        return json.load(f)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def _available_metric_values(rows: Iterable[Dict[str, Any]], metric: str) -> List[float]:
    return [
        _safe_float(row.get(metric))
        for row in rows
        if metric in row and row.get(metric) not in (None, "")
    ]


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def _std(values: Iterable[float]) -> float:
    values = list(values)
    if len(values) < 2:
        return 0.0
    m = _mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def _ci95(values: Iterable[float]) -> float:
    values = list(values)
    if len(values) < 2:
        return 0.0
    return 1.96 * _std(values) / math.sqrt(len(values))


def _paired_bootstrap_ci(values: List[float], iterations: int = 5000, seed: str = "") -> Tuple[float, float]:
    if not values:
        return 0.0, 0.0
    rng = random.Random("parksim-bootstrap:" + seed)
    count = len(values)
    samples = []
    for _ in range(iterations):
        samples.append(_mean(values[rng.randrange(count)] for _ in range(count)))
    samples.sort()
    low_index = max(0, int(0.025 * iterations))
    high_index = min(iterations - 1, int(0.975 * iterations) - 1)
    return samples[low_index], samples[high_index]


def _paired_permutation_p(values: List[float], iterations: int = 5000, seed: str = "") -> float:
    nonzero = [value for value in values if abs(value) > 1e-12]
    if not nonzero:
        return 1.0
    observed = abs(_mean(nonzero))
    extreme = 0
    if len(nonzero) <= 20:
        total = 1 << len(nonzero)
        for mask in range(total):
            candidate = _mean(value if mask & (1 << idx) else -value for idx, value in enumerate(nonzero))
            extreme += int(abs(candidate) >= observed - 1e-12)
        return extreme / total
    rng = random.Random("parksim-permutation:" + seed)
    for _ in range(iterations):
        candidate = _mean(value if rng.random() < 0.5 else -value for value in nonzero)
        extreme += int(abs(candidate) >= observed - 1e-12)
    return (extreme + 1.0) / (iterations + 1.0)


def _apply_holm(rows: List[Dict[str, Any]], p_key: str, output_key: str) -> None:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if row.get(p_key) not in (None, ""):
            grouped[str(row.get("agent_type"))].append(row)
    for group in grouped.values():
        ordered = sorted(group, key=lambda row: _safe_float(row.get(p_key), 1.0))
        running = 0.0
        total = len(ordered)
        for index, row in enumerate(ordered):
            adjusted = min(1.0, (total - index) * _safe_float(row.get(p_key), 1.0))
            running = max(running, adjusted)
            row[output_key] = running
            row[output_key + "_reject_0_05"] = bool(running < 0.05)


def _normal_cdf(value: float) -> float:
    return 0.5 * (1.0 + math.erf(value / math.sqrt(2.0)))


def _two_sided_normal_p(z_value: float) -> float:
    return max(0.0, min(1.0, 2.0 * (1.0 - _normal_cdf(abs(z_value)))))


def _rank_abs_values(values: List[float]) -> List[float]:
    indexed = sorted(enumerate(abs(value) for value in values), key=lambda item: item[1])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(indexed):
        j = i + 1
        while j < len(indexed) and indexed[j][1] == indexed[i][1]:
            j += 1
        average_rank = (i + 1 + j) / 2.0
        for k in range(i, j):
            ranks[indexed[k][0]] = average_rank
        i = j
    return ranks


def _wilcoxon_signed_rank(values: Iterable[float]) -> Dict[str, Any]:
    nonzero = [float(value) for value in values if abs(float(value)) > 1e-12]
    n = len(nonzero)
    if n == 0:
        return {"n": 0, "w_plus": 0.0, "w_minus": 0.0, "statistic": 0.0, "z": 0.0, "p_value": 1.0}
    ranks = _rank_abs_values(nonzero)
    w_plus = sum(rank for rank, value in zip(ranks, nonzero) if value > 0.0)
    w_minus = sum(rank for rank, value in zip(ranks, nonzero) if value < 0.0)
    statistic = min(w_plus, w_minus)
    mean_w = n * (n + 1) / 4.0
    var_w = n * (n + 1) * (2 * n + 1) / 24.0
    if var_w <= 0.0:
        z_value = 0.0
        p_value = 1.0
    else:
        continuity = 0.5 if w_plus > mean_w else -0.5 if w_plus < mean_w else 0.0
        z_value = (w_plus - mean_w - continuity) / math.sqrt(var_w)
        p_value = _two_sided_normal_p(z_value)
    return {
        "n": n,
        "w_plus": w_plus,
        "w_minus": w_minus,
        "statistic": statistic,
        "z": z_value,
        "p_value": p_value,
    }


def _sign_test(values: Iterable[float]) -> Dict[str, Any]:
    values = [float(value) for value in values]
    positive = sum(1 for value in values if value > 1e-12)
    negative = sum(1 for value in values if value < -1e-12)
    ties = len(values) - positive - negative
    n = positive + negative
    if n == 0:
        p_value = 1.0
    else:
        k = min(positive, negative)
        lower_tail = sum(math.comb(n, i) for i in range(k + 1)) / float(2 ** n)
        p_value = min(1.0, 2.0 * lower_tail)
    return {"positive": positive, "negative": negative, "ties": ties, "n": n, "p_value": p_value}


def _metric_direction(metric: str) -> str:
    return "higher_is_better" if metric in HIGHER_IS_BETTER_METRICS else "lower_is_better"


def _reference_better(delta: float, metric: str) -> bool:
    if abs(delta) <= 1e-12:
        return False
    if _metric_direction(metric) == "higher_is_better":
        return delta < 0.0
    return delta > 0.0


def _agent_better(delta: float, metric: str) -> bool:
    if abs(delta) <= 1e-12:
        return False
    if _metric_direction(metric) == "higher_is_better":
        return delta > 0.0
    return delta < 0.0


def _density_label(row: Dict[str, Any], benchmark_id: str) -> str:
    background = str(row.get("background_mode", ""))
    text = benchmark_id
    prefix = background + "_"
    if background and text.startswith(prefix):
        text = text[len(prefix):]
    suffix = "_enter%s_exit%s" % (row.get("spawn_entering", ""), row.get("spawn_exiting", ""))
    if text.endswith(suffix):
        text = text[:-len(suffix)]
    return text or "unknown"


def collect_rows(inputs: List[Path], require_validation_ok: bool = True) -> Tuple[List[Dict[str, Any]], List[str]]:
    rows: List[Dict[str, Any]] = []
    warnings: List[str] = []
    for bench_dir in inputs:
        bench_dir = bench_dir.resolve()
        metrics_path = bench_dir / "metrics.json"
        if not metrics_path.exists():
            warnings.append("skip %s: missing metrics.json" % bench_dir)
            continue
        validation_path = bench_dir / "validation.json"
        if require_validation_ok:
            if not validation_path.exists():
                warnings.append("skip %s: missing validation.json" % bench_dir)
                continue
            validation = _load_json(validation_path)
            if not validation.get("ok"):
                warnings.append("skip %s: validation not ok" % bench_dir)
                continue
        metrics = _load_json(metrics_path)
        if not isinstance(metrics, list):
            warnings.append("skip %s: metrics.json is not a list" % bench_dir)
            continue
        run_config = {}
        run_config_path = bench_dir / "run_config.json"
        if run_config_path.exists():
            run_config = _load_json(run_config_path)
        for idx, row in enumerate(metrics):
            item = dict(row)
            item["benchmark_dir"] = str(bench_dir)
            item["benchmark_id"] = bench_dir.name
            item["row_index"] = idx
            item["pair_key"] = "%s/%s" % (bench_dir.name, item.get("scenario_id", "scenario"))
            item["density_label"] = _density_label(item, bench_dir.name)
            item["spawn_profile"] = "enter%s_exit%s" % (item.get("spawn_entering"), item.get("spawn_exiting"))
            item["git_commit"] = run_config.get("git_commit", "")
            item["git_branch"] = run_config.get("git_branch", "")
            rows.append(item)
    return rows, warnings


def summarize(rows: List[Dict[str, Any]], metrics: List[str]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("agent_type"))].append(row)
    summary: List[Dict[str, Any]] = []
    for agent in sorted(grouped):
        group = grouped[agent]
        item: Dict[str, Any] = {"agent_type": agent, "episodes": len(group)}
        item["scenario_count"] = len({row.get("pair_key") for row in group})
        item["success_rate_mean"] = _mean(1.0 if row.get("completed") else 0.0 for row in group)
        item["fleet_success_rate_mean"] = _mean(_safe_float(row.get("automated_vehicle_completion_rate"), 1.0 if row.get("completed") else 0.0) for row in group)
        for metric in metrics:
            if metric == "completed":
                continue
            values = _available_metric_values(group, metric)
            if not values:
                continue
            item[metric + "_mean"] = _mean(values)
            item[metric + "_std"] = _std(values)
            item[metric + "_ci95"] = _ci95(values)
        summary.append(item)
    return summary


def paired_deltas(rows: List[Dict[str, Any]], reference_agent: str, metrics: List[str]) -> List[Dict[str, Any]]:
    by_pair: Dict[str, Dict[str, Dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        by_pair[str(row.get("pair_key"))][str(row.get("agent_type"))] = row
    deltas: List[Dict[str, Any]] = []
    for pair_key, agents in sorted(by_pair.items()):
        reference = agents.get(reference_agent)
        if not reference:
            continue
        for agent, row in sorted(agents.items()):
            if agent == reference_agent:
                continue
            item = {
                "pair_key": pair_key,
                "agent_type": agent,
                "reference_agent": reference_agent,
                "scenario_id": row.get("scenario_id"),
                "benchmark_id": row.get("benchmark_id"),
            }
            for metric in metrics:
                if metric == "completed":
                    continue
                if (
                    metric not in row
                    or row.get(metric) in (None, "")
                    or metric not in reference
                    or reference.get(metric) in (None, "")
                ):
                    continue
                item[metric + "_delta_vs_ref"] = _safe_float(row.get(metric)) - _safe_float(reference.get(metric))
            deltas.append(item)
    return deltas


def summarize_deltas(deltas: List[Dict[str, Any]], metrics: List[str]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in deltas:
        grouped[str(row.get("agent_type"))].append(row)
    summary: List[Dict[str, Any]] = []
    for agent in sorted(grouped):
        group = grouped[agent]
        item: Dict[str, Any] = {"agent_type": agent, "paired_count": len(group)}
        for metric in metrics:
            if metric == "completed":
                continue
            key = metric + "_delta_vs_ref"
            values = _available_metric_values(group, key)
            if not values:
                continue
            item[key + "_mean"] = _mean(values)
            item[key + "_std"] = _std(values)
            item[key + "_ci95"] = _ci95(values)
        summary.append(item)
    return summary


def statistical_tests(deltas: List[Dict[str, Any]], metrics: List[str]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in deltas:
        grouped[str(row.get("agent_type"))].append(row)
    stats: List[Dict[str, Any]] = []
    for agent in sorted(grouped):
        group = grouped[agent]
        for metric in metrics:
            if metric == "completed":
                continue
            key = metric + "_delta_vs_ref"
            values = _available_metric_values(group, key)
            if not values:
                continue
            sign = _sign_test(values)
            wilcoxon = _wilcoxon_signed_rank(values)
            if metric in MARKDOWN_TEST_METRICS:
                bootstrap_low, bootstrap_high = _paired_bootstrap_ci(values, seed=agent + ":" + metric)
                permutation_p: Optional[float] = _paired_permutation_p(values, seed=agent + ":" + metric)
            else:
                bootstrap_low, bootstrap_high = None, None
                permutation_p = None
            std = _std(values)
            ref_better = sum(1 for value in values if _reference_better(value, metric))
            agent_better = sum(1 for value in values if _agent_better(value, metric))
            tie = len(values) - ref_better - agent_better
            stats.append({
                "agent_type": agent,
                "metric": metric,
                "metric_direction": _metric_direction(metric),
                "paired_count": len(values),
                "delta_mean": _mean(values),
                "delta_std": std,
                "delta_ci95": _ci95(values),
                "delta_bootstrap_ci95_low": bootstrap_low,
                "delta_bootstrap_ci95_high": bootstrap_high,
                "paired_permutation_p": permutation_p,
                "effect_dz": _mean(values) / std if std > 1e-12 else 0.0,
                "reference_better_count": ref_better,
                "agent_better_count": agent_better,
                "tie_count": tie,
                "reference_better_rate": ref_better / len(values) if values else 0.0,
                "agent_better_rate": agent_better / len(values) if values else 0.0,
                "sign_test_n": sign["n"],
                "sign_test_positive_delta": sign["positive"],
                "sign_test_negative_delta": sign["negative"],
                "sign_test_ties": sign["ties"],
                "sign_test_p": sign["p_value"],
                "wilcoxon_n": wilcoxon["n"],
                "wilcoxon_w_plus": wilcoxon["w_plus"],
                "wilcoxon_w_minus": wilcoxon["w_minus"],
                "wilcoxon_statistic": wilcoxon["statistic"],
                "wilcoxon_z": wilcoxon["z"],
                "wilcoxon_p_normal_approx": wilcoxon["p_value"],
            })
    _apply_holm(stats, "paired_permutation_p", "paired_permutation_p_holm")
    return stats


def stratified_summary(rows: List[Dict[str, Any]], metrics: List[str], factors: List[str]) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    for factor in factors:
        factor_rows = [row for row in rows if row.get(factor) is not None]
        grouped: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
        for row in factor_rows:
            grouped[(str(row.get(factor)), str(row.get("agent_type")))].append(row)
        for (level, agent), group in sorted(grouped.items()):
            item: Dict[str, Any] = {
                "factor": factor,
                "level": level,
                "agent_type": agent,
                "episodes": len(group),
                "scenario_count": len({row.get("pair_key") for row in group}),
                "success_rate_mean": _mean(1.0 if row.get("completed") else 0.0 for row in group),
            }
            for metric in metrics:
                if metric == "completed":
                    continue
                values = _available_metric_values(group, metric)
                if not values:
                    continue
                item[metric + "_mean"] = _mean(values)
                item[metric + "_ci95"] = _ci95(values)
            output.append(item)
    return output


def reproducibility_manifest(
    rows: List[Dict[str, Any]],
    inputs: List[Path],
    warnings: List[str],
    reference_agent: str,
    metrics: List[str],
) -> Dict[str, Any]:
    agents = sorted({str(row.get("agent_type")) for row in rows})
    metric_coverage = {
        metric: {
            "present_row_count": len(_available_metric_values(rows, metric)),
            "missing_row_count": len(rows) - len(_available_metric_values(rows, metric)),
            "complete": len(_available_metric_values(rows, metric)) == len(rows),
        }
        for metric in metrics
        if metric != "completed"
    }
    return {
        "type": "parksim_vla_reproducibility_manifest",
        "inputs": [str(path.resolve()) for path in inputs],
        "reference_agent": reference_agent,
        "metrics": metrics,
        "row_count": len(rows),
        "agent_count": len(agents),
        "agents": agents,
        "benchmark_count": len({row.get("benchmark_id") for row in rows}),
        "scenario_count": len({row.get("pair_key") for row in rows}),
        "background_modes": sorted({str(row.get("background_mode")) for row in rows if row.get("background_mode") is not None}),
        "density_labels": sorted({str(row.get("density_label")) for row in rows if row.get("density_label") is not None}),
        "spawn_profiles": sorted({str(row.get("spawn_profile")) for row in rows if row.get("spawn_profile") is not None}),
        "seeds": sorted({str(row.get("seed")) for row in rows if row.get("seed") is not None}),
        "git_commits": sorted({str(row.get("git_commit")) for row in rows if row.get("git_commit")}),
        "git_branches": sorted({str(row.get("git_branch")) for row in rows if row.get("git_branch")}),
        "validated_inputs_only": True,
        "missing_metric_policy": "missing values remain missing and are excluded from aggregation; formal gate rejects required incomplete columns",
        "metric_coverage": metric_coverage,
        "warnings": warnings,
        "outputs": [
            "paper_rows.csv",
            "paper_summary.csv",
            "paper_paired_deltas.csv",
            "paper_paired_delta_summary.csv",
            "paper_statistical_tests.csv",
            "paper_stratified_summary.csv",
            "paper_reproducibility.json",
            "paper_summary.md",
            "paper_table.tex",
            "paper_report_manifest.json",
        ],
    }


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        json.dump(payload, f, indent=2)


def write_markdown(
    path: Path,
    summary: List[Dict[str, Any]],
    delta_summary: List[Dict[str, Any]],
    stats: List[Dict[str, Any]],
    strata: List[Dict[str, Any]],
    reproducibility: Dict[str, Any],
    reference_agent: str,
) -> None:
    lines = [
        "# ParkSim-VLA Paper Report",
        "",
        "## Agent Summary",
        "",
        "| agent | episodes | success | objective | path_m | non_idle_s | near_miss | collision_proxy | barrier_coverage | barrier_failures |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary:
        lines.append(
            "| {agent_type} | {episodes:d} | {success_rate_mean:.3f} | {objective_score_mean:.3f} +/- {objective_score_ci95:.3f} | "
            "{path_length_mean:.3f} +/- {path_length_ci95:.3f} | {total_non_idle_time_mean:.3f} +/- {total_non_idle_time_ci95:.3f} | "
            "{near_miss_event_count_mean:.3f} | {collision_proxy_event_count_mean:.3f} | "
            "{decision_barrier_ack_coverage_rate_mean:.3f} | {decision_barrier_failure_count_mean:.3f} |".format_map(_with_defaults(row))
        )
    lines.extend([
        "",
        "## Paired Delta vs `%s`" % reference_agent,
        "",
        "Positive delta means the agent is worse than the reference for lower-is-better metrics.",
        "",
        "| agent | pairs | objective_delta | path_delta | non_idle_delta | near_miss_delta | collision_delta | unsafe_spot_delta |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in delta_summary:
        lines.append(
            "| {agent_type} | {paired_count:d} | {objective_score_delta_vs_ref_mean:.3f} +/- {objective_score_delta_vs_ref_ci95:.3f} | "
            "{path_length_delta_vs_ref_mean:.3f} | {total_non_idle_time_delta_vs_ref_mean:.3f} | "
            "{near_miss_event_count_delta_vs_ref_mean:.3f} | {collision_proxy_event_count_delta_vs_ref_mean:.3f} | "
            "{unsafe_occupancy_action_count_delta_vs_ref_mean:.3f} |".format_map(_with_defaults(row))
        )
    lines.extend([
        "",
        "## Paired Statistical Tests",
        "",
        "Sign tests are exact two-sided binomial tests after dropping ties. Wilcoxon p-values use the signed-rank normal approximation. For lower-is-better metrics, a positive delta means the compared agent is worse than `%s`." % reference_agent,
        "",
        "| agent | metric | pairs | mean_delta | ci95 | dz | ref_better | agent_better | ties | sign_p | wilcoxon_p |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in stats:
        if row.get("metric") not in MARKDOWN_TEST_METRICS:
            continue
        lines.append(
            "| {agent_type} | {metric} | {paired_count:d} | {delta_mean:.3f} | {delta_ci95:.3f} | {effect_dz:.3f} | "
            "{reference_better_count:d} | {agent_better_count:d} | {tie_count:d} | {sign_test_p:.4f} | {wilcoxon_p_normal_approx:.4f} |".format_map(_with_defaults(row))
        )
    lines.extend([
        "",
        "## Scenario Stratification",
        "",
        "Full stratified results are in `paper_stratified_summary.csv`; the table below keeps the objective score by background mode and density level.",
        "",
        "| factor | level | agent | episodes | success | objective | path_m | non_idle_s |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in strata:
        if row.get("factor") not in {"background_mode", "density_label"}:
            continue
        data = _with_defaults(row)
        lines.append(
            "| {factor} | {level} | {agent_type} | {episodes:d} | {success_rate_mean:.3f} | "
            "{objective_score_mean:.3f} | {path_length_mean:.3f} | {total_non_idle_time_mean:.3f} |".format(**data)
        )
    lines.extend([
        "",
        "## Reproducibility Checklist",
        "",
        "- Validated benchmark inputs: %d" % len(reproducibility.get("inputs", [])),
        "- Rows / agents / paired scenarios: %d / %d / %d" % (
            int(reproducibility.get("row_count", 0)),
            int(reproducibility.get("agent_count", 0)),
            int(reproducibility.get("scenario_count", 0)),
        ),
        "- Background modes: %s" % ", ".join(reproducibility.get("background_modes", [])),
        "- Density labels: %s" % ", ".join(reproducibility.get("density_labels", [])),
        "- Seeds: %s" % ", ".join(reproducibility.get("seeds", [])),
        "- Git commits: %s" % ", ".join(reproducibility.get("git_commits", [])),
        "- Additional machine-readable evidence: `paper_reproducibility.json`, `paper_statistical_tests.csv`, and `paper_report_manifest.json`.",
    ])
    path.write_text("\n".join(lines) + "\n")


def write_latex(path: Path, summary: List[Dict[str, Any]]) -> None:
    line_end = "\\\\"
    lines = [
        "\\begin{tabular}{lrrrrr}",
        "\\toprule",
        "Agent & Success & Objective & Path (m) & Non-idle (s) & Safety events " + line_end,
        "\\midrule",
    ]
    for row in summary:
        data = _with_defaults(row)
        safety = data["near_miss_event_count_mean"] + data["collision_proxy_event_count_mean"] + data["unsafe_occupancy_action_count_mean"]
        lines.append(
            "{} & {:.3f} & {:.3f} +/- {:.3f} & {:.3f} +/- {:.3f} & {:.3f} +/- {:.3f} & {:.3f} {}".format(
                data["agent_type"].replace("_", "\\_"),
                data["success_rate_mean"],
                data["objective_score_mean"],
                data["objective_score_ci95"],
                data["path_length_mean"],
                data["path_length_ci95"],
                data["total_non_idle_time_mean"],
                data["total_non_idle_time_ci95"],
                safety,
                line_end,
            )
        )
    lines.extend(["\\bottomrule", "\\end{tabular}"])
    path.write_text("\n".join(lines) + "\n")

def _with_defaults(row: Dict[str, Any]) -> Dict[str, Any]:
    defaults = defaultdict(float)
    defaults.update(row)
    defaults.setdefault("agent_type", "")
    defaults.setdefault("episodes", 0)
    defaults.setdefault("paired_count", 0)
    return defaults


def main() -> None:
    parser = argparse.ArgumentParser(description="Build paper-ready aggregate tables from ParkSim-VLA benchmark directories.")
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--inputs", nargs="+", type=Path, required=True)
    parser.add_argument("--reference-agent", default="qwen_vla")
    parser.add_argument("--allow-invalid", action="store_true")
    parser.add_argument("--metrics", default=",".join(DEFAULT_METRICS))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    metrics = [item for item in args.metrics.split(",") if item]
    rows, warnings = collect_rows(args.inputs, require_validation_ok=not args.allow_invalid)
    summary = summarize(rows, metrics)
    deltas = paired_deltas(rows, args.reference_agent, metrics)
    delta_summary = summarize_deltas(deltas, metrics)
    stats = statistical_tests(deltas, metrics)
    strata = stratified_summary(rows, metrics, STRATIFY_FIELDS)
    repro = reproducibility_manifest(rows, args.inputs, warnings, args.reference_agent, metrics)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "paper_rows.csv", rows)
    write_csv(out_dir / "paper_summary.csv", summary)
    write_csv(out_dir / "paper_paired_deltas.csv", deltas)
    write_csv(out_dir / "paper_paired_delta_summary.csv", delta_summary)
    write_csv(out_dir / "paper_statistical_tests.csv", stats)
    write_csv(out_dir / "paper_stratified_summary.csv", strata)
    write_json(out_dir / "paper_reproducibility.json", repro)
    write_markdown(out_dir / "paper_summary.md", summary, delta_summary, stats, strata, repro, args.reference_agent)
    write_latex(out_dir / "paper_table.tex", summary)
    manifest = {
        "type": "parksim_vla_paper_report",
        "out_dir": str(out_dir),
        "inputs": [str(path.resolve()) for path in args.inputs],
        "reference_agent": args.reference_agent,
        "metrics": metrics,
        "row_count": len(rows),
        "agent_count": len(summary),
        "paired_delta_count": len(deltas),
        "statistical_test_count": len(stats),
        "stratified_row_count": len(strata),
        "reproducibility_manifest": str(out_dir / "paper_reproducibility.json"),
        "warnings": warnings,
    }
    write_json(out_dir / "paper_report_manifest.json", manifest)
    for warning in warnings:
        print("warning: %s" % warning)
    print("paper_report=%s rows=%d agents=%d paired=%d" % (out_dir, len(rows), len(summary), len(deltas)))


if __name__ == "__main__":
    main()
