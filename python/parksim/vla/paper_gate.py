import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set


PROFILE_DEFAULTS: Dict[str, Dict[str, Any]] = {
    "pilot": {
        "required_agents": ["risk_aware_rule", "qwen_vla"],
        "min_agents": 2,
        "min_seeds": 2,
        "min_background_modes": 2,
        "min_density_labels": 2,
        "min_spawn_profiles": 2,
        "min_scenarios": 4,
        "min_rows": 8,
        "require_real_qwen": False,
        "max_reference_unsafe_actions": 0.0,
        "max_reference_collision_proxy": 0.0,
    },
    "paper": {
        "required_agents": ["rule_based", "greedy_nearest", "greedy_shortest_path", "risk_aware_rule", "bundle_risk_aware", "conflict_aware_bundle", "reservation_bundle", "centralized_min_cost", "oracle_intent_bundle", "qwen_vla"],
        "min_agents": 10,
        "min_seeds": 3,
        "min_background_modes": 3,
        "min_density_labels": 4,
        "min_spawn_profiles": 4,
        "min_scenarios": 27,
        "min_rows": 270,
        "require_real_qwen": True,
        "require_manuscript": True,
        "require_video_evidence": False,
        "require_oracle_upper_bound": True,
        "max_reference_unsafe_actions": 0.0,
        "max_reference_collision_proxy": 0.0,
    },
    "journal": {
        "required_agents": ["rule_based", "greedy_nearest", "greedy_shortest_path", "risk_aware_rule", "bundle_risk_aware", "conflict_aware_bundle", "reservation_bundle", "rolling_horizon_bundle", "centralized_min_cost", "oracle_intent_bundle", "qwen_vla"],
        "min_agents": 11,
        "min_seeds": 5,
        "min_background_modes": 3,
        "min_density_labels": 5,
        "min_spawn_profiles": 4,
        "min_scenarios": 55,
        "min_rows": 605,
        "require_real_qwen": True,
        "require_manuscript": True,
        "require_video_evidence": True,
        "require_oracle_upper_bound": True,
        "max_reference_unsafe_actions": 0.0,
        "max_reference_collision_proxy": 0.0,
    },
    "trc": {
        "required_agents": [
            "rule_based",
            "risk_aware_rule",
            "reservation_bundle",
            "centralized_min_cost",
            "oracle_intent_bundle",
            "fleet_min_cost",
            "mllm_direct",
            "mllm_self_reflect",
            "mllm_external_feedback",
        ],
        "min_agents": 9,
        "min_seeds": 10,
        "min_background_modes": 3,
        "min_density_labels": 3,
        "min_spawn_profiles": 3,
        "min_scenarios": 7,
        "min_rows": 630,
        "require_real_qwen": True,
        "require_manuscript": False,
        "require_video_evidence": True,
        "require_oracle_upper_bound": True,
        "require_open_science": True,
        "require_cloud_fleet": True,
        "max_reference_unsafe_actions": 0.0,
        "max_reference_collision_proxy": 0.0,
    },
}

REQUIRED_MANUSCRIPT_FILES = [
    "docs/qwen_vla_ieee_report_zh.tex",
    "docs/qwen_vla_ieee_report_zh.md",
]

REQUIRED_METRIC_COLUMNS = [
    "objective_score",
    "automated_demand_released_count",
    "automated_demand_service_rate",
    "automated_throughput_per_sim_hour",
    "censored_automated_vehicle_count",
    "system_exposure_vehicle_km",
    "system_exposure_vehicle_hours",
    "system_near_miss_events_per_100_vehicle_km",
    "system_collision_proxy_events_per_100_vehicle_km",
    "trajectory_conflicts_per_100_vehicle_km",
    "feedback_repair_success_rate",
    "feedback_hard_violation_reduction_rate",
    "feedback_route_conflict_reduction_rate",
    "path_length",
    "total_non_idle_time",
    "waiting_time",
    "system_near_miss_event_count",
    "system_collision_proxy_event_count",
    "trajectory_conflict_event_count",
    "mixed_intent_conflict_event_count",
    "unsafe_occupancy_action_count",
    "shield_rejection_count",
    "qwen_fallback_count",
    "decision_barrier_epoch_count",
    "decision_barrier_ack_coverage_rate",
    "decision_barrier_failure_count",
    "decision_barrier_sim_time_mismatch_count",
    "simulation_time_policy",
    "wall_clock_latency_in_performance_metrics",
    "automated_vehicle_count",
    "cloud_served_vehicle_count",
    "human_like_vehicle_count",
    "cloud_fleet_decision_count",
    "cloud_fleet_vehicle_decision_count",
    "trace_integrity_ok",
    "trace_integrity_invalid_file_count",
    "trace_identity_conflict_count",
    "trace_time_regression_count",
    "trace_wall_time_regression_count",
    "trace_kinematic_jump_count",
]

TRC_METRIC_GROUPS = {
    "fleet_efficiency": ["automated_demand_service_rate", "automated_throughput_per_sim_hour", "fleet_mean_automated_total_time", "fleet_total_automated_waiting_time"],
    "operational_safety": ["system_near_miss_events_per_100_vehicle_km", "system_collision_proxy_events_per_100_vehicle_km", "trajectory_conflicts_per_100_vehicle_km"],
    "cloud_coordination": ["feedback_hard_violation_reduction_rate", "feedback_route_conflict_reduction_rate", "cloud_fleet_decision_count", "shield_rejection_count", "qwen_fallback_count"],
    "mixed_human_traffic": ["human_like_vehicle_count", "replay_vehicle_count", "hidden_intent_vehicle_count"],
    "synchronous_decision_integrity": ["decision_barrier_ack_coverage_rate", "decision_barrier_failure_count", "decision_barrier_sim_time_mismatch_count"],
    "data_integrity": ["trace_integrity_ok", "trace_integrity_invalid_file_count", "trace_identity_conflict_count", "trace_time_regression_count", "trace_wall_time_regression_count", "trace_kinematic_jump_count"],
}

TRC_PRIMARY_TEST_METRICS = [
    "automated_demand_service_rate",
    "automated_throughput_per_sim_hour",
    "fleet_mean_automated_total_time",
    "fleet_total_automated_waiting_time",
    "system_near_miss_events_per_100_vehicle_km",
    "trajectory_conflicts_per_100_vehicle_km",
]

REQUIRED_REPORT_FILES = [
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
]

TRC_ANALYSIS_FILES = [
    "trc_metric_group_summary.csv",
    "trc_reference_improvements.csv",
    "trc_stratified_findings.csv",
    "trc_key_findings.md",
    "trc_analysis_manifest.json",
    "critical_states/window_metrics.csv",
    "critical_states/window_paired_deltas.csv",
    "critical_states/critical_window_rank.csv",
    "critical_states/window_metrics_manifest.json",
    "critical_states/critical_window_mechanism_evidence.csv",
    "critical_states/critical_metric_drivers.csv",
    "critical_states/critical_state_events.csv",
    "critical_states/critical_traffic_events.csv",
    "critical_states/critical_trajectory_intervals.csv",
    "critical_states/critical_state_analysis.md",
]


class Gate:
    def __init__(self) -> None:
        self.checks: List[Dict[str, Any]] = []

    def require(self, name: str, ok: bool, detail: str, evidence: Optional[Dict[str, Any]] = None) -> None:
        self.checks.append({
            "name": name,
            "ok": bool(ok),
            "detail": detail,
            "evidence": evidence or {},
        })

    @property
    def ok(self) -> bool:
        return all(item["ok"] for item in self.checks)

    @property
    def failures(self) -> List[Dict[str, Any]]:
        return [item for item in self.checks if not item["ok"]]


def _load_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    with path.open() as f:
        return json.load(f)


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _load_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _csv_values(rows: Iterable[Dict[str, str]], key: str) -> Set[str]:
    return {str(row.get(key)) for row in rows if row.get(key) not in (None, "")}


def _resolve_report_dir(suite_dir: Path, suite_manifest: Dict[str, Any], report_dir_arg: Optional[Path]) -> Path:
    if report_dir_arg is not None:
        return report_dir_arg.resolve()
    report = suite_manifest.get("paper_report") if isinstance(suite_manifest, dict) else None
    if isinstance(report, dict) and report.get("dir"):
        path = Path(str(report["dir"]))
        if path.is_absolute():
            return path
        cwd_candidate = path.resolve()
        if cwd_candidate.exists():
            return cwd_candidate
        suite_candidate = (suite_dir / path).resolve()
        if suite_candidate.exists():
            return suite_candidate
        return cwd_candidate
    return (suite_dir / "reports" / "paper_report").resolve()


def _profile_config(args: argparse.Namespace) -> Dict[str, Any]:
    config = dict(PROFILE_DEFAULTS[args.profile])
    if args.required_agents:
        config["required_agents"] = [item for item in args.required_agents.split(",") if item]
    for key in [
        "min_agents",
        "min_seeds",
        "min_background_modes",
        "min_density_labels",
        "min_spawn_profiles",
        "min_scenarios",
        "min_rows",
    ]:
        value = getattr(args, key)
        if value is not None:
            config[key] = value
    if args.require_real_qwen:
        config["require_real_qwen"] = True
    if args.allow_mock_qwen:
        config["require_real_qwen"] = False
    if args.require_video_evidence:
        config["require_video_evidence"] = True
    if args.skip_video_evidence:
        config["require_video_evidence"] = False
    if args.skip_manuscript:
        config["require_manuscript"] = False
    return config


def evaluate(suite_dir: Path, report_dir: Optional[Path], profile: str, config: Dict[str, Any]) -> Dict[str, Any]:
    suite_dir = suite_dir.resolve()
    suite_manifest_path = suite_dir / "suite_manifest.json"
    suite_manifest = _load_json(suite_manifest_path, {}) or {}
    resolved_report_dir = _resolve_report_dir(suite_dir, suite_manifest, report_dir)
    report_manifest_path = resolved_report_dir / "paper_report_manifest.json"
    repro_path = resolved_report_dir / "paper_reproducibility.json"
    report_manifest = _load_json(report_manifest_path, {}) or {}
    repro = _load_json(repro_path, {}) or {}
    rows = _load_csv(resolved_report_dir / "paper_rows.csv")
    summary_rows = _load_csv(resolved_report_dir / "paper_summary.csv")
    stats_rows = _load_csv(resolved_report_dir / "paper_statistical_tests.csv")
    strata_rows = _load_csv(resolved_report_dir / "paper_stratified_summary.csv")
    failures = _load_jsonl(suite_dir / "failures.jsonl")

    gate = Gate()
    gate.require("suite_manifest_exists", suite_manifest_path.exists(), "suite_manifest.json must exist", {"path": str(suite_manifest_path)})
    gate.require("suite_all_valid", bool(suite_manifest.get("all_valid")), "all suite benchmark validations must pass", {"all_valid": suite_manifest.get("all_valid")})
    gate.require("no_recorded_failures", len(failures) == 0, "failures.jsonl must be empty", {"failure_count": len(failures)})
    gate.require("report_manifest_exists", report_manifest_path.exists(), "paper_report_manifest.json must exist", {"path": str(report_manifest_path)})
    gate.require("reproducibility_manifest_exists", repro_path.exists(), "paper_reproducibility.json must exist", {"path": str(repro_path)})

    missing_files = [name for name in REQUIRED_REPORT_FILES if not (resolved_report_dir / name).exists()]
    gate.require("required_report_files", not missing_files, "all paper report artifacts must exist", {"missing": missing_files, "report_dir": str(resolved_report_dir)})
    if profile == "trc":
        missing_trc_analysis = [name for name in TRC_ANALYSIS_FILES if not (resolved_report_dir / name).exists()]
        gate.require("trc_csv_first_analysis_files", not missing_trc_analysis, "TR-C profile requires CSV-first analysis outputs generated from paper_report CSV files", {"missing": missing_trc_analysis, "report_dir": str(resolved_report_dir)})

    if bool(config.get("require_manuscript", False)):
        repo_root = Path(__file__).resolve().parents[3]
        missing_manuscripts = [name for name in REQUIRED_MANUSCRIPT_FILES if not (repo_root / name).exists()]
        manuscript_sizes = {name: (repo_root / name).stat().st_size if (repo_root / name).exists() else 0 for name in REQUIRED_MANUSCRIPT_FILES}
        gate.require("manuscript_sources", not missing_manuscripts and all(size > 500 for size in manuscript_sizes.values()), "journal/paper profile requires editable manuscript sources", {"missing": missing_manuscripts, "sizes": manuscript_sizes})

    if rows:
        metric_columns = set(rows[0].keys())
    else:
        metric_columns = set()
    missing_metric_columns = sorted(set(REQUIRED_METRIC_COLUMNS) - metric_columns)
    gate.require("required_metric_columns", not missing_metric_columns, "paper rows must include safety, efficiency, coordination, and synchronous-decision integrity metrics", {"missing": missing_metric_columns})
    invalid_integrity_rows = [
        "%s/%s" % (row.get("scenario_id"), row.get("agent_type"))
        for row in rows
        if not _as_bool(row.get("trace_integrity_ok"))
    ]
    gate.require(
        "trace_integrity",
        not invalid_integrity_rows,
        "every paper row must pass trace identity, monotonic-time, and kinematic-jump checks",
        {"invalid_rows": invalid_integrity_rows[:50], "invalid_count": len(invalid_integrity_rows)},
    )
    if bool(config.get("require_cloud_fleet", False)):
        missing_groups: Dict[str, List[str]] = {}
        for group, columns in TRC_METRIC_GROUPS.items():
            missing = [column for column in columns if column not in metric_columns]
            if missing:
                missing_groups[group] = missing
        gate.require("trc_csv_metric_groups", not missing_groups, "TR-C CSV-first evidence requires fleet efficiency, safety, coordination, mixed-human-traffic, and synchronous-decision integrity groups", {"missing_groups": missing_groups})

    agents = set(repro.get("agents") or _csv_values(rows, "agent_type"))
    required_agents = set(config["required_agents"])
    missing_agents = sorted(required_agents - agents)
    gate.require("required_agents", not missing_agents, "required baseline agents must be present", {"required": sorted(required_agents), "present": sorted(agents), "missing": missing_agents})
    gate.require("min_agents", len(agents) >= int(config["min_agents"]), "agent count must meet profile minimum", {"actual": len(agents), "minimum": config["min_agents"]})

    if bool(config.get("require_oracle_upper_bound", False)):
        gate.require("oracle_upper_bound_agent", "oracle_intent_bundle" in agents, "oracle intent upper-bound baseline must be present", {"present": sorted(agents)})
        gate.require("centralized_cost_agent", "centralized_min_cost" in agents, "centralized min-cost baseline must be present", {"present": sorted(agents)})

    seeds = set(repro.get("seeds") or _csv_values(rows, "seed"))
    backgrounds = set(repro.get("background_modes") or _csv_values(rows, "background_mode"))
    densities = set(repro.get("density_labels") or _csv_values(rows, "density_label"))
    spawn_profiles = set(repro.get("spawn_profiles") or _csv_values(rows, "spawn_profile"))
    gate.require("min_seeds", len(seeds) >= int(config["min_seeds"]), "seed count must meet profile minimum", {"actual": sorted(seeds), "minimum": config["min_seeds"]})
    gate.require("min_background_modes", len(backgrounds) >= int(config["min_background_modes"]), "background-mode coverage must meet profile minimum", {"actual": sorted(backgrounds), "minimum": config["min_background_modes"]})
    gate.require("min_density_labels", len(densities) >= int(config["min_density_labels"]), "density coverage must meet profile minimum", {"actual": sorted(densities), "minimum": config["min_density_labels"]})
    gate.require("min_spawn_profiles", len(spawn_profiles) >= int(config["min_spawn_profiles"]), "spawn-profile coverage must meet profile minimum", {"actual": sorted(spawn_profiles), "minimum": config["min_spawn_profiles"]})

    scenario_count = int(repro.get("scenario_count") or len(_csv_values(rows, "pair_key")))
    row_count = int(repro.get("row_count") or len(rows))
    gate.require("min_scenarios", scenario_count >= int(config["min_scenarios"]), "paired scenario count must meet profile minimum", {"actual": scenario_count, "minimum": config["min_scenarios"]})
    gate.require("min_rows", row_count >= int(config["min_rows"]), "metric-row count must meet profile minimum", {"actual": row_count, "minimum": config["min_rows"]})

    reference_agent = str(repro.get("reference_agent") or report_manifest.get("reference_agent") or "qwen_vla")
    non_reference_agents = sorted(agent for agent in agents if agent != reference_agent)
    expected_paired_delta_count = scenario_count * len(non_reference_agents)
    paired_delta_count = int(report_manifest.get("paired_delta_count") or 0)
    gate.require("paired_delta_coverage", paired_delta_count >= expected_paired_delta_count, "paired deltas must cover every non-reference agent and scenario", {"actual": paired_delta_count, "expected": expected_paired_delta_count})

    required_test_metrics = TRC_PRIMARY_TEST_METRICS if profile == "trc" else ["objective_score"]
    stats_pairs = {(row.get("agent_type"), row.get("metric")) for row in stats_rows}
    missing_test_pairs = sorted(
        "%s/%s" % (agent, metric)
        for agent in non_reference_agents
        for metric in required_test_metrics
        if (agent, metric) not in stats_pairs
    )
    gate.require("primary_statistical_tests", not missing_test_pairs and bool(stats_rows), "primary metrics must have paired statistical tests for each non-reference method", {"missing": missing_test_pairs, "statistical_rows": len(stats_rows)})
    if profile == "trc":
        required_stat_columns = {
            "delta_bootstrap_ci95_low",
            "delta_bootstrap_ci95_high",
            "paired_permutation_p",
            "paired_permutation_p_holm",
            "paired_permutation_p_holm_reject_0_05",
        }
        stat_columns = set(stats_rows[0].keys()) if stats_rows else set()
        missing_stat_columns = sorted(required_stat_columns - stat_columns)
        gate.require("trc_statistical_schema", not missing_stat_columns, "TR-C statistics require paired bootstrap, permutation tests, and Holm correction", {"missing": missing_stat_columns})
    strata_factors = {row.get("factor") for row in strata_rows}
    missing_strata = sorted(set(["background_mode", "density_label", "spawn_profile"]) - strata_factors)
    gate.require("stratified_summaries", not missing_strata and bool(strata_rows), "scenario-factor stratified summaries must exist", {"missing_factors": missing_strata, "stratified_rows": len(strata_rows)})

    qwen = suite_manifest.get("qwen") if isinstance(suite_manifest, dict) else {}
    qwen_health = qwen.get("health") if isinstance(qwen, dict) else None
    qwen_ok = bool(isinstance(qwen_health, dict) and qwen_health.get("ok"))
    gate.require("qwen_health_ok", qwen_ok, "Qwen health payload must be present and ok", {"qwen": qwen})
    qwen_decision_logs = list(suite_dir.glob("**/qwen_vla_decisions.jsonl"))
    gate.require("qwen_decision_logs", bool(qwen_decision_logs), "Qwen/baseline decision logs must be present for replayable audit", {"log_count": len(qwen_decision_logs)})
    if bool(config.get("require_cloud_fleet", False)):
        fleet_protocol_records = 0
        fleet_vehicle_decisions = 0
        for row in rows:
            fleet_protocol_records += int(_safe_float(row.get("cloud_fleet_decision_count")))
            fleet_vehicle_decisions += int(_safe_float(row.get("cloud_fleet_vehicle_decision_count")))
        gate.require("cloud_fleet_decision_logs", fleet_protocol_records > 0 and fleet_vehicle_decisions > 0, "TR-C profile requires cloud fleet decision packets and fleet vehicle decisions in CSV evidence", {"cloud_fleet_decision_count": fleet_protocol_records, "cloud_fleet_vehicle_decision_count": fleet_vehicle_decisions})
        synchronous_agents = {"qwen_vla", "mllm_direct", "mllm_self_reflect", "mllm_external_feedback", "fleet_min_cost"}
        invalid_barrier_rows = [
            "%s/%s" % (row.get("scenario_id"), row.get("agent_type"))
            for row in rows
            if row.get("agent_type") in synchronous_agents and (
                _safe_float(row.get("decision_barrier_epoch_count")) <= 0
                or _safe_float(row.get("decision_barrier_ack_coverage_rate")) < 1.0 - 1e-9
                or _safe_float(row.get("decision_barrier_failure_count")) > 0
                or _safe_float(row.get("decision_barrier_sim_time_mismatch_count")) > 0
                or row.get("simulation_time_policy") != "decision_then_advance"
                or _as_bool(row.get("wall_clock_latency_in_performance_metrics"))
            )
        ]
        gate.require(
            "synchronous_decision_barrier",
            not invalid_barrier_rows,
            "every cloud-fleet run must apply all decisions at the frozen decision time before simulation advances",
            {"invalid_rows": invalid_barrier_rows[:50], "invalid_count": len(invalid_barrier_rows)},
        )
    decision_audits = list(suite_dir.glob("**/decision_audit.json"))
    gate.require("decision_audit_artifacts", bool(decision_audits), "decision audit artifacts must be present", {"audit_count": len(decision_audits)})
    if bool(config.get("require_video_evidence", False)):
        video_manifests = list(suite_dir.glob("**/video_manifest.json")) + list(resolved_report_dir.glob("**/video_manifest.json"))
        gate.require("video_evidence", bool(video_manifests), "journal profile requires visualizer video/GIF evidence manifests", {"video_manifest_count": len(video_manifests)})
    if bool(config["require_real_qwen"]):
        is_mock = bool(isinstance(qwen_health, dict) and qwen_health.get("mock"))
        gate.require("real_qwen_required", qwen_ok and not is_mock, "paper profile requires real Qwen, not mock mode", {"qwen_health": qwen_health})
    if bool(config.get("require_open_science", False)):
        repo_root = Path(__file__).resolve().parents[3]
        protocol_path = repo_root / "python" / "parksim" / "vla" / "benchmark_protocol.json"
        missing_open_files = [
            str(path) for path in [
                resolved_report_dir / "paper_rows.csv",
                resolved_report_dir / "paper_reproducibility.json",
                resolved_report_dir / "paper_report_manifest.json",
                suite_dir / "suite_config.json",
                suite_dir / "suite_manifest.json",
            ]
            if not path.exists()
        ]
        gate.require("open_science_protocol", protocol_path.exists(), "TR-C profile requires an explicit benchmark protocol for transferability and benchmarking", {"path": str(protocol_path)})
        gate.require("open_science_csv_json_exports", not missing_open_files, "TR-C profile requires reusable CSV/JSON result exports before manuscript writing", {"missing": missing_open_files})

    reference_summary = next((row for row in summary_rows if row.get("agent_type") == reference_agent), None)
    if reference_summary:
        unsafe = _safe_float(reference_summary.get("unsafe_occupancy_action_count_mean"))
        collision = _safe_float(reference_summary.get("collision_proxy_event_count_mean"))
        gate.require("reference_unsafe_actions", unsafe <= float(config["max_reference_unsafe_actions"]), "reference agent must not apply unsafe occupied-spot actions", {"agent": reference_agent, "actual": unsafe, "maximum": config["max_reference_unsafe_actions"]})
        gate.require("reference_collision_proxy", collision <= float(config["max_reference_collision_proxy"]), "reference agent collision-proxy mean must stay within profile limit", {"agent": reference_agent, "actual": collision, "maximum": config["max_reference_collision_proxy"]})
    else:
        gate.require("reference_summary", False, "reference agent must exist in paper_summary.csv", {"reference_agent": reference_agent})

    return {
        "type": "parksim_vla_paper_gate",
        "profile": profile,
        "ok": gate.ok,
        "suite_dir": str(suite_dir),
        "report_dir": str(resolved_report_dir),
        "config": config,
        "summary": {
            "check_count": len(gate.checks),
            "failure_count": len(gate.failures),
            "agents": sorted(agents),
            "seeds": sorted(seeds),
            "background_modes": sorted(backgrounds),
            "density_labels": sorted(densities),
            "spawn_profiles": sorted(spawn_profiles),
            "scenario_count": scenario_count,
            "row_count": row_count,
            "paired_delta_count": paired_delta_count,
            "statistical_rows": len(stats_rows),
            "stratified_rows": len(strata_rows),
        },
        "checks": gate.checks,
        "failures": gate.failures,
    }


def write_outputs(out_dir: Path, result: Dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "paper_gate.json").write_text(json.dumps(result, indent=2) + "\n")
    lines = [
        "# ParkSim-VLA Paper Gate",
        "",
        "- Profile: `%s`" % result["profile"],
        "- Status: `%s`" % ("pass" if result["ok"] else "fail"),
        "- Suite: `%s`" % result["suite_dir"],
        "- Report: `%s`" % result["report_dir"],
        "",
        "## Coverage",
        "",
    ]
    summary = result["summary"]
    for key in ["agents", "seeds", "background_modes", "density_labels", "spawn_profiles"]:
        values = summary.get(key, [])
        lines.append("- %s: %s" % (key, ", ".join(values) if values else "none"))
    lines.extend([
        "- scenarios: %s" % summary.get("scenario_count"),
        "- rows: %s" % summary.get("row_count"),
        "",
        "## Failed Checks",
        "",
    ])
    if result["failures"]:
        for item in result["failures"]:
            lines.append("- `%s`: %s" % (item["name"], item["detail"]))
            if item.get("evidence"):
                lines.append("  Evidence: `%s`" % json.dumps(item["evidence"], sort_keys=True))
    else:
        lines.append("- none")
    (out_dir / "paper_gate.md").write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Check whether a ParkSim-VLA suite satisfies pilot or paper reproduction requirements.")
    parser.add_argument("suite_dir", type=Path)
    parser.add_argument("--report-dir", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--profile", choices=sorted(PROFILE_DEFAULTS), default="pilot")
    parser.add_argument("--required-agents", default="")
    parser.add_argument("--min-agents", type=int)
    parser.add_argument("--min-seeds", type=int)
    parser.add_argument("--min-background-modes", type=int)
    parser.add_argument("--min-density-labels", type=int)
    parser.add_argument("--min-spawn-profiles", type=int)
    parser.add_argument("--min-scenarios", type=int)
    parser.add_argument("--min-rows", type=int)
    parser.add_argument("--require-real-qwen", action="store_true")
    parser.add_argument("--allow-mock-qwen", action="store_true")
    parser.add_argument("--require-video-evidence", action="store_true")
    parser.add_argument("--skip-video-evidence", action="store_true")
    parser.add_argument("--skip-manuscript", action="store_true")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()

    config = _profile_config(args)
    suite_dir = args.suite_dir.resolve()
    result = evaluate(suite_dir, args.report_dir, args.profile, config)
    out_dir = args.out_dir.resolve() if args.out_dir else suite_dir / "reports" / "paper_gate"
    write_outputs(out_dir, result)
    print("paper_gate=%s profile=%s ok=%s failures=%d" % (out_dir, args.profile, result["ok"], len(result["failures"])))
    if args.strict and not result["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
