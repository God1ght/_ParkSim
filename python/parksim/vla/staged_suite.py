import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Set


FORMAL_AGENTS = [
    "rule_based",
    "risk_aware_rule",
    "reservation_bundle",
    "centralized_min_cost",
    "oracle_intent_bundle",
    "fleet_min_cost",
    "mllm_direct",
    "mllm_self_reflect",
    "mllm_external_feedback",
]
SYNCHRONOUS_AGENTS = {
    "fleet_min_cost",
    "mllm_direct",
    "mllm_self_reflect",
    "mllm_external_feedback",
}
STAGE_SPECS: Dict[str, Dict[str, Any]] = {
    "core": {
        "seeds": {"0", "1", "2", "3", "4"},
        "scenario_pairs": {("mixed", "medium")},
    },
    "mixed_density": {
        "seeds": {"0", "1", "2", "3", "4"},
        "scenario_pairs": {("mixed", "low"), ("mixed", "high")},
    },
    "human_behavior": {
        "seeds": {"0", "1", "2", "3", "4"},
        "scenario_pairs": {
            ("rule_random", "low"),
            ("rule_random", "medium"),
            ("rule_random", "high"),
            ("replay", "replay"),
        },
    },
    "replication": {
        "seeds": {"5", "6", "7", "8", "9"},
        "scenario_pairs": {
            ("mixed", "low"),
            ("mixed", "medium"),
            ("mixed", "high"),
            ("rule_random", "low"),
            ("rule_random", "medium"),
            ("rule_random", "high"),
            ("replay", "replay"),
        },
    },
}


def _load_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return default


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return rows
    for line in lines:
        if not line.strip():
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            rows.append(item)
    return rows


def _load_csv(path: Path) -> List[Dict[str, str]]:
    try:
        with path.open(newline="") as handle:
            return list(csv.DictReader(handle))
    except OSError:
        return []


def _as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _as_float(value: Any, default: float = float("nan")) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _report_dir(suite_dir: Path, manifest: Dict[str, Any]) -> Path:
    report = manifest.get("paper_report", {}) if isinstance(manifest, dict) else {}
    raw = str(report.get("dir", ""))
    return Path(raw).resolve() if raw else (suite_dir / "reports" / "stage_report").resolve()


def validate_stage(suite_dir: Path, stage: str) -> Dict[str, Any]:
    spec = STAGE_SPECS[stage]
    manifest = _load_json(suite_dir / "suite_manifest.json", {})
    config = _load_json(suite_dir / "suite_config.json", {})
    gate = _load_json(suite_dir / "reports" / "paper_gate" / "paper_gate.json", {})
    rows = _load_csv(_report_dir(suite_dir, manifest) / "paper_rows.csv")
    expected_agents = set(FORMAL_AGENTS)
    expected_seeds: Set[str] = spec["seeds"]
    expected_pairs = spec["scenario_pairs"]
    actual_agents = {row.get("agent_type", "") for row in rows if row.get("agent_type")}
    actual_seeds = {row.get("seed", "") for row in rows if row.get("seed")}
    actual_pairs = {
        (row.get("background_mode", ""), row.get("density_label", ""))
        for row in rows
    }
    expected_rows = len(expected_agents) * len(expected_seeds) * len(expected_pairs)
    duplicate_keys = Counter(
        (
            row.get("agent_type", ""),
            row.get("seed", ""),
            row.get("background_mode", ""),
            row.get("density_label", ""),
        )
        for row in rows
    )
    duplicates = [list(key) for key, count in duplicate_keys.items() if count != 1]
    synchronous_failures = [
        "%s/%s/%s/%s"
        % (
            row.get("agent_type"),
            row.get("seed"),
            row.get("background_mode"),
            row.get("density_label"),
        )
        for row in rows
        if row.get("agent_type") in SYNCHRONOUS_AGENTS
        and (
            row.get("simulation_time_policy") != "decision_then_advance"
            or _as_bool(row.get("wall_clock_latency_in_performance_metrics"))
            or not math.isclose(
                _as_float(row.get("decision_barrier_ack_coverage_rate")),
                1.0,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            or _as_float(row.get("decision_barrier_epoch_count"), 0.0) <= 0.0
            or _as_float(row.get("decision_barrier_failure_count"), float("inf")) > 0.0
            or _as_float(row.get("decision_barrier_sim_time_mismatch_count"), float("inf")) > 0.0
        )
    ]
    checks = {
        "suite_all_valid": bool(manifest.get("all_valid")),
        "calibration_gate_passed": bool(gate.get("ok")),
        "duration_3600s": str(config.get("duration")) == "3600s",
        "real_qwen": str(config.get("qwen_mode")) == "real",
        "reference_agent": str(config.get("reference_agent")) == "mllm_external_feedback",
        "exact_agents": actual_agents == expected_agents,
        "exact_seeds": actual_seeds == expected_seeds,
        "exact_scenarios": actual_pairs == expected_pairs,
        "exact_row_count": len(rows) == expected_rows,
        "one_row_per_cell": not duplicates,
        "synchronous_decision_integrity": not synchronous_failures,
    }
    return {
        "type": "parksim_vla_trc_stage_validation",
        "stage": stage,
        "ok": all(checks.values()),
        "suite_dir": str(suite_dir.resolve()),
        "expected": {
            "agents": sorted(expected_agents),
            "seeds": sorted(expected_seeds),
            "scenario_pairs": sorted([list(item) for item in expected_pairs]),
            "row_count": expected_rows,
        },
        "actual": {
            "agents": sorted(actual_agents),
            "seeds": sorted(actual_seeds),
            "scenario_pairs": sorted([list(item) for item in actual_pairs]),
            "row_count": len(rows),
        },
        "checks": checks,
        "duplicates": duplicates,
        "synchronous_failures": synchronous_failures,
    }


def _write_stage_validation(suite_dir: Path, result: Dict[str, Any]) -> None:
    (suite_dir / "stage_validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    )
    lines = [
        "# TR-C 分组实验门控",
        "",
        "- 分组：`%s`" % result["stage"],
        "- 状态：`%s`" % ("通过" if result["ok"] else "失败"),
        "- 实际/预期结果行：%d/%d"
        % (result["actual"]["row_count"], result["expected"]["row_count"]),
        "",
        "## 检查项",
        "",
    ]
    for name, passed in result["checks"].items():
        lines.append("- %s：%s" % (name, "通过" if passed else "失败"))
    (suite_dir / "stage_validation_zh.md").write_text("\n".join(lines) + "\n")


def _unique(values: Iterable[Any]) -> List[str]:
    return sorted({str(value) for value in values if str(value)})


def prepare_aggregate(out_dir: Path, stage_dirs: Sequence[Path]) -> Dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "reports" / "paper_report").mkdir(parents=True, exist_ok=True)
    benchmark_rows: List[Dict[str, Any]] = []
    failures: List[Dict[str, Any]] = []
    stage_records: List[Dict[str, Any]] = []
    configs: List[Dict[str, Any]] = []
    health_records: List[Dict[str, Any]] = []
    for stage_dir in stage_dirs:
        stage_name = stage_dir.name
        if stage_name not in STAGE_SPECS:
            raise ValueError("Unknown stage directory name: %s" % stage_name)
        validation = validate_stage(stage_dir, stage_name)
        if not validation["ok"]:
            raise ValueError("Stage validation failed: %s" % stage_dir)
        manifest = _load_json(stage_dir / "suite_manifest.json", {})
        configs.append(_load_json(stage_dir / "suite_config.json", {}))
        qwen = manifest.get("qwen", {}) if isinstance(manifest, dict) else {}
        health = qwen.get("health") if isinstance(qwen, dict) else None
        if isinstance(health, dict):
            health_records.append(health)
        for item in _load_jsonl(stage_dir / "benchmarks.jsonl"):
            enriched = dict(item)
            enriched["stage"] = stage_name
            benchmark_rows.append(enriched)
        for item in _load_jsonl(stage_dir / "failures.jsonl"):
            enriched = dict(item)
            enriched["stage"] = stage_name
            failures.append(enriched)
        stage_records.append({
            "stage": stage_name,
            "suite_dir": str(stage_dir.resolve()),
            "validation": str((stage_dir / "stage_validation.json").resolve()),
        })
    stage_names = [row["stage"] for row in stage_records]
    stage_set_complete = len(stage_names) == len(STAGE_SPECS) and set(stage_names) == set(STAGE_SPECS)
    benchmark_dirs = _unique(row.get("benchmark_dir", "") for row in benchmark_rows)
    aggregate_health = {
        "ok": bool(health_records) and all(bool(row.get("ok")) for row in health_records),
        "mock": any(bool(row.get("mock")) for row in health_records),
        "stage_health_count": len(health_records),
        "stage_health": health_records,
    }
    config = {
        "type": "parksim_vla_trc_staged_aggregate",
        "duration": "3600s",
        "agents": FORMAL_AGENTS,
        "seeds": [str(value) for value in range(10)],
        "background_modes": ["mixed", "replay", "rule_random"],
        "density_labels": ["high", "low", "medium", "replay"],
        "reference_agent": "mllm_external_feedback",
        "qwen_mode": "real",
        "simulation_time_policy": "decision_then_advance",
        "wall_clock_latency_in_performance_metrics": False,
        "stage_suites": stage_records,
        "source_configs": configs,
    }
    status_counts = Counter(str(row.get("status", "unknown")) for row in benchmark_rows)
    all_valid = stage_set_complete and bool(benchmark_rows) and not failures and all(
        bool(row.get("validation_ok", row.get("status") in {"completed", "skipped_valid"}))
        for row in benchmark_rows
    )
    manifest = {
        "type": "parksim_vla_trc_staged_aggregate",
        "out_dir": str(out_dir.resolve()),
        "benchmark_count": len(benchmark_rows),
        "status_counts": dict(status_counts),
        "benchmarks": benchmark_rows,
        "all_valid": all_valid,
        "paper_report": {
            "dir": str((out_dir / "reports" / "paper_report").resolve()),
            "summary_md": str((out_dir / "reports" / "paper_report" / "paper_summary.md").resolve()),
            "table_tex": str((out_dir / "reports" / "paper_report" / "paper_table.tex").resolve()),
            "manifest": str((out_dir / "reports" / "paper_report" / "paper_report_manifest.json").resolve()),
        },
        "qwen": {
            "mode": "real",
            "required": "1",
            "health": aggregate_health,
        },
        "stages": stage_records,
    }
    with (out_dir / "benchmarks.jsonl").open("w") as handle:
        for row in benchmark_rows:
            handle.write(json.dumps(row) + "\n")
    with (out_dir / "failures.jsonl").open("w") as handle:
        for row in failures:
            handle.write(json.dumps(row) + "\n")
    (out_dir / "benchmark_dirs.txt").write_text("\n".join(benchmark_dirs) + "\n")
    (out_dir / "suite_config.json").write_text(json.dumps(config, indent=2) + "\n")
    (out_dir / "suite_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (out_dir / "qwen_health.json").write_text(json.dumps(aggregate_health, indent=2) + "\n")
    return {
        "type": "parksim_vla_trc_aggregate_preparation",
        "ok": all_valid and aggregate_health["ok"] and not aggregate_health["mock"],
        "out_dir": str(out_dir.resolve()),
        "stage_count": len(stage_records),
        "stage_set_complete": stage_set_complete,
        "benchmark_count": len(benchmark_rows),
        "benchmark_dir_count": len(benchmark_dirs),
        "failure_count": len(failures),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate and aggregate staged ParkSim TR-C suites.")
    sub = parser.add_subparsers(dest="command", required=True)
    validate = sub.add_parser("validate-stage")
    validate.add_argument("suite_dir", type=Path)
    validate.add_argument("--stage", choices=sorted(STAGE_SPECS), required=True)
    validate.add_argument("--strict", action="store_true")
    aggregate = sub.add_parser("prepare-aggregate")
    aggregate.add_argument("out_dir", type=Path)
    aggregate.add_argument("--inputs", nargs="+", type=Path, required=True)
    aggregate.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    if args.command == "validate-stage":
        result = validate_stage(args.suite_dir.resolve(), args.stage)
        _write_stage_validation(args.suite_dir.resolve(), result)
        print("stage_validation=%s stage=%s ok=%s" % (args.suite_dir, args.stage, result["ok"]))
    else:
        result = prepare_aggregate(args.out_dir.resolve(), [path.resolve() for path in args.inputs])
        (args.out_dir / "aggregate_preparation.json").write_text(json.dumps(result, indent=2) + "\n")
        print("aggregate_preparation=%s ok=%s" % (args.out_dir, result["ok"]))
    if args.strict and not result["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
