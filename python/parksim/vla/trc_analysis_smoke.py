import csv
import json
import tempfile
from pathlib import Path

from parksim.vla.trc_analysis import analyze
from parksim.vla.paper_report import paired_deltas, statistical_tests, summarize, summarize_deltas, write_markdown
from parksim.vla.safety_metrics import collect_system_traffic_metrics


def _write_csv(path, rows):
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main():
    sparse_rows = [
        {"agent_type": "baseline", "pair_key": "s0", "available": 2.0},
        {"agent_type": "reference", "pair_key": "s0", "available": 1.0},
    ]
    sparse_summary = summarize(sparse_rows, ["available", "missing"])
    assert all("missing_mean" not in row for row in sparse_summary)
    sparse_deltas = paired_deltas(sparse_rows, "reference", ["available", "missing"])
    assert sparse_deltas[0]["available_delta_vs_ref"] == 1.0
    assert "missing_delta_vs_ref" not in sparse_deltas[0]
    assert all(row["metric"] != "missing" for row in statistical_tests(sparse_deltas, ["available", "missing"]))
    assert "missing_delta_vs_ref_mean" not in summarize_deltas(sparse_deltas, ["available", "missing"])[0]

    with tempfile.TemporaryDirectory() as sparse_tmp:
        write_markdown(
            Path(sparse_tmp) / "sparse.md",
            sparse_summary,
            summarize_deltas(sparse_deltas, ["available", "missing"]),
            statistical_tests(sparse_deltas, ["available", "missing"]),
            [],
            {},
            "reference",
        )
        assert (Path(sparse_tmp) / "sparse.md").exists()

    with tempfile.TemporaryDirectory() as safety_tmp:
        log_dir = Path(safety_tmp)
        (log_dir / "traffic_schedule.json").write_text(json.dumps({
            "long_horizon_duration": 3600.0,
            "events": [
                {"event_id": "av0", "actor_class": "av", "event_type": "entering"},
                {"event_id": "av1", "actor_class": "av", "event_type": "entering"},
                {"event_id": "av2", "actor_class": "av", "event_type": "exiting"},
            ],
        }))
        for vehicle_id, completed in ((1, True), (2, False)):
            (log_dir / ("vehicle_%d_summary.json" % vehicle_id)).write_text(json.dumps({
                "vehicle_id": vehicle_id,
                "agent_type": "mllm_external_feedback",
                "vehicle_role": "av_entering",
                "intent_observable": True,
                "completed": completed,
                "censored": not completed,
                "total_time": 100.0,
            }))
        traffic_metrics = collect_system_traffic_metrics(log_dir)
        assert traffic_metrics["automated_demand_released_count"] == 3
        assert traffic_metrics["automated_demand_completed_count"] == 1
        assert traffic_metrics["automated_demand_backlog_count"] == 2
        assert abs(traffic_metrics["automated_demand_service_rate"] - 1.0 / 3.0) < 1e-12
        assert traffic_metrics["automated_throughput_per_sim_hour"] == 1.0

    with tempfile.TemporaryDirectory() as tmp:
        report = Path(tmp) / "report"
        out = Path(tmp) / "analysis"
        report.mkdir()
        _write_csv(report / "paper_summary.csv", [
            {
                "agent_type": "rule_based",
                "episodes": 10,
                "automated_demand_service_rate_mean": 0.70,
                "automated_demand_service_rate_std": 0.10,
                "automated_demand_service_rate_ci95": 0.06,
                "fleet_mean_automated_waiting_time_mean": 20.0,
                "fleet_mean_automated_waiting_time_std": 4.0,
                "fleet_mean_automated_waiting_time_ci95": 2.5,
                "cloud_fleet_event_trigger_epoch_count_mean": 8.0,
            },
            {
                "agent_type": "mllm_external_feedback",
                "episodes": 10,
                "automated_demand_service_rate_mean": 0.80,
                "automated_demand_service_rate_std": 0.08,
                "automated_demand_service_rate_ci95": 0.05,
                "fleet_mean_automated_waiting_time_mean": 18.0,
                "fleet_mean_automated_waiting_time_std": 3.0,
                "fleet_mean_automated_waiting_time_ci95": 2.0,
                "cloud_fleet_event_trigger_epoch_count_mean": 9.0,
            },
        ])
        _write_csv(report / "paper_paired_delta_summary.csv", [{
            "agent_type": "rule_based",
            "paired_count": 10,
            "automated_demand_service_rate_delta_vs_ref_mean": -0.10,
            "automated_demand_service_rate_delta_vs_ref_ci95": 0.04,
            "fleet_mean_automated_waiting_time_delta_vs_ref_mean": 2.0,
            "fleet_mean_automated_waiting_time_delta_vs_ref_ci95": 1.0,
            "cloud_fleet_event_trigger_epoch_count_delta_vs_ref_mean": -1.0,
            "cloud_fleet_event_trigger_epoch_count_delta_vs_ref_ci95": 0.0,
        }])
        _write_csv(report / "paper_statistical_tests.csv", [
            {
                "agent_type": "rule_based",
                "metric": "automated_demand_service_rate",
                "effect_dz": -1.1,
                "paired_permutation_p": 0.01,
                "paired_permutation_p_holm": 0.03,
            },
            {
                "agent_type": "rule_based",
                "metric": "fleet_mean_automated_waiting_time",
                "effect_dz": 0.9,
                "paired_permutation_p": 0.02,
                "paired_permutation_p_holm": 0.06,
            },
        ])
        _write_csv(report / "paper_stratified_summary.csv", [
            {
                "factor": "density_label",
                "level": "medium",
                "agent_type": "rule_based",
                "automated_demand_service_rate_mean": 0.70,
                "fleet_mean_automated_waiting_time_mean": 20.0,
            },
            {
                "factor": "density_label",
                "level": "medium",
                "agent_type": "mllm_external_feedback",
                "automated_demand_service_rate_mean": 0.80,
                "fleet_mean_automated_waiting_time_mean": 18.0,
            },
        ])
        (report / "paper_reproducibility.json").write_text(json.dumps({
            "row_count": 20,
            "agent_count": 2,
            "scenario_count": 10,
            "seeds": list(range(10)),
            "background_modes": ["mixed"],
            "density_labels": ["medium"],
        }))

        manifest = analyze(report, out, "mllm_external_feedback")
        assert manifest["qwen_wall_clock_latency_in_performance_metrics"] is False
        assert "No sums" in manifest["aggregation_policy"]
        metric_rows = _read_csv(out / "trc_metric_group_summary.csv")
        assert metric_rows and all(not key.endswith("_score") for key in metric_rows[0])
        comparisons = _read_csv(out / "trc_reference_improvements.csv")
        by_metric = {row["metric"]: row for row in comparisons}
        assert float(by_metric["automated_demand_service_rate"]["reference_improvement"]) == 0.10
        assert by_metric["automated_demand_service_rate"]["holm_significant_0_05"] == "True"
        assert float(by_metric["fleet_mean_automated_waiting_time"]["reference_improvement"]) == 2.0
        assert by_metric["fleet_mean_automated_waiting_time"]["holm_significant_0_05"] == "False"
        assert by_metric["cloud_fleet_event_trigger_epoch_count"]["reference_improvement"] == ""
        assert "不作方法优劣判定" in by_metric["cloud_fleet_event_trigger_epoch_count"]["conclusion_zh"]
        definitions = _read_csv(out / "trc_metric_definitions_zh.csv")
        assert definitions and all(row["Qwen墙钟响应时间是否参与"] == "False" for row in definitions)
        findings = (out / "trc_key_findings_zh.md").read_text()
        assert "主要指标配对比较" in findings
        assert "不会自动修改 TeX" in findings
        assert "Qwen 墙钟响应时间不进入" in findings
    print("parksim.vla TR-C Chinese group analysis smoke ok")


if __name__ == "__main__":
    main()
