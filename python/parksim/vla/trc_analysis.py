import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


GROUP_NAMES_ZH = {
    "fleet_efficiency": "车队运行效率",
    "operational_safety": "运行安全",
    "cloud_coordination": "云端协调",
    "cloud_invocation_budget": "云端调用预算",
    "mixed_traffic_scale": "混合交通规模",
    "synchronous_decision_integrity": "同步决策完整性",
    "data_integrity": "数据完整性",
}

# All entries are simulator-time or simulator-state quantities. Qwen wall-clock
# latency is deliberately absent because the simulator is frozen during inference.
METRIC_CATALOG: List[Dict[str, Any]] = [
    {
        "metric": "automated_demand_service_rate",
        "group": "fleet_efficiency",
        "name_zh": "自动驾驶需求服务率",
        "definition_zh": "3600 s 内完成的 AV 进场或离场任务数除以已释放 AV 需求数。",
        "unit": "比例",
        "direction": "higher_is_better",
        "primary": True,
    },
    {
        "metric": "automated_throughput_per_sim_hour",
        "group": "fleet_efficiency",
        "name_zh": "自动驾驶吞吐量",
        "definition_zh": "每仿真小时完成的 AV 进场或离场任务数。",
        "unit": "辆/仿真小时",
        "direction": "higher_is_better",
        "primary": True,
    },
    {
        "metric": "automated_demand_backlog_count",
        "group": "fleet_efficiency",
        "name_zh": "自动驾驶未服务需求数",
        "definition_zh": "评估时域结束时已释放但尚未完成的 AV 任务数。",
        "unit": "辆",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "fleet_mean_automated_total_time",
        "group": "fleet_efficiency",
        "name_zh": "AV 平均任务历时",
        "definition_zh": "从任务释放到完成或右删失时刻的逐 AV 平均仿真时间。",
        "unit": "s/辆",
        "direction": "lower_is_better",
        "primary": True,
    },
    {
        "metric": "fleet_mean_automated_waiting_time",
        "group": "fleet_efficiency",
        "name_zh": "AV 平均等待时间",
        "definition_zh": "逐 AV 因让行或阻塞进入等待状态的累计仿真时间均值。",
        "unit": "s/辆",
        "direction": "lower_is_better",
        "primary": True,
    },
    {
        "metric": "fleet_mean_automated_path_length",
        "group": "fleet_efficiency",
        "name_zh": "AV 平均路径长度",
        "definition_zh": "逐 AV 在固定仿真时域内实际执行轨迹长度的均值。",
        "unit": "m/辆",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "objective_score",
        "group": "fleet_efficiency",
        "name_zh": "辅助系统代价",
        "definition_zh": "预冻结权重下路径、时间、安全和未服务需求的辅助综合代价；不作为唯一主要终点。",
        "unit": "无量纲",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "system_near_miss_events_per_100_vehicle_km",
        "group": "operational_safety",
        "name_zh": "每百车公里近失事件率",
        "definition_zh": "系统近失事件数按全部车辆累计行驶暴露量归一化至每 100 车公里。",
        "unit": "次/100车公里",
        "direction": "lower_is_better",
        "primary": True,
    },
    {
        "metric": "system_collision_proxy_events_per_100_vehicle_km",
        "group": "operational_safety",
        "name_zh": "每百车公里碰撞代理事件率",
        "definition_zh": "系统碰撞代理事件数按全部车辆累计行驶暴露量归一化至每 100 车公里。",
        "unit": "次/100车公里",
        "direction": "lower_is_better",
        "primary": True,
    },
    {
        "metric": "trajectory_conflicts_per_100_vehicle_km",
        "group": "operational_safety",
        "name_zh": "每百车公里轨迹冲突率",
        "definition_zh": "候选或执行轨迹时间窗冲突数按全部车辆行驶暴露量归一化。",
        "unit": "次/100车公里",
        "direction": "lower_is_better",
        "primary": True,
    },
    {
        "metric": "mixed_intent_conflicts_per_100_vehicle_km",
        "group": "operational_safety",
        "name_zh": "每百车公里混合意图冲突率",
        "definition_zh": "涉及隐藏路径或目标泊位意图的人类车辆冲突数按行驶暴露量归一化。",
        "unit": "次/100车公里",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "system_min_distance_m",
        "group": "operational_safety",
        "name_zh": "系统最小车辆间距",
        "definition_zh": "剧集内任意两辆动态车辆之间观测到的最小欧氏距离。",
        "unit": "m",
        "direction": "higher_is_better",
        "primary": False,
    },
    {
        "metric": "feedback_repair_success_rate",
        "group": "cloud_coordination",
        "name_zh": "反馈修复成功率",
        "definition_zh": "外部反馈尝试中，经一次有界修正后通过 Fleet Critic 的比例。",
        "unit": "比例",
        "direction": "higher_is_better",
        "primary": False,
    },
    {
        "metric": "feedback_hard_violation_reduction_rate",
        "group": "cloud_coordination",
        "name_zh": "反馈硬约束违规削减率",
        "definition_zh": "反馈前后重复泊位、不可达动作等硬约束问题的相对减少比例。",
        "unit": "比例",
        "direction": "higher_is_better",
        "primary": False,
    },
    {
        "metric": "feedback_route_conflict_reduction_rate",
        "group": "cloud_coordination",
        "name_zh": "反馈路径冲突削减率",
        "definition_zh": "反馈前后候选路径时间窗冲突数量的相对减少比例。",
        "unit": "比例",
        "direction": "higher_is_better",
        "primary": False,
    },
    {
        "metric": "shield_rejection_count",
        "group": "cloud_coordination",
        "name_zh": "安全盾拒绝次数",
        "definition_zh": "模型或规则提案因占用、预约、可达性或系统冲突被 Safety Shield 拒绝的次数。",
        "unit": "次",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "qwen_fallback_count",
        "group": "cloud_coordination",
        "name_zh": "确定性回退次数",
        "definition_zh": "MLLM 输出缺失、非法或未通过校验后执行确定性规则回退的次数。",
        "unit": "次",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "cloud_fleet_missing_vehicle_decision_count",
        "group": "cloud_coordination",
        "name_zh": "车队决策缺失数",
        "definition_zh": "已纳入同一 fleet epoch 请求但响应中没有对应动作的 AV 数量。",
        "unit": "车次",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "cloud_fleet_event_trigger_epoch_count",
        "group": "cloud_invocation_budget",
        "name_zh": "事件触发决策轮数",
        "definition_zh": "由新车注册、任务边界或可重规划阻塞事件触发的云端车队决策轮数。",
        "unit": "轮",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "cloud_fleet_watchdog_epoch_count",
        "group": "cloud_invocation_budget",
        "name_zh": "Watchdog 决策轮数",
        "definition_zh": "因 30 s 仿真时间状态新鲜度上限触发的云端审计轮数。",
        "unit": "轮",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "cloud_fleet_actionable_epoch_count",
        "group": "cloud_invocation_budget",
        "name_zh": "有效决策轮数",
        "definition_zh": "至少向一辆 AV 下发可执行高层动作的 fleet epoch 数。",
        "unit": "轮",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "automated_vehicle_count",
        "group": "mixed_traffic_scale",
        "name_zh": "自动驾驶车辆数",
        "definition_zh": "剧集内实际生成并记录轨迹的 AV 数量。",
        "unit": "辆",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "human_like_vehicle_count",
        "group": "mixed_traffic_scale",
        "name_zh": "人工行为车辆数",
        "definition_zh": "由 replay、rule-random 或 mixed 人工行为模型生成的车辆数量。",
        "unit": "辆",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "hidden_intent_vehicle_count",
        "group": "mixed_traffic_scale",
        "name_zh": "隐藏细粒度意图车辆数",
        "definition_zh": "操作类别可见但目标泊位、意向路径和未来轨迹对云端不可见的人工车辆数。",
        "unit": "辆",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "decision_barrier_ack_coverage_rate",
        "group": "synchronous_decision_integrity",
        "name_zh": "同步决策 ACK 覆盖率",
        "definition_zh": "冻结时刻被寻址 AV 中，在同一仿真时刻确认动作应用的比例。",
        "unit": "比例",
        "direction": "higher_is_better",
        "primary": False,
    },
    {
        "metric": "decision_barrier_failure_count",
        "group": "synchronous_decision_integrity",
        "name_zh": "同步决策栅栏失败数",
        "definition_zh": "暂停确认、动作应用或 ACK 覆盖未满足而禁止恢复仿真的 epoch 数。",
        "unit": "轮",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "decision_barrier_sim_time_mismatch_count",
        "group": "synchronous_decision_integrity",
        "name_zh": "ACK 仿真时刻不一致数",
        "definition_zh": "车辆 ACK 仿真时刻与冻结决策时刻不一致的记录数。",
        "unit": "次",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "simulation_step_seconds",
        "group": "data_integrity",
        "name_zh": "仿真积分步长",
        "definition_zh": "车辆动力学、交通事件和指标时间轴采用的固定仿真时间增量。",
        "unit": "s",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "simulation_speedup",
        "group": "data_integrity",
        "name_zh": "仿真墙钟加速倍率",
        "definition_zh": "固定仿真步长相对于 ROS 墙钟调度周期的倍率；只影响实验周转时间。",
        "unit": "倍",
        "direction": "descriptive",
        "primary": False,
    },
    {
        "metric": "trace_integrity_ok",
        "group": "data_integrity",
        "name_zh": "轨迹完整性通过率",
        "definition_zh": "车辆身份、仿真时间单调性和运动学连续性检查全部通过的比例。",
        "unit": "比例",
        "direction": "higher_is_better",
        "primary": False,
    },
    {
        "metric": "trace_identity_conflict_count",
        "group": "data_integrity",
        "name_zh": "车辆身份冲突数",
        "definition_zh": "动态、replay 或 AV 复用车辆 ID 或 ROS 命名空间的冲突数。",
        "unit": "次",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "trace_time_regression_count",
        "group": "data_integrity",
        "name_zh": "仿真时间回退数",
        "definition_zh": "同一轨迹文件中仿真时间戳逆序的记录数。",
        "unit": "次",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "trace_sim_step_gap_count",
        "group": "data_integrity",
        "name_zh": "仿真步漏步数",
        "definition_zh": "加速执行期间，相邻轨迹仿真时刻差超过固定积分步长所对应的缺失步数。",
        "unit": "步",
        "direction": "lower_is_better",
        "primary": False,
    },
    {
        "metric": "trace_kinematic_jump_count",
        "group": "data_integrity",
        "name_zh": "运动学跳变数",
        "definition_zh": "相邻轨迹点位移超过速度与仿真步长允许范围的异常数。",
        "unit": "次",
        "direction": "lower_is_better",
        "primary": False,
    },
]

METRIC_BY_NAME = {item["metric"]: item for item in METRIC_CATALOG}
METRIC_GROUPS: Dict[str, List[str]] = {}
for _item in METRIC_CATALOG:
    METRIC_GROUPS.setdefault(_item["group"], []).append(_item["metric"])
PRIMARY_METRICS = [item["metric"] for item in METRIC_CATALOG if item["primary"]]


def _load_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _load_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    with path.open() as f:
        return json.load(f)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value in (None, ""):
            return default
        return float(value)
    except Exception:
        return default


def _optional_float(value: Any) -> Optional[float]:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except Exception:
        return None


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]], fields: Optional[Sequence[str]] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(fields))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def metric_definition_rows() -> List[Dict[str, Any]]:
    return [
        {
            "metric": item["metric"],
            "指标中文名": item["name_zh"],
            "指标组": GROUP_NAMES_ZH[item["group"]],
            "指标组代码": item["group"],
            "定义": item["definition_zh"],
            "单位": item["unit"],
            "优劣方向": item["direction"],
            "是否主要终点": bool(item["primary"]),
            "时间基准": "仿真时间",
            "Qwen墙钟响应时间是否参与": False,
        }
        for item in METRIC_CATALOG
    ]


def build_metric_group_summary(summary_rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    for row in summary_rows:
        for item in METRIC_CATALOG:
            metric = item["metric"]
            mean_key = metric + "_mean"
            if row.get(mean_key) in (None, ""):
                continue
            output.append({
                "agent_type": row.get("agent_type", ""),
                "episodes": row.get("episodes", ""),
                "metric": metric,
                "metric_name_zh": item["name_zh"],
                "metric_group": item["group"],
                "metric_group_zh": GROUP_NAMES_ZH[item["group"]],
                "unit": item["unit"],
                "direction": item["direction"],
                "primary_endpoint": bool(item["primary"]),
                "mean": row.get(mean_key, ""),
                "std": row.get(metric + "_std", ""),
                "ci95_half_width": row.get(metric + "_ci95", ""),
            })
    return output


def _comparison_effect(delta_baseline_minus_reference: float, direction: str) -> Optional[float]:
    if direction == "higher_is_better":
        return -delta_baseline_minus_reference
    if direction == "lower_is_better":
        return delta_baseline_minus_reference
    return None


def _comparison_conclusion_zh(effect: Optional[float], paired_count: int, p_holm: Optional[float]) -> str:
    if effect is None:
        return "描述性规模或调用预算指标，不作方法优劣判定"
    if abs(effect) <= 1e-12:
        return "目标方法与基线的配对均值无差异"
    direction = "优于" if effect > 0.0 else "劣于"
    if paired_count < 3:
        return "校准样本中目标方法方向上%s基线；样本不足，不作显著性结论" % direction
    if p_holm is None:
        return "目标方法方向上%s基线；缺少可用的 Holm 校正检验" % direction
    if p_holm < 0.05:
        return "目标方法%s基线，配对置换检验经 Holm 校正后显著" % direction
    return "目标方法方向上%s基线，但 Holm 校正后未达显著" % direction


def build_reference_improvements(
    delta_summary: Sequence[Dict[str, str]],
    statistical_tests: Sequence[Dict[str, str]],
    summary_rows: Sequence[Dict[str, str]],
    reference_agent: str,
) -> List[Dict[str, Any]]:
    means = {
        (str(row.get("agent_type", "")), item["metric"]): _optional_float(row.get(item["metric"] + "_mean"))
        for row in summary_rows
        for item in METRIC_CATALOG
    }
    stats = {
        (str(row.get("agent_type", "")), str(row.get("metric", ""))): row
        for row in statistical_tests
    }
    output: List[Dict[str, Any]] = []
    for row in delta_summary:
        baseline = str(row.get("agent_type", ""))
        paired_count = int(_safe_float(row.get("paired_count")))
        for item in METRIC_CATALOG:
            metric = item["metric"]
            key = metric + "_delta_vs_ref_mean"
            if row.get(key) in (None, ""):
                continue
            delta = _safe_float(row.get(key))
            effect = _comparison_effect(delta, item["direction"])
            baseline_mean = means.get((baseline, metric))
            reference_mean = means.get((reference_agent, metric))
            percent = None
            if effect is not None and baseline_mean is not None and abs(baseline_mean) > 1e-12:
                percent = 100.0 * effect / abs(baseline_mean)
            stat = stats.get((baseline, metric), {})
            p_raw = _optional_float(stat.get("paired_permutation_p"))
            p_holm = _optional_float(stat.get("paired_permutation_p_holm"))
            output.append({
                "reference_agent": reference_agent,
                "baseline_agent": baseline,
                "paired_count": paired_count,
                "metric": metric,
                "metric_name_zh": item["name_zh"],
                "metric_group": item["group"],
                "metric_group_zh": GROUP_NAMES_ZH[item["group"]],
                "unit": item["unit"],
                "direction": item["direction"],
                "primary_endpoint": bool(item["primary"]),
                "baseline_mean": "" if baseline_mean is None else baseline_mean,
                "reference_mean": "" if reference_mean is None else reference_mean,
                "delta_baseline_minus_reference": delta,
                "delta_ci95_half_width": row.get(metric + "_delta_vs_ref_ci95", ""),
                "reference_improvement": "" if effect is None else effect,
                "reference_improvement_percent": "" if percent is None else percent,
                "reference_directionally_better": "" if effect is None else effect > 1e-12,
                "paired_permutation_p": "" if p_raw is None else p_raw,
                "paired_permutation_p_holm": "" if p_holm is None else p_holm,
                "holm_significant_0_05": bool(p_holm is not None and p_holm < 0.05),
                "effect_dz_baseline_minus_reference": stat.get("effect_dz", ""),
                "conclusion_zh": _comparison_conclusion_zh(effect, paired_count, p_holm),
                "evidence_level": "calibration" if paired_count < 3 else ("pilot" if paired_count < 10 else "confirmatory_eligible"),
                "time_basis": "simulator_time",
                "qwen_wall_clock_latency_in_performance_metrics": False,
            })
    return output


def build_stratified_findings(
    strata_rows: Sequence[Dict[str, str]],
    reference_agent: str,
) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, str], List[Dict[str, str]]] = {}
    for row in strata_rows:
        key = (str(row.get("factor", "")), str(row.get("level", "")))
        grouped.setdefault(key, []).append(row)
    output: List[Dict[str, Any]] = []
    for (factor, level), rows in sorted(grouped.items()):
        for metric in PRIMARY_METRICS:
            item = METRIC_BY_NAME[metric]
            available = [row for row in rows if row.get(metric + "_mean") not in (None, "")]
            if not available:
                continue
            reverse = item["direction"] == "higher_is_better"
            best = sorted(available, key=lambda row: _safe_float(row.get(metric + "_mean")), reverse=reverse)[0]
            reference = next((row for row in available if row.get("agent_type") == reference_agent), None)
            reference_value = _optional_float(reference.get(metric + "_mean")) if reference else None
            best_value = _safe_float(best.get(metric + "_mean"))
            raw_gap = None if reference_value is None else reference_value - best_value
            output.append({
                "factor": factor,
                "level": level,
                "metric": metric,
                "metric_name_zh": item["name_zh"],
                "unit": item["unit"],
                "direction": item["direction"],
                "best_agent": best.get("agent_type", ""),
                "best_mean": best_value,
                "reference_agent": reference_agent,
                "reference_mean": "" if reference_value is None else reference_value,
                "reference_is_best": bool(reference and reference.get("agent_type") == best.get("agent_type")),
                "reference_minus_best_raw": "" if raw_gap is None else raw_gap,
                "interpretation": "分层描述性比较；未进行该层内多重校正检验",
            })
    return output


def render_findings_md(
    improvements: Sequence[Dict[str, Any]],
    strata: Sequence[Dict[str, Any]],
    reproducibility: Dict[str, Any],
    reference_agent: str,
) -> str:
    seeds = [str(item) for item in reproducibility.get("seeds", [])]
    lines = [
        "# TR-C 实验组结果摘要",
        "",
        "> 本文件由冻结的 CSV/JSON 生成，用于人工分析与论文回填；不会自动修改 TeX。",
        "> 所有性能指标均使用仿真时间。Qwen 墙钟响应时间不进入等待、历时、效率、安全或方法排序。",
        "",
        "## 证据覆盖",
        "",
        "- 结果行 / 方法 / 场景：`%s / %s / %s`" % (
            reproducibility.get("row_count", 0),
            reproducibility.get("agent_count", 0),
            reproducibility.get("scenario_count", 0),
        ),
        "- 随机种子：`%s`" % (", ".join(seeds) if seeds else "无"),
        "- 人工车辆模式：`%s`" % ", ".join(reproducibility.get("background_modes", [])),
        "- 需求密度：`%s`" % ", ".join(reproducibility.get("density_labels", [])),
        "- 参考方法：`%s`" % reference_agent,
        "",
        "## 主要指标配对比较",
        "",
        "| 基线 | 指标 | 基线均值 | 参考均值 | 参考改善量 | Holm p | 结论 |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    primary_rows = [row for row in improvements if row.get("primary_endpoint")]
    for row in primary_rows:
        lines.append(
            "| {baseline_agent} | {metric_name_zh} | {baseline_mean} | {reference_mean} | {reference_improvement} | {paired_permutation_p_holm} | {conclusion_zh} |".format(**row)
        )
    if not primary_rows:
        lines.append("| - | - | - | - | - | - | 当前没有可配对的主要指标 |")
    lines.extend([
        "",
        "## 分层稳健性",
        "",
        "- 已生成 `%d` 条分层-指标描述性记录。分层结论须结合配对统计，不从单层最优值直接作因果推断。" % len(strata),
        "",
        "## 指标解释原则",
        "",
        "- `reference_improvement > 0` 表示参考方法按该指标的预设优劣方向优于基线；不同量纲之间不相加。",
        "- 服务率与吞吐量越高越好；任务历时、等待、近失、碰撞代理和轨迹冲突率越低越好。",
        "- 车辆数、事件触发轮数和 watchdog 轮数属于暴露量或调用预算，只描述规模，不作优劣判定。",
        "- 配对数少于 10 的结果只能作为校准或预实验，不进入正式方法优越性结论。",
        "- 关键窗口事件是机制诊断关联，不构成因果证明；因果表述需要预注册消融与跨种子检验。",
        "",
        "## 机器可读输出",
        "",
        "- `trc_metric_definitions_zh.csv`",
        "- `trc_metric_group_summary.csv`",
        "- `trc_reference_improvements.csv`",
        "- `trc_group_result_summary_zh.csv`",
        "- `trc_stratified_findings.csv`",
        "- `trc_analysis_manifest.json`",
    ])
    return "\n".join(lines) + "\n"


def analyze(report_dir: Path, out_dir: Path, reference_agent: str) -> Dict[str, Any]:
    report_dir = report_dir.resolve()
    out_dir = out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = _load_csv(report_dir / "paper_summary.csv")
    delta_summary = _load_csv(report_dir / "paper_paired_delta_summary.csv")
    statistical_tests = _load_csv(report_dir / "paper_statistical_tests.csv")
    strata_rows = _load_csv(report_dir / "paper_stratified_summary.csv")
    reproducibility = _load_json(report_dir / "paper_reproducibility.json", {}) or {}

    definitions = metric_definition_rows()
    metric_groups = build_metric_group_summary(summary_rows)
    improvements = build_reference_improvements(
        delta_summary,
        statistical_tests,
        summary_rows,
        reference_agent,
    )
    group_results = [row for row in improvements if row.get("primary_endpoint")]
    strata = build_stratified_findings(strata_rows, reference_agent)

    _write_csv(out_dir / "trc_metric_definitions_zh.csv", definitions)
    _write_csv(out_dir / "trc_metric_group_summary.csv", metric_groups)
    _write_csv(out_dir / "trc_reference_improvements.csv", improvements)
    _write_csv(out_dir / "trc_group_result_summary_zh.csv", group_results)
    _write_csv(out_dir / "trc_stratified_findings.csv", strata)
    findings = render_findings_md(improvements, strata, reproducibility, reference_agent)
    (out_dir / "trc_key_findings.md").write_text(findings)
    (out_dir / "trc_key_findings_zh.md").write_text(findings)
    manifest = {
        "type": "parksim_vla_trc_csv_first_analysis",
        "report_dir": str(report_dir),
        "out_dir": str(out_dir),
        "reference_agent": reference_agent,
        "time_basis": "simulator_time",
        "qwen_wall_clock_latency_in_performance_metrics": False,
        "aggregation_policy": "No sums or weighted scores across heterogeneous metric units.",
        "comparison_delta": "baseline minus reference; reference_improvement is sign-adjusted by the predeclared metric direction",
        "formal_evidence_minimum_paired_seeds": 10,
        "inputs": [
            "paper_summary.csv",
            "paper_paired_delta_summary.csv",
            "paper_statistical_tests.csv",
            "paper_stratified_summary.csv",
            "paper_reproducibility.json",
        ],
        "outputs": [
            "trc_metric_definitions_zh.csv",
            "trc_metric_group_summary.csv",
            "trc_reference_improvements.csv",
            "trc_group_result_summary_zh.csv",
            "trc_stratified_findings.csv",
            "trc_key_findings.md",
            "trc_key_findings_zh.md",
            "trc_analysis_manifest.json",
        ],
        "row_counts": {
            "metric_definitions": len(definitions),
            "metric_group_summary": len(metric_groups),
            "reference_improvements": len(improvements),
            "group_result_summary": len(group_results),
            "stratified_findings": len(strata),
        },
    }
    (out_dir / "trc_analysis_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Build TR-C-oriented CSV-first analysis from ParkSim-VLA paper report outputs.")
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--reference-agent", default="qwen_vla")
    args = parser.parse_args()

    report_dir = args.report_dir.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else report_dir
    manifest = analyze(report_dir, out_dir, args.reference_agent)
    counts = manifest["row_counts"]
    print(
        "trc_analysis=%s metric_rows=%d comparisons=%d primary_group_rows=%d strata=%d"
        % (
            out_dir,
            counts["metric_group_summary"],
            counts["reference_improvements"],
            counts["group_result_summary"],
            counts["stratified_findings"],
        )
    )


if __name__ == "__main__":
    main()
