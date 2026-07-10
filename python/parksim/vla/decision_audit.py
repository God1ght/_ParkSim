import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from parksim.vla.schema import VLA_DECISION_PROTOCOL_VERSION, VLA_PROMPT_VERSION, VLA_REASON_CODES


FAILURE_KEYS = (
    "missing_protocol_version",
    "unexpected_protocol_version",
    "missing_prompt_version",
    "missing_decision_packet",
    "missing_valid_actions",
    "missing_action_id",
    "action_not_in_valid_actions",
    "target_mismatch",
    "missing_reason_code",
    "unknown_reason_code",
    "shield_rejection",
    "unsafe_applied_spot",
)


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open() as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except Exception as exc:
                rows.append({"_parse_error": str(exc), "_line_no": line_no})
                continue
            if isinstance(row, dict):
                row["_line_no"] = line_no
                rows.append(row)
    return rows


def discover_logs(inputs: Iterable[Path]) -> List[Path]:
    logs: List[Path] = []
    for item in inputs:
        item = item.expanduser()
        if item.is_file():
            logs.append(item)
        elif item.is_dir():
            logs.extend(sorted(item.glob("**/qwen_vla_decisions.jsonl")))
    return sorted(dict.fromkeys(path.resolve() for path in logs))


def audit_record(record: Dict[str, Any], log_path: Path) -> Tuple[List[str], Dict[str, Any]]:
    failures: List[str] = []
    if record.get("_parse_error"):
        failures.append("malformed_json_line")
        return failures, _failure_record(record, log_path, failures)

    protocol_version = record.get("protocol_version") or ((record.get("decision_packet") or {}).get("protocol_version"))
    prompt_version = record.get("prompt_version") or ((record.get("decision_packet") or {}).get("prompt_version"))
    if not protocol_version:
        failures.append("missing_protocol_version")
    elif protocol_version != VLA_DECISION_PROTOCOL_VERSION:
        failures.append("unexpected_protocol_version")
    if not prompt_version:
        failures.append("missing_prompt_version")

    packet = record.get("decision_packet")
    if not isinstance(packet, dict):
        failures.append("missing_decision_packet")
        packet = {}

    valid_actions = _valid_actions(record, packet)
    valid_action_ids = [str(action.get("action_id")) for action in valid_actions if action.get("action_id")]
    if not valid_actions:
        failures.append("missing_valid_actions")

    decision = record.get("decision") or {}
    action_id = str(decision.get("action_id", ""))
    if not action_id:
        failures.append("missing_action_id")
    elif valid_action_ids and action_id not in valid_action_ids:
        failures.append("action_not_in_valid_actions")

    selected = _action_by_id(valid_actions, action_id)
    selected_target = _optional_int((selected or {}).get("target_spot_index"))
    decision_target = _optional_int(decision.get("target_spot_index"))
    if decision_target is not None and selected_target != decision_target:
        failures.append("target_mismatch")

    reason_code = str(decision.get("reason_code", ""))
    if not reason_code:
        failures.append("missing_reason_code")
    elif reason_code not in VLA_REASON_CODES:
        failures.append("unknown_reason_code")

    shield_reason = str(record.get("shield_reason") or "")
    if shield_reason and shield_reason != "ok":
        if not shield_reason.startswith("executor_recovery:"):
            failures.append("shield_rejection")

    applied = record.get("applied_action") or {}
    features = applied.get("features") or {}
    if applied.get("target_spot_index") is not None:
        occupancy_status = features.get("occupancy_status")
        selectable = features.get("selectable", True)
        if selectable is False or occupancy_status in ("occupied", "unknown", "blocked"):
            failures.append("unsafe_applied_spot")

    return failures, _failure_record(record, log_path, failures)


def audit_logs(logs: List[Path]) -> Dict[str, Any]:
    counts: Dict[str, int] = {key: 0 for key in FAILURE_KEYS}
    counts["malformed_json_line"] = 0
    failure_rows: List[Dict[str, Any]] = []
    total_records = 0
    decision_records = 0
    protocol_versions: Dict[str, int] = {}
    prompt_versions: Dict[str, int] = {}
    executor_recovery_count = 0

    for log_path in logs:
        for record in load_jsonl(log_path):
            total_records += 1
            if record.get("decision"):
                decision_records += 1
            protocol = record.get("protocol_version") or ((record.get("decision_packet") or {}).get("protocol_version")) or ""
            prompt = record.get("prompt_version") or ((record.get("decision_packet") or {}).get("prompt_version")) or ""
            if protocol:
                protocol_versions[protocol] = protocol_versions.get(protocol, 0) + 1
            if prompt:
                prompt_versions[prompt] = prompt_versions.get(prompt, 0) + 1
            failures, failure_row = audit_record(record, log_path)
            if str(record.get("shield_reason") or "").startswith("executor_recovery:"):
                executor_recovery_count += 1
            for failure in failures:
                counts[failure] = counts.get(failure, 0) + 1
            if failures:
                failure_rows.append(failure_row)

    return {
        "ok": not failure_rows,
        "log_count": len(logs),
        "decision_record_count": total_records,
        "records_with_decision": decision_records,
        "failure_count": len(failure_rows),
        "failure_type_counts": {key: value for key, value in sorted(counts.items()) if value},
        "protocol_versions": protocol_versions,
        "prompt_versions": prompt_versions,
        "executor_recovery_count": int(executor_recovery_count),
        "logs": [str(path) for path in logs],
        "failures": failure_rows,
    }


def write_outputs(summary: Dict[str, Any], out_dir: Optional[Path]) -> None:
    if out_dir is None:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    failures = summary.get("failures", [])
    compact = dict(summary)
    compact["failures"] = []
    (out_dir / "decision_audit.json").write_text(json.dumps(compact, indent=2) + "\n")
    with (out_dir / "decision_audit_failures.jsonl").open("w") as f:
        for row in failures:
            f.write(json.dumps(row) + "\n")


def _valid_actions(record: Dict[str, Any], packet: Dict[str, Any]) -> List[Dict[str, Any]]:
    actions = packet.get("valid_actions")
    if isinstance(actions, list) and actions:
        return [action for action in actions if isinstance(action, dict)]
    context = record.get("context") or {}
    actions = context.get("valid_actions")
    if isinstance(actions, list):
        return [action for action in actions if isinstance(action, dict)]
    return []


def _action_by_id(actions: List[Dict[str, Any]], action_id: str) -> Dict[str, Any]:
    for action in actions:
        if str(action.get("action_id", "")) == action_id:
            return action
    return {}


def _optional_int(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except Exception:
        return None


def _failure_record(record: Dict[str, Any], log_path: Path, failures: List[str]) -> Dict[str, Any]:
    decision = record.get("decision") or {}
    applied = record.get("applied_action") or {}
    return {
        "log_path": str(log_path),
        "line_no": record.get("_line_no"),
        "time": record.get("time"),
        "failures": failures,
        "action_id": decision.get("action_id"),
        "decision_target_spot_index": decision.get("target_spot_index"),
        "reason_code": decision.get("reason_code"),
        "shield_reason": record.get("shield_reason"),
        "applied_action_id": applied.get("action_id"),
        "applied_target_spot_index": applied.get("target_spot_index"),
        "protocol_version": record.get("protocol_version") or ((record.get("decision_packet") or {}).get("protocol_version")),
        "prompt_version": record.get("prompt_version") or ((record.get("decision_packet") or {}).get("prompt_version")),
        "valid_action_ids": record.get("valid_action_ids") or ((record.get("decision_packet") or {}).get("valid_action_ids")) or [],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Audit ParkSim Qwen-VLA decision logs for protocol compliance.")
    parser.add_argument("inputs", nargs="+", type=Path, help="Benchmark directories or qwen_vla_decisions.jsonl files.")
    parser.add_argument("--out-dir", type=Path, default=None, help="Optional directory for decision_audit.json and decision_audit_failures.jsonl.")
    parser.add_argument("--strict", action="store_true", help="Exit non-zero when any protocol failure is found.")
    args = parser.parse_args()

    logs = discover_logs(args.inputs)
    summary = audit_logs(logs)
    write_outputs(summary, args.out_dir)
    printable = dict(summary)
    printable["failures"] = summary.get("failures", [])[:10]
    print(json.dumps(printable, indent=2))
    if args.strict and not summary.get("ok"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
