import math
from typing import Any, Dict, Optional, Tuple


def _same_run(payload: Dict[str, Any], run_id: str) -> bool:
    packet_run_id = str(payload.get("run_id", ""))
    return not packet_run_id or not run_id or packet_run_id == str(run_id)


def _expected_vehicle_ids(pending_decision: Dict[str, Any]) -> set:
    vehicle_ids = set()
    for item in pending_decision.get("fleet_decisions", []) or []:
        if not isinstance(item, dict):
            continue
        try:
            vehicle_ids.add(int(item.get("vehicle_id")))
        except (TypeError, ValueError):
            continue
    return vehicle_ids


def validate_pause_ack(
    payload: Any,
    run_id: str,
    requested_sim_time: float,
    epoch_active: bool = False,
) -> Optional[Dict[str, Any]]:
    if epoch_active or not isinstance(payload, dict) or not _same_run(payload, run_id):
        return None
    try:
        ack_time = float(payload.get("sim_time"))
    except (TypeError, ValueError):
        return None
    if not bool(payload.get("paused")) or not math.isfinite(ack_time):
        return None
    if ack_time + 1e-9 < float(requested_sim_time):
        return None
    return dict(payload)


def validate_decision_ack(
    payload: Any,
    run_id: str,
    pending_decision: Optional[Dict[str, Any]],
) -> Optional[Tuple[int, Dict[str, Any]]]:
    if not isinstance(payload, dict) or pending_decision is None or not _same_run(payload, run_id):
        return None
    try:
        epoch_id = int(payload.get("epoch_id"))
        vehicle_id = int(payload.get("vehicle_id"))
    except (TypeError, ValueError):
        return None
    if epoch_id != int(pending_decision.get("epoch_id", -1)):
        return None
    expected = _expected_vehicle_ids(pending_decision)
    if vehicle_id not in expected:
        return None
    checked = dict(payload)
    decision_sim_time = float(pending_decision.get("sim_time", 0.0))
    try:
        ack_decision_time = float(checked.get("decision_sim_time"))
        vehicle_sim_time = float(checked.get("vehicle_sim_time"))
    except (TypeError, ValueError):
        ack_decision_time = float("nan")
        vehicle_sim_time = float("nan")
    if (
        not math.isfinite(ack_decision_time)
        or not math.isfinite(vehicle_sim_time)
        or abs(ack_decision_time - decision_sim_time) > 1e-6
        or abs(vehicle_sim_time - decision_sim_time) > 1e-6
    ):
        checked["applied"] = False
        checked["error"] = "decision acknowledgement violated frozen simulation-time barrier"
    return vehicle_id, checked


def decision_barrier_status(
    pending_decision: Optional[Dict[str, Any]],
    acknowledgements: Dict[int, Dict[str, Any]],
    now: float,
    published_wall_time: Optional[float],
    timeout: float,
) -> str:
    if pending_decision is None:
        return "idle"
    expected = _expected_vehicle_ids(pending_decision)
    if any(not bool(ack.get("applied")) for ack in acknowledgements.values()):
        return "decision_apply_failed"
    if expected.issubset(acknowledgements):
        return "applied"
    started = float(now if published_wall_time is None else published_wall_time)
    if float(now) - started >= max(0.1, float(timeout)):
        return "decision_ack_timeout"
    return "pending"


def barrier_audit_fields(
    pending_decision: Dict[str, Any],
    acknowledgements: Dict[int, Dict[str, Any]],
    now: float,
    published_wall_time: Optional[float],
    status: str,
) -> Dict[str, Any]:
    expected = sorted(_expected_vehicle_ids(pending_decision))
    received = sorted(acknowledgements)
    failed = sorted(
        vehicle_id
        for vehicle_id, ack in acknowledgements.items()
        if not bool(ack.get("applied"))
    )
    started = float(now if published_wall_time is None else published_wall_time)
    resolved_status = "no_action_required" if status == "applied" and not expected else str(status)
    return {
        "decision_ack_expected_vehicle_ids": expected,
        "decision_ack_received_vehicle_ids": received,
        "decision_acknowledgements": [acknowledgements[item] for item in received],
        "decision_ack_failed_vehicle_ids": failed,
        "decision_ack_coverage_rate": len(received) / len(expected) if expected else 1.0,
        "decision_apply_wait_seconds": float(now) - started,
        "decision_barrier_status": resolved_status,
        "simulation_time_policy": "decision_then_advance",
        "wall_clock_latency_in_performance_metrics": False,
    }
