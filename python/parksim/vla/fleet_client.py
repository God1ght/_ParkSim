import base64
import json
import mimetypes
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib import error, request

from parksim.vla.fleet_protocol import build_fleet_decision_packet, build_fleet_qwen_prompt
from parksim.vla.fleet_schema import VLAFleetContext, VLAFleetDecision, VLAFleetResponse
from parksim.vla.qwen_client import _select_lowest_cost_fallback
from parksim.vla.schema import VLACandidateAction


class QwenFleetPolicyClient:
    def __init__(self, endpoint: str = "", model: str = "Qwen2.5-VL-7B-Instruct", timeout: float = 15.0):
        self.endpoint = endpoint.rstrip("/") if endpoint else ""
        self.model = model
        self.timeout = float(timeout)

    def decide_fleet(self, context: VLAFleetContext) -> VLAFleetResponse:
        if not self.endpoint:
            return self._fallback(context, reason="qwen endpoint is not configured")
        payload = self._build_payload(context)
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(self.endpoint, data=data, headers={"Content-Type": "application/json"}, method="POST")
        started = time.time()
        try:
            with request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8", errors="replace")
        except (error.URLError, TimeoutError, OSError) as exc:
            return self._fallback(context, reason="qwen fleet request failed: %s" % exc)
        response = self._parse_response(body, context)
        for decision in response.decisions:
            decision.raw_response = body
            if decision.confidence <= 0.0:
                decision.confidence = max(0.1, 1.0 - min(time.time() - started, 10.0) / 20.0)
        response.raw_response = body
        return response

    def _build_payload(self, context: VLAFleetContext) -> Dict[str, Any]:
        packet = build_fleet_decision_packet(context)
        prompt = build_fleet_qwen_prompt(packet)
        content: Any = prompt
        if context.bev_image_path:
            image_url = self._image_data_url(context.bev_image_path)
            if image_url:
                content = [
                    {"type": "image_url", "image_url": {"url": image_url}},
                    {"type": "text", "text": prompt},
                ]
        return {
            "model": self.model,
            "messages": [
                {"role": "system", "content": "Return strict JSON only for the ParkSim cloud fleet VLA decision protocol."},
                {"role": "user", "content": content},
            ],
            "temperature": 0.0,
            "max_tokens": 768,
            "context": packet,
            "metadata": {
                "protocol_version": packet.get("protocol_version"),
                "prompt_version": packet.get("prompt_version"),
                "decision_scope": "cloud_fleet",
            },
        }

    def _image_data_url(self, image_path: str) -> str:
        path = Path(image_path)
        if not path.exists() or path.suffix == ".npy":
            return ""
        mime = mimetypes.guess_type(str(path))[0] or "image/png"
        with path.open("rb") as f:
            encoded = base64.b64encode(f.read()).decode("ascii")
        return "data:%s;base64,%s" % (mime, encoded)

    def _parse_response(self, text: str, context: VLAFleetContext) -> VLAFleetResponse:
        parsed: Dict[str, Any] = {}
        try:
            parsed = json.loads(text)
        except Exception:
            match = re.search(r"\{.*\}", text, flags=re.S)
            if match:
                try:
                    parsed = json.loads(match.group(0))
                except Exception:
                    parsed = {}
        if "choices" in parsed:
            try:
                content = parsed["choices"][0]["message"]["content"]
                return self._parse_response(content, context)
            except Exception:
                pass
        raw_decisions = parsed.get("fleet_decisions")
        if raw_decisions is None:
            raw_decisions = parsed.get("decisions")
        if raw_decisions is None and parsed.get("action_id") is not None:
            vehicle_id = _first_vehicle_id(context)
            raw_decisions = [dict(parsed, vehicle_id=vehicle_id)]
        if not isinstance(raw_decisions, list):
            return self._fallback(context, reason="qwen response did not contain fleet_decisions")
        decisions: List[VLAFleetDecision] = []
        for item in raw_decisions:
            if not isinstance(item, dict):
                continue
            vehicle_id = _parse_int(item.get("vehicle_id"), default=None)
            if vehicle_id is None:
                continue
            decisions.append(VLAFleetDecision(
                vehicle_id=vehicle_id,
                action_id=str(item.get("action_id", "")),
                target_spot_index=_parse_optional_int(item.get("target_spot_index")),
                priority=_parse_int(item.get("priority"), default=len(decisions)),
                reason_code=str(item.get("reason_code", "")),
                reason=str(item.get("reason", "")),
                confidence=_parse_float(item.get("confidence"), default=0.0),
                used_fallback=bool(item.get("used_fallback", False)),
            ))
        if not decisions:
            return self._fallback(context, reason="qwen response did not contain usable fleet decisions")
        return VLAFleetResponse(decisions=decisions, raw_response=text)

    def _fallback(self, context: VLAFleetContext, reason: str) -> VLAFleetResponse:
        decisions: List[VLAFleetDecision] = []
        reserved_spots = set()
        for vehicle in context.to_dict().get("automated_vehicles", []):
            vehicle_id = int(vehicle.get("vehicle_id", -1))
            actions = _candidate_actions_from_dicts(vehicle.get("valid_actions", []))
            selected = _select_non_conflicting_fallback(actions, reserved_spots)
            if selected is None:
                decisions.append(VLAFleetDecision(vehicle_id=vehicle_id, action_id="", reason=reason, confidence=0.0, used_fallback=True, reason_code="FALLBACK_OR_RECOVERY"))
                continue
            if selected.target_spot_index is not None:
                reserved_spots.add(abs(int(selected.target_spot_index)))
            decisions.append(VLAFleetDecision(
                vehicle_id=vehicle_id,
                action_id=selected.action_id,
                target_spot_index=selected.target_spot_index,
                priority=len(decisions),
                reason=reason + "; fleet fallback selected lowest-cost non-conflicting bundle",
                confidence=0.0,
                used_fallback=True,
                reason_code="FALLBACK_OR_RECOVERY",
            ))
        return VLAFleetResponse(decisions=decisions)


def _candidate_actions_from_dicts(rows: List[Dict[str, Any]]) -> List[VLACandidateAction]:
    actions: List[VLACandidateAction] = []
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        actions.append(VLACandidateAction(
            action_id=str(row.get("action_id", "")),
            action_type=str(row.get("action_type", "")),
            target_spot_index=_parse_optional_int(row.get("target_spot_index")),
            target_coords=row.get("target_coords"),
            duration=_parse_optional_float(row.get("duration")),
            route_id=row.get("route_id"),
            reason=str(row.get("reason", "")),
            features=dict(row.get("features") or {}),
        ))
    return actions


def _select_non_conflicting_fallback(actions: List[VLACandidateAction], reserved_spots: set) -> Optional[VLACandidateAction]:
    filtered = [
        action for action in actions
        if action.target_spot_index is None or abs(int(action.target_spot_index)) not in reserved_spots
    ]
    return _select_lowest_cost_fallback(filtered or actions)


def _first_vehicle_id(context: VLAFleetContext) -> int:
    ids = context.to_dict().get("automated_vehicle_ids", [])
    return int(ids[0]) if ids else -1


def _parse_optional_int(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except Exception:
        return None


def _parse_optional_float(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except Exception:
        return None


def _parse_int(value: Any, default: Optional[int] = 0) -> Optional[int]:
    try:
        return int(value)
    except Exception:
        return default


def _parse_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default
