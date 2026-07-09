import base64
import json
import mimetypes
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib import request, error

from parksim.vla.decision_protocol import build_decision_packet, build_qwen_prompt
from parksim.vla.schema import VLAActionType, VLACandidateAction, VLADecision, VLAContext


class QwenPolicyClient:
    def __init__(self, endpoint: str = "", model: str = "Qwen2.5-VL-7B-Instruct", timeout: float = 15.0):
        self.endpoint = endpoint.rstrip("/") if endpoint else ""
        self.model = model
        self.timeout = float(timeout)

    def decide(self, context: VLAContext) -> VLADecision:
        if not self.endpoint:
            return self._fallback(context.valid_actions, reason="qwen endpoint is not configured")
        payload = self._build_payload(context)
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(self.endpoint, data=data, headers={"Content-Type": "application/json"}, method="POST")
        started = time.time()
        try:
            with request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8", errors="replace")
        except (error.URLError, TimeoutError, OSError) as exc:
            return self._fallback(context.valid_actions, reason="qwen request failed: %s" % exc)
        decision = self._parse_decision(body)
        decision.raw_response = body
        if not decision.action_id:
            fallback = self._fallback(context.valid_actions, reason="qwen response did not contain action_id")
            fallback.raw_response = body
            return fallback
        if decision.confidence <= 0.0:
            decision.confidence = max(0.1, 1.0 - min(time.time() - started, 10.0) / 20.0)
        return decision

    def _build_payload(self, context: VLAContext) -> Dict[str, Any]:
        packet = build_decision_packet(context)
        prompt = build_qwen_prompt(packet)
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
                {"role": "system", "content": "Return strict JSON only for the ParkSim VLA decision protocol."},
                {"role": "user", "content": content},
            ],
            "temperature": 0.0,
            "max_tokens": 256,
            "context": packet,
            "metadata": {
                "protocol_version": packet.get("protocol_version"),
                "prompt_version": packet.get("prompt_version"),
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

    def _parse_decision(self, text: str) -> VLADecision:
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
                return self._parse_decision(content)
            except Exception:
                pass
        return VLADecision(
            action_id=str(parsed.get("action_id", "")),
            reason=str(parsed.get("reason", "")),
            confidence=float(parsed.get("confidence", 0.0) or 0.0),
            target_spot_index=_parse_optional_int(parsed.get("target_spot_index")),
            reason_code=str(parsed.get("reason_code", "")),
        )

    def _fallback(self, actions: List[VLACandidateAction], reason: str) -> VLADecision:
        selected = _select_lowest_cost_fallback(actions)
        if selected is None:
            return VLADecision(action_id="", reason=reason, confidence=0.0, used_fallback=True, reason_code="FALLBACK_OR_RECOVERY")
        return VLADecision(
            action_id=selected.action_id,
            target_spot_index=selected.target_spot_index,
            reason=reason + "; selected lowest-cost valid bundle",
            confidence=0.0,
            used_fallback=True,
            reason_code="FALLBACK_OR_RECOVERY",
        )


def _select_lowest_cost_fallback(actions: List[VLACandidateAction]) -> Optional[VLACandidateAction]:
    if not actions:
        return None
    progress_types = {VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE, VLAActionType.CRUISE_TO_EXIT}
    cost_ranked = [action for action in actions if _finite_action_cost(action) is not None]
    if cost_ranked:
        progress_ranked = [action for action in cost_ranked if action.action_type in progress_types]
        ranked = progress_ranked or cost_ranked
        return min(ranked, key=lambda action: (_finite_action_cost(action), _feature_float(action, "conflict_risk"), _feature_float(action, "expected_wait_s")))
    for action in actions:
        if action.action_type == VLAActionType.SELECT_SPOT_AND_CRUISE:
            return action
    return actions[0]


def _finite_action_cost(action: VLACandidateAction) -> Optional[float]:
    try:
        value = float((action.features or {}).get("bundle_cost"))
    except Exception:
        return None
    if value != value or value in (float("inf"), float("-inf")):
        return None
    return value


def _feature_float(action: VLACandidateAction, key: str) -> float:
    try:
        return float((action.features or {}).get(key, 1e9))
    except Exception:
        return 1e9


def _parse_optional_int(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except Exception:
        return None
