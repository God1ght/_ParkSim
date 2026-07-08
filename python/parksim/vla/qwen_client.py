import base64
import json
import mimetypes
import re
import time
from pathlib import Path
from typing import Any, Dict, List
from urllib import request, error

from parksim.vla.schema import VLACandidateAction, VLADecision, VLAContext


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
        prompt = (
            "You are the high-level VLA policy for a parking-lot vehicle. "
            "Choose exactly one action_id from valid_actions. Return strict JSON with "
            "action_id, reason, and confidence. Do not output steering, acceleration, or free-form routes.\n\n"
            + json.dumps(context.to_dict(), ensure_ascii=False)
        )
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
            "messages": [{"role": "user", "content": content}],
            "temperature": 0.1,
            "max_tokens": 256,
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
        )

    def _fallback(self, actions: List[VLACandidateAction], reason: str) -> VLADecision:
        action_id = actions[0].action_id if actions else ""
        for action in actions:
            if action.action_type == "SELECT_SPOT_AND_CRUISE":
                action_id = action.action_id
                break
        return VLADecision(action_id=action_id, reason=reason, confidence=0.0, used_fallback=True)
