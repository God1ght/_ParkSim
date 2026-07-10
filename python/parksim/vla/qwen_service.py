import argparse
import base64
import json
import os
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from typing import Any, Dict, List, Sequence, Tuple


DEFAULT_MODEL_ID = "Qwen/Qwen2.5-VL-7B-Instruct"


def resolve_model_id(model_id: str) -> str:
    model_id = (model_id or DEFAULT_MODEL_ID).strip()
    if "/" in model_id:
        return model_id
    if model_id.startswith("Qwen"):
        return "Qwen/%s" % model_id
    return model_id


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def first_json_object(text: str) -> Dict[str, Any]:
    if not text:
        return {}
    decoder = json.JSONDecoder()
    for match in re.finditer(r"\{", text):
        try:
            parsed, _ = decoder.raw_decode(text[match.start() :])
        except Exception:
            continue
        if isinstance(parsed, dict):
            return parsed
    return {}


def _message_text(messages: Sequence[Dict[str, Any]]) -> str:
    chunks: List[str] = []
    for message in messages:
        content = message.get("content", "")
        if isinstance(content, str):
            chunks.append(content)
        elif isinstance(content, list):
            for item in content:
                if isinstance(item, dict) and item.get("type") == "text":
                    chunks.append(str(item.get("text", "")))
    return "\n".join(chunks)


def extract_context(payload: Dict[str, Any]) -> Dict[str, Any]:
    context = payload.get("context")
    if isinstance(context, dict):
        return context
    messages = payload.get("messages", [])
    if not isinstance(messages, list):
        return {}
    text = _message_text(messages)
    marker = text.find('{"instruction"')
    if marker >= 0:
        try:
            parsed, _ = json.JSONDecoder().raw_decode(text[marker:])
            if isinstance(parsed, dict):
                return parsed
        except Exception:
            pass
    parsed = first_json_object(text)
    return parsed if isinstance(parsed, dict) else {}


def valid_actions_from_context(context: Dict[str, Any]) -> List[Dict[str, Any]]:
    actions = context.get("valid_actions", [])
    if not isinstance(actions, list):
        return []
    return [action for action in actions if isinstance(action, dict) and action.get("action_id")]


def is_fleet_context(context: Dict[str, Any]) -> bool:
    vehicles = context.get("automated_vehicles")
    vehicle_ids = context.get("automated_vehicle_ids")
    return isinstance(vehicles, list) or isinstance(vehicle_ids, list)


def fleet_vehicles_from_context(context: Dict[str, Any]) -> List[Dict[str, Any]]:
    vehicles = context.get("automated_vehicles", [])
    if not isinstance(vehicles, list):
        return []
    output: List[Dict[str, Any]] = []
    for vehicle in vehicles:
        if not isinstance(vehicle, dict):
            continue
        try:
            vehicle_id = int(vehicle.get("vehicle_id"))
        except Exception:
            continue
        actions = vehicle.get("valid_actions", [])
        if not isinstance(actions, list):
            actions = []
        output.append({
            "vehicle_id": vehicle_id,
            "valid_actions": [action for action in actions if isinstance(action, dict) and action.get("action_id")],
        })
    return output


def choose_fallback_action(actions: Sequence[Dict[str, Any]]) -> Tuple[str, str]:
    if not actions:
        return "", "no valid action was provided"
    cost_ranked = [action for action in actions if _finite_bundle_cost(action) is not None]
    if cost_ranked:
        progress_types = {"SELECT_SPOT_AND_CRUISE", "PARK", "REROUTE", "CRUISE_TO_EXIT"}
        progress_ranked = [action for action in cost_ranked if action.get("action_type") in progress_types]
        ranked = progress_ranked or cost_ranked
        selected = min(ranked, key=lambda action: (_finite_bundle_cost(action), _conflict_risk(action), _expected_wait(action)))
        return str(selected.get("action_id", "")), "fallback selected lowest bundle_cost action"
    preferred_order = ["SELECT_SPOT_AND_CRUISE", "PARK", "REROUTE", "WAIT", "CRUISE_TO_EXIT"]
    for action_type in preferred_order:
        for action in actions:
            if action.get("action_type") == action_type:
                return str(action.get("action_id", "")), "fallback selected %s" % action_type
    return str(actions[0].get("action_id", "")), "fallback selected first valid action"


def _finite_bundle_cost(action: Dict[str, Any]) -> Any:
    features = action.get("features", {}) if isinstance(action, dict) else {}
    try:
        value = float(features.get("bundle_cost"))
    except Exception:
        return None
    if value != value or value in (float("inf"), float("-inf")):
        return None
    return value


def _conflict_risk(action: Dict[str, Any]) -> float:
    try:
        return float((action.get("features") or {}).get("conflict_risk", 1e9))
    except Exception:
        return 1e9


def _expected_wait(action: Dict[str, Any]) -> float:
    try:
        return float((action.get("features") or {}).get("expected_wait_s", 1e9))
    except Exception:
        return 1e9


def action_by_id(actions: Sequence[Dict[str, Any]], action_id: str) -> Dict[str, Any]:
    for action in actions:
        if str(action.get("action_id", "")) == str(action_id):
            return action
    return {}


def action_target(action: Dict[str, Any]) -> Any:
    value = action.get("target_spot_index")
    if value is None or value == "":
        return None
    try:
        return int(value)
    except Exception:
        return None


def reason_code_for_action(action: Dict[str, Any], fallback: bool = False) -> str:
    if fallback:
        return "FALLBACK_OR_RECOVERY"
    action_type = action.get("action_type")
    if action_type == "SELECT_SPOT_AND_CRUISE":
        try:
            if float(action.get("duration") or 0.0) > 0.0:
                return "YIELD_TRAFFIC"
        except Exception:
            pass
        return "PARK_AVAILABLE"
    if action_type == "PARK":
        return "PARK_READY"
    if action_type == "REROUTE":
        return "REROUTE_CONFLICT"
    if action_type == "CRUISE_TO_EXIT":
        return "EXIT_READY"
    if action_type == "WAIT":
        return "YIELD_TRAFFIC"
    return "FALLBACK_OR_RECOVERY"


def int_or_none(value: Any) -> Any:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except Exception:
        return None


def normalize_decision(text: str, actions: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    parsed = first_json_object(text)
    valid_ids = {str(action.get("action_id")) for action in actions}
    action_id = str(parsed.get("action_id", "")) if parsed else ""
    reason = str(parsed.get("reason", "")) if parsed else ""
    reason_code = str(parsed.get("reason_code", "")) if parsed else ""
    parsed_target = int_or_none(parsed.get("target_spot_index")) if parsed else None
    try:
        confidence = float(parsed.get("confidence", 0.0)) if parsed else 0.0
    except Exception:
        confidence = 0.0
    fallback = False
    if action_id not in valid_ids:
        fallback_id, fallback_reason = choose_fallback_action(actions)
        reason = "model output invalid action_id %r; %s" % (action_id, fallback_reason)
        action_id = fallback_id
        confidence = 0.0
        fallback = True
    selected = action_by_id(actions, action_id)
    selected_target = action_target(selected)
    if not fallback and parsed_target is not None and selected_target != parsed_target:
        fallback_id, fallback_reason = choose_fallback_action(actions)
        reason = "model output target_spot_index %r mismatched action_id %r; %s" % (parsed_target, action_id, fallback_reason)
        action_id = fallback_id
        selected = action_by_id(actions, action_id)
        selected_target = action_target(selected)
        confidence = 0.0
        fallback = True
    if not reason_code:
        reason_code = reason_code_for_action(selected, fallback=fallback)
    elif fallback:
        reason_code = "FALLBACK_OR_RECOVERY"
    return {
        "action_id": action_id,
        "target_spot_index": selected_target,
        "reason_code": reason_code,
        "reason": reason,
        "confidence": clamp(confidence, 0.0, 1.0),
    }


def normalize_fleet_decisions(text: str, context: Dict[str, Any]) -> Dict[str, Any]:
    parsed = first_json_object(text)
    raw_decisions = parsed.get("fleet_decisions") if isinstance(parsed, dict) else None
    if raw_decisions is None and isinstance(parsed, dict):
        raw_decisions = parsed.get("decisions")
    if raw_decisions is None and isinstance(parsed, dict) and parsed.get("action_id") is not None:
        vehicles = fleet_vehicles_from_context(context)
        vehicle_id = vehicles[0]["vehicle_id"] if vehicles else -1
        raw_decisions = [dict(parsed, vehicle_id=vehicle_id)]
    if not isinstance(raw_decisions, list):
        raw_decisions = []
    raw_by_vehicle: Dict[int, Dict[str, Any]] = {}
    for item in raw_decisions:
        if not isinstance(item, dict):
            continue
        try:
            raw_by_vehicle[int(item.get("vehicle_id"))] = item
        except Exception:
            continue

    output: List[Dict[str, Any]] = []
    reserved_targets: Dict[int, int] = {}
    for priority, vehicle in enumerate(fleet_vehicles_from_context(context)):
        vehicle_id = int(vehicle["vehicle_id"])
        actions = vehicle["valid_actions"]
        raw = raw_by_vehicle.get(vehicle_id, {})
        decision = _normalize_fleet_vehicle_decision(raw, actions, vehicle_id, priority)
        target = decision.get("target_spot_index")
        if target is not None:
            spot = abs(int(target))
            if spot in reserved_targets:
                fallback_id, fallback_reason = choose_fallback_action(_actions_without_reserved(actions, reserved_targets))
                selected = action_by_id(actions, fallback_id)
                decision = {
                    "vehicle_id": vehicle_id,
                    "action_id": fallback_id,
                    "target_spot_index": action_target(selected),
                    "priority": priority,
                    "reason_code": "FALLBACK_OR_RECOVERY",
                    "reason": "fleet duplicate target %s already assigned to vehicle %s; %s" % (spot, reserved_targets[spot], fallback_reason),
                    "confidence": 0.0,
                    "used_fallback": True,
                }
                target = decision.get("target_spot_index")
            if target is not None:
                reserved_targets[abs(int(target))] = vehicle_id
        output.append(decision)
    return {"fleet_decisions": output}


def _normalize_fleet_vehicle_decision(raw: Dict[str, Any], actions: Sequence[Dict[str, Any]], vehicle_id: int, priority: int) -> Dict[str, Any]:
    valid_ids = {str(action.get("action_id")) for action in actions}
    action_id = str(raw.get("action_id", "")) if isinstance(raw, dict) else ""
    reason = str(raw.get("reason", "")) if isinstance(raw, dict) else ""
    reason_code = str(raw.get("reason_code", "")) if isinstance(raw, dict) else ""
    parsed_target = int_or_none(raw.get("target_spot_index")) if isinstance(raw, dict) else None
    try:
        confidence = float(raw.get("confidence", 0.0)) if isinstance(raw, dict) else 0.0
    except Exception:
        confidence = 0.0
    fallback = False
    if action_id not in valid_ids:
        fallback_id, fallback_reason = choose_fallback_action(actions)
        reason = "model output invalid fleet action_id %r for vehicle %s; %s" % (action_id, vehicle_id, fallback_reason)
        action_id = fallback_id
        confidence = 0.0
        fallback = True
    selected = action_by_id(actions, action_id)
    selected_target = action_target(selected)
    if not fallback and parsed_target is not None and selected_target != parsed_target:
        fallback_id, fallback_reason = choose_fallback_action(actions)
        reason = "model output fleet target_spot_index %r mismatched action_id %r for vehicle %s; %s" % (parsed_target, action_id, vehicle_id, fallback_reason)
        action_id = fallback_id
        selected = action_by_id(actions, action_id)
        selected_target = action_target(selected)
        confidence = 0.0
        fallback = True
    if not reason_code:
        reason_code = reason_code_for_action(selected, fallback=fallback)
    elif fallback:
        reason_code = "FALLBACK_OR_RECOVERY"
    return {
        "vehicle_id": int(vehicle_id),
        "action_id": action_id,
        "target_spot_index": selected_target,
        "priority": int(raw.get("priority", priority) if isinstance(raw, dict) else priority),
        "reason_code": reason_code,
        "reason": reason,
        "confidence": clamp(confidence, 0.0, 1.0),
        "used_fallback": bool(fallback),
    }


def _actions_without_reserved(actions: Sequence[Dict[str, Any]], reserved_targets: Dict[int, int]) -> List[Dict[str, Any]]:
    output = []
    for action in actions:
        target = action_target(action)
        if target is None or abs(int(target)) not in reserved_targets:
            output.append(action)
    return output or list(actions)


def _actions_without_cloud_reservations(actions: Sequence[Dict[str, Any]], reservations: Dict[int, Dict[str, Any]], vehicle_id: int) -> List[Dict[str, Any]]:
    output = []
    for action in actions:
        target = action_target(action)
        if target is None:
            output.append(action)
            continue
        owner = reservations.get(abs(int(target)))
        if owner is None or int(owner.get("vehicle_id", -1)) == int(vehicle_id):
            output.append(action)
    return output or list(actions)


def openai_response(model: str, decision: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "id": "parksim-qwen-vla-%d" % int(time.time() * 1000),
        "object": "chat.completion",
        "created": int(time.time()),
        "model": model,
        "choices": [
            {
                "index": 0,
                "message": {
                    "role": "assistant",
                    "content": json.dumps(decision, ensure_ascii=False),
                },
                "finish_reason": "stop",
            }
        ],
    }


def _decode_data_url(value: str) -> Any:
    if not value.startswith("data:"):
        return value
    try:
        _, encoded = value.split(",", 1)
        image_bytes = base64.b64decode(encoded)
        from PIL import Image

        return Image.open(BytesIO(image_bytes)).convert("RGB")
    except Exception:
        return value


def normalize_messages(messages: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    normalized: List[Dict[str, Any]] = []
    for message in messages:
        role = str(message.get("role", "user"))
        content = message.get("content", "")
        if isinstance(content, str):
            normalized.append({"role": role, "content": [{"type": "text", "text": content}]})
            continue
        if not isinstance(content, list):
            normalized.append({"role": role, "content": [{"type": "text", "text": str(content)}]})
            continue
        qwen_content: List[Dict[str, Any]] = []
        for item in content:
            if not isinstance(item, dict):
                continue
            item_type = item.get("type")
            if item_type == "text":
                qwen_content.append({"type": "text", "text": str(item.get("text", ""))})
            elif item_type == "image":
                qwen_content.append({"type": "image", "image": item.get("image")})
            elif item_type == "image_url":
                image_url = item.get("image_url", {})
                image_value = image_url.get("url", "") if isinstance(image_url, dict) else str(image_url)
                qwen_content.append({"type": "image", "image": _decode_data_url(str(image_value))})
        if not qwen_content:
            qwen_content.append({"type": "text", "text": ""})
        normalized.append({"role": role, "content": qwen_content})
    return normalized


class QwenVLAInferenceService:
    def __init__(
        self,
        model_id: str = DEFAULT_MODEL_ID,
        device_map: str = "auto",
        torch_dtype: str = "auto",
        max_new_tokens: int = 512,
        mock: bool = False,
    ):
        self.model_id = resolve_model_id(model_id)
        self.device_map = device_map
        self.torch_dtype = torch_dtype
        self.max_new_tokens = int(max_new_tokens)
        self.mock = bool(mock)
        self.model = None
        self.processor = None
        self._target_reservations: Dict[int, Dict[str, Any]] = {}
        self._reservation_ttl_seconds = 180.0

    def load(self) -> None:
        if self.mock or self.model is not None:
            return
        import torch
        from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration

        dtype: Any = self.torch_dtype
        if self.torch_dtype == "bf16":
            dtype = torch.bfloat16
        elif self.torch_dtype == "fp16":
            dtype = torch.float16
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            self.model_id,
            torch_dtype=dtype,
            device_map=self.device_map,
        )
        self.processor = AutoProcessor.from_pretrained(self.model_id)

    def decide(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        context = extract_context(payload)
        if is_fleet_context(context):
            if self.mock:
                return self._apply_cross_request_reservations(normalize_fleet_decisions("{}", context), context)
            self.load()
            messages = normalize_messages(payload.get("messages", []))
            output = self._generate(messages)
            return self._apply_cross_request_reservations(normalize_fleet_decisions(output, context), context)
        actions = valid_actions_from_context(context)
        if self.mock:
            action_id, reason = choose_fallback_action(actions)
            selected = action_by_id(actions, action_id)
            return {
                "action_id": action_id,
                "target_spot_index": action_target(selected),
                "reason_code": reason_code_for_action(selected),
                "reason": "mock qwen service: " + reason,
                "confidence": 1.0 if action_id else 0.0,
            }
        self.load()
        messages = normalize_messages(payload.get("messages", []))
        output = self._generate(messages)
        return normalize_decision(output, actions)

    def _apply_cross_request_reservations(self, response: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        now = time.time()
        self._expire_reservations(now)
        vehicles = {int(row["vehicle_id"]): row for row in fleet_vehicles_from_context(context)}
        output: List[Dict[str, Any]] = []
        for raw_decision in response.get("fleet_decisions", []):
            if not isinstance(raw_decision, dict):
                continue
            decision = dict(raw_decision)
            vehicle_id = int(decision.get("vehicle_id", -1))
            actions = vehicles.get(vehicle_id, {"valid_actions": []}).get("valid_actions", [])
            selected = action_by_id(actions, decision.get("action_id", ""))
            target = action_target(selected)
            if target is None:
                target = int_or_none(decision.get("target_spot_index"))
            if target is not None:
                spot = abs(int(target))
                owner = self._target_reservations.get(spot)
                if owner and int(owner.get("vehicle_id", -1)) != vehicle_id:
                    fallback_actions = _actions_without_cloud_reservations(actions, self._target_reservations, vehicle_id)
                    fallback_id, fallback_reason = choose_fallback_action(fallback_actions)
                    fallback_selected = action_by_id(actions, fallback_id)
                    decision.update({
                        "action_id": fallback_id,
                        "target_spot_index": action_target(fallback_selected),
                        "priority": int(decision.get("priority", len(output)) or len(output)),
                        "reason_code": "FALLBACK_OR_RECOVERY",
                        "reason": "cloud reservation conflict: spot %s already reserved by vehicle %s; %s" % (spot, owner.get("vehicle_id"), fallback_reason),
                        "confidence": 0.0,
                        "used_fallback": True,
                    })
                    target = action_target(fallback_selected)
                if target is not None:
                    self._target_reservations[abs(int(target))] = {"vehicle_id": vehicle_id, "expires_at": now + self._reservation_ttl_seconds}
            else:
                self._release_vehicle_reservations(vehicle_id)
            output.append(decision)
        return {"fleet_decisions": output}

    def _expire_reservations(self, now: float) -> None:
        for spot, owner in list(self._target_reservations.items()):
            if float(owner.get("expires_at", 0.0)) <= now:
                del self._target_reservations[spot]

    def _release_vehicle_reservations(self, vehicle_id: int) -> None:
        for spot, owner in list(self._target_reservations.items()):
            if int(owner.get("vehicle_id", -1)) == int(vehicle_id):
                del self._target_reservations[spot]

    def _generate(self, messages: List[Dict[str, Any]]) -> str:
        if self.model is None or self.processor is None:
            raise RuntimeError("model is not loaded")
        from qwen_vl_utils import process_vision_info

        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        image_inputs, video_inputs = process_vision_info(messages)
        inputs = self.processor(
            text=[text],
            images=image_inputs,
            videos=video_inputs,
            padding=True,
            return_tensors="pt",
        )
        inputs = inputs.to(self.model.device)
        generated_ids = self.model.generate(
            **inputs,
            max_new_tokens=self.max_new_tokens,
            do_sample=False,
        )
        generated_ids_trimmed = [out_ids[len(in_ids) :] for in_ids, out_ids in zip(inputs.input_ids, generated_ids)]
        return self.processor.batch_decode(generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]


def make_handler(service: QwenVLAInferenceService):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.rstrip("/") in ("", "/healthz"):
                self._send_json({"ok": True, "model": service.model_id, "mock": service.mock})
                return
            self.send_error(404, "not found")

        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            raw_body = self.rfile.read(length)
            try:
                payload = json.loads(raw_body.decode("utf-8"))
            except Exception as exc:
                self._send_json({"error": "invalid json: %s" % exc}, status=400)
                return
            try:
                decision = service.decide(payload)
                model = str(payload.get("model") or service.model_id)
                self._send_json(openai_response(model, decision))
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=500)

        def _send_json(self, payload: Dict[str, Any], status: int = 200) -> None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, format, *args):
            return

    return Handler


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a local OpenAI-compatible Qwen VLA service for ParkSim.")
    parser.add_argument("--host", default=os.environ.get("QWEN_VLA_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("QWEN_VLA_PORT", "8000")))
    parser.add_argument("--model-id", default=os.environ.get("QWEN_MODEL_ID", DEFAULT_MODEL_ID))
    parser.add_argument("--device-map", default=os.environ.get("QWEN_DEVICE_MAP", "auto"))
    parser.add_argument("--torch-dtype", default=os.environ.get("QWEN_TORCH_DTYPE", "auto"), choices=["auto", "bf16", "fp16"])
    parser.add_argument("--max-new-tokens", type=int, default=int(os.environ.get("QWEN_MAX_NEW_TOKENS", "512")))
    parser.add_argument("--mock", action="store_true", help="Serve deterministic valid decisions without loading a model.")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    service = QwenVLAInferenceService(
        model_id=args.model_id,
        device_map=args.device_map,
        torch_dtype=args.torch_dtype,
        max_new_tokens=args.max_new_tokens,
        mock=args.mock,
    )
    service.load()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    host, port = server.server_address
    print("qwen vla service listening on http://%s:%d/v1/chat/completions" % (host, port), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
