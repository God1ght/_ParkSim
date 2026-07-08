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


def choose_fallback_action(actions: Sequence[Dict[str, Any]]) -> Tuple[str, str]:
    if not actions:
        return "", "no valid action was provided"
    preferred_order = ["SELECT_SPOT_AND_CRUISE", "PARK", "REROUTE", "WAIT", "CRUISE_TO_EXIT"]
    for action_type in preferred_order:
        for action in actions:
            if action.get("action_type") == action_type:
                return str(action.get("action_id", "")), "fallback selected %s" % action_type
    return str(actions[0].get("action_id", "")), "fallback selected first valid action"


def normalize_decision(text: str, actions: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    parsed = first_json_object(text)
    valid_ids = {str(action.get("action_id")) for action in actions}
    action_id = str(parsed.get("action_id", "")) if parsed else ""
    reason = str(parsed.get("reason", "")) if parsed else ""
    try:
        confidence = float(parsed.get("confidence", 0.0)) if parsed else 0.0
    except Exception:
        confidence = 0.0
    if action_id not in valid_ids:
        fallback_id, fallback_reason = choose_fallback_action(actions)
        reason = "model output invalid action_id %r; %s" % (action_id, fallback_reason)
        action_id = fallback_id
        confidence = 0.0
    return {
        "action_id": action_id,
        "reason": reason,
        "confidence": clamp(confidence, 0.0, 1.0),
    }


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
        max_new_tokens: int = 128,
        mock: bool = False,
    ):
        self.model_id = resolve_model_id(model_id)
        self.device_map = device_map
        self.torch_dtype = torch_dtype
        self.max_new_tokens = int(max_new_tokens)
        self.mock = bool(mock)
        self.model = None
        self.processor = None

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
        actions = valid_actions_from_context(context)
        if self.mock:
            action_id, reason = choose_fallback_action(actions)
            return {"action_id": action_id, "reason": "mock qwen service: " + reason, "confidence": 1.0 if action_id else 0.0}
        self.load()
        messages = normalize_messages(payload.get("messages", []))
        output = self._generate(messages)
        return normalize_decision(output, actions)

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
    parser.add_argument("--max-new-tokens", type=int, default=int(os.environ.get("QWEN_MAX_NEW_TOKENS", "128")))
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
