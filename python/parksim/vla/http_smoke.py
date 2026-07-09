import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

from parksim.vla.qwen_client import QwenPolicyClient
from parksim.vla.schema import VLA_DECISION_PROTOCOL_VERSION, VLAActionType, VLACandidateAction, VLAContext

REQUESTS = []


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        REQUESTS.append(payload)
        body = {
            "choices": [
                {"message": {"content": json.dumps({"action_id": "cruise_to_spot_1", "target_spot_index": 1, "reason_code": "PARK_AVAILABLE", "reason": "mock", "confidence": 0.9})}}
            ]
        }
        response = json.dumps(body).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def log_message(self, format, *args):
        return


def main():
    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    endpoint = "http://127.0.0.1:%d" % server.server_port
    client = QwenPolicyClient(endpoint=endpoint, timeout=2.0)
    context = VLAContext(
        instruction="choose action",
        state={"ego": {"task": None}},
        valid_actions=[
            VLACandidateAction(action_id="wait_2s", action_type=VLAActionType.WAIT, duration=2.0),
            VLACandidateAction(action_id="cruise_to_spot_1", action_type=VLAActionType.SELECT_SPOT_AND_CRUISE, target_spot_index=1),
        ],
    )
    decision = client.decide(context)
    server.shutdown()
    thread.join(timeout=2.0)
    assert decision.action_id == "cruise_to_spot_1", decision
    assert decision.target_spot_index == 1, decision
    assert decision.reason_code == "PARK_AVAILABLE", decision
    assert REQUESTS and REQUESTS[0]["context"]["protocol_version"] == VLA_DECISION_PROTOCOL_VERSION
    assert REQUESTS[0]["context"]["valid_action_ids"] == ["wait_2s", "cruise_to_spot_1"]
    print("parksim.vla http smoke ok")


if __name__ == "__main__":
    main()
