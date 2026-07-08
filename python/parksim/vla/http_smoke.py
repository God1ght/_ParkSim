import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

from parksim.vla.qwen_client import QwenPolicyClient
from parksim.vla.schema import VLAContext, VLACandidateAction, VLAActionType


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        self.rfile.read(length)
        body = {
            "choices": [
                {"message": {"content": json.dumps({"action_id": "cruise_to_spot_1", "reason": "mock", "confidence": 0.9})}}
            ]
        }
        payload = json.dumps(body).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

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
    assert decision.action_id == "cruise_to_spot_1", decision
    print("parksim.vla http smoke ok")


if __name__ == "__main__":
    main()
