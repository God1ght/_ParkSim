import threading

from parksim.vla.qwen_client import QwenPolicyClient
from parksim.vla.qwen_service import QwenVLAInferenceService, make_handler
from parksim.vla.schema import VLAActionType, VLACandidateAction, VLAContext


def main():
    from http.server import ThreadingHTTPServer

    service = QwenVLAInferenceService(mock=True)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(service))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    endpoint = "http://127.0.0.1:%d/v1/chat/completions" % server.server_port
    client = QwenPolicyClient(endpoint=endpoint, timeout=2.0)
    context = VLAContext(
        instruction="choose action",
        state={"ego": {"task": None}},
        valid_actions=[
            VLACandidateAction(action_id="wait_2s", action_type=VLAActionType.WAIT, duration=2.0),
            VLACandidateAction(
                action_id="cruise_to_spot_1",
                action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
                target_spot_index=1,
            ),
        ],
    )
    try:
        decision = client.decide(context)
    finally:
        server.shutdown()
        thread.join(timeout=2.0)
    assert decision.action_id == "cruise_to_spot_1", decision
    assert not decision.used_fallback, decision
    print("parksim.vla qwen service smoke ok")


if __name__ == "__main__":
    main()
