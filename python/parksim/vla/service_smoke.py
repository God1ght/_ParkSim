import threading

from parksim.vla.fleet_client import QwenFleetPolicyClient
from parksim.vla.fleet_schema import VLAFleetContext
from parksim.vla.qwen_client import QwenPolicyClient
from parksim.vla.qwen_service import QwenVLAInferenceService, make_handler, raw_fleet_decisions
from parksim.vla.schema import VLAActionType, VLACandidateAction, VLAContext


def main():
    from http.server import ThreadingHTTPServer

    service = QwenVLAInferenceService(mock=True)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(service))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    endpoint = "http://127.0.0.1:%d/v1/chat/completions" % server.server_port
    client = QwenPolicyClient(endpoint=endpoint, timeout=2.0)
    fleet_client = QwenFleetPolicyClient(endpoint=endpoint, timeout=2.0)
    context = VLAContext(
        instruction="choose action",
        state={"ego": {"vehicle_id": 1, "task": None}},
        valid_actions=[
            VLACandidateAction(action_id="wait_2s", action_type=VLAActionType.WAIT, duration=2.0, features={"bundle_cost": 5.0}),
            VLACandidateAction(
                action_id="cruise_to_spot_1",
                action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
                target_spot_index=1,
                route_id="direct",
                features={"bundle_cost": 50.0, "conflict_risk": 0.8, "expected_wait_s": 8.0},
            ),
            VLACandidateAction(
                action_id="yield_2s_then_cruise_to_spot_2",
                action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
                target_spot_index=2,
                duration=2.0,
                route_id="yield_2s",
                features={"bundle_cost": 20.0, "conflict_risk": 0.1, "expected_wait_s": 2.0},
            ),
        ],
    )
    try:
        decision = client.decide(context)
        fleet_response = fleet_client.decide_fleet(VLAFleetContext(
            instruction="coordinate automated vehicles",
            state={"cloud_policy_role": "fleet_level_qwen_vla_server"},
            vehicle_contexts=[context],
        ))
    finally:
        server.shutdown()
        thread.join(timeout=2.0)
    assert decision.action_id == "yield_2s_then_cruise_to_spot_2", decision
    assert decision.target_spot_index == 2, decision
    assert decision.reason_code == "YIELD_TRAFFIC", decision
    assert not decision.used_fallback, decision
    assert fleet_response.decision_for(1).action_id == "yield_2s_then_cruise_to_spot_2", fleet_response.to_dict()
    raw = raw_fleet_decisions('{"fleet_decisions":[{"vehicle_id":1,"action_id":"a","target_spot_index":3},{"vehicle_id":2,"action_id":"b","target_spot_index":3}]}')
    assert len(raw["fleet_decisions"]) == 2
    assert raw["fleet_decisions"][0]["target_spot_index"] == raw["fleet_decisions"][1]["target_spot_index"]
    print("parksim.vla qwen service smoke ok")


if __name__ == "__main__":
    main()
