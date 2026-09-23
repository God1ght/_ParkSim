#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ParkSim-JTH Docker 镜像冒烟探针（WebSocket）。

流程：
  1) 连 ws://<host>:<port>/ws，读第一条 init，检查 sim_state / spawn / spots / scene_token
  2) 发 {"type":"restart", config:{... map=jth_b1, 入2出2, seed=<seed>}}
  3) 观察 status 序列 restarting -> started -> running，并等待出现「帧里车辆数 > 0」
     （证明 publish/subscribe、自定义 msg/srv、jth_b1 地图资产、逐泊位机动表全链路可用）
  4) 跑 RUN_SECONDS 秒后发 {"type":"stop"}，等待 status=stopped
  5) 打印结构化结论；全部通过 exit 0，否则 exit 1

用法：
  python3 ws_probe.py http://127.0.0.1:8098 [seed] [run_seconds]
"""
import asyncio
import json
import sys
import time

import aiohttp

URL = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8098"
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 12345
RUN_SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0


def log(*args):
    print("[probe %s]" % time.strftime("%H:%M:%S"), *args, flush=True)


async def main():
    ws_url = URL.replace("http://", "ws://").replace("https://", "wss://").rstrip("/") + "/ws"
    log("connecting", ws_url)

    timeout = aiohttp.ClientTimeout(total=None, sock_connect=15)
    checks = {}
    statuses = []
    frames = 0
    frames_with_vehicles = 0
    max_vehicles = 0
    first_after_start = None      # restart 之后第一帧有车的帧
    pos_before = None             # 某车在 restart 后的位置
    pos_after = None              # 同一车稍后的位置（证明真在动）
    sim_t_first = None
    sim_t_last = None

    async with aiohttp.ClientSession(timeout=timeout) as sess:
        async with sess.ws_connect(ws_url, heartbeat=30) as ws:
            # ---------- 1) init ----------
            init = None
            deadline = time.time() + 25
            while time.time() < deadline:
                msg = await ws.receive()
                if msg.type != aiohttp.WSMsgType.TEXT:
                    if msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                        log("FATAL ws closed while waiting init")
                        return 1
                    continue
                d = json.loads(msg.data)
                if d.get("type") == "init":
                    init = d
                    break
                if d.get("type") == "status":
                    statuses.append(d.get("value"))
                    log("status(before init):", d)

            if init is None:
                log("FATAL no init payload received")
                return 1

            checks["init.sim_state present"] = "sim_state" in init
            checks["init.spawn present"] = "spawn" in init
            checks["init.spots non-empty"] = bool(init.get("spots"))
            checks["init.waypoints non-empty"] = bool(init.get("waypoints"))
            log("init: type=%s scene_token=%r sim_state=%r spawn=%r spots=%d waypoints=%s"
                % (init.get("type"), init.get("scene_token"), init.get("sim_state"),
                   init.get("spawn"), len(init.get("spots") or []),
                   list((init.get("waypoints") or {}).keys())))
            log("init.options.map=%r has_experience=%r obstacle_mode=%r base_map_url=%r gates=%d"
                % ((init.get("options") or {}).get("map"),
                   (init.get("options") or {}).get("has_experience"),
                   (init.get("options") or {}).get("obstacle_mode"),
                   (init.get("options") or {}).get("base_map_url"),
                   len(init.get("gates") or [])))

            # ---------- 2) restart: jth_b1 / 入2出2 / 指定 seed ----------
            config = {
                "map": "jth_b1",
                "init_mode": "random",
                "allocation_method": "random",
                "route_planner": "astar",
                "maneuver_provider": "per_spot",
                "params": {"random": {"seed": SEED, "entering": 2, "exiting": 2,
                                      "interval_mean": 3.0}},
            }
            log("sending restart:", json.dumps(config, ensure_ascii=False))
            await ws.send_str(json.dumps({"type": "restart", "config": config}))

            # ---------- 3) 等 started + 车辆出现（只看 restart 之后的帧） ----------
            t0 = time.time()
            restart_started = False
            stopped_sent = False
            while True:
                remaining = 30 if not stopped_sent else 25
                try:
                    msg = await asyncio.wait_for(ws.receive(), timeout=remaining)
                except asyncio.TimeoutError:
                    log("waiting timeout (stopped_sent=%s)" % stopped_sent)
                    break
                if msg.type != aiohttp.WSMsgType.TEXT:
                    if msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                        log("FATAL ws closed")
                        break
                    continue
                d = json.loads(msg.data)
                t = d.get("type")
                if t == "status":
                    statuses.append(d.get("value"))
                    if d.get("value") == "started":
                        restart_started = True
                    log("status:", d.get("value"), d.get("message") or "")
                elif t == "frame":
                    if not stopped_sent:
                        frames += 1
                        n = len(d.get("vehicles") or [])
                        max_vehicles = max(max_vehicles, n)
                        if n > 0:
                            frames_with_vehicles += 1
                        # 只统计 restart 生效之后的帧的 sim 时间：restarting 期间旧仿真
                        # 仍在吐帧（t 是旧值，可能上百秒），若一并纳入会把 sim_t_first
                        # 记成旧时间，导致 “sim_time progressed” 误判为 FAIL。
                        if restart_started and d.get("t") is not None:
                            if sim_t_first is None:
                                sim_t_first = d["t"]
                            sim_t_last = d["t"]
                        if restart_started and n > 0 and first_after_start is None:
                            first_after_start = d
                            log("FIRST VEHICLE FRAME AFTER RESTART: t=%s seq=%s vehicles=%d %s"
                                % (d.get("t"), d.get("seq"), n,
                                   json.dumps(d.get("vehicles")[:2], ensure_ascii=False)[:400]))
                        elif restart_started and first_after_start is not None:
                            _vid = first_after_start["vehicles"][0].get("id")
                            for v in d.get("vehicles") or []:
                                if v.get("id") != _vid:
                                    continue
                                if pos_before is None:
                                    pos_before = (v.get("x"), v.get("y"), d.get("t"))
                                elif pos_after is None and (time.time() - t0) > (RUN_SECONDS - 8):
                                    pos_after = (v.get("x"), v.get("y"), d.get("t"))
                elif t in ("stopped", "stop_failed"):
                    log("recv:", json.dumps(d, ensure_ascii=False)[:200])

                # restart 后出现车辆 + 跑够 RUN_SECONDS → 发 stop
                if (not stopped_sent) and first_after_start is not None \
                        and (time.time() - t0) > RUN_SECONDS:
                    log("sending stop (frames=%d frames_with_vehicles=%d max_vehicles=%d)"
                        % (frames, frames_with_vehicles, max_vehicles))
                    await ws.send_str(json.dumps({"type": "stop"}))
                    stopped_sent = True
                if stopped_sent and "stopped" in statuses and (time.time() - t0) > RUN_SECONDS + 5:
                    break
                if (time.time() - t0) > RUN_SECONDS * 3 + 90:
                    log("global timeout, aborting wait loop")
                    break

    # ---------- 4) 结论 ----------
    checks["status started"] = "started" in statuses
    checks["status running"] = "running" in statuses
    checks["frame received (pre-stop)"] = frames > 0
    checks["vehicle appeared AFTER restart"] = first_after_start is not None
    checks["sim_time progressed (restart->pre-stop)"] = (sim_t_first is not None and sim_t_last is not None
                                               and sim_t_last > sim_t_first)
    checks["status stopped after stop"] = "stopped" in statuses

    log("---- summary ----")
    log("statuses:", statuses)
    log("frames=%d frames_with_vehicles=%d max_vehicles=%d sim_t=[%s..%s]"
        % (frames, frames_with_vehicles, max_vehicles, sim_t_first, sim_t_last))
    if first_after_start is not None:
        v0 = first_after_start["vehicles"][0]
        log("sample vehicle:", json.dumps(v0, ensure_ascii=False))
    if pos_before and pos_after:
        log("vehicle motion: %s -> %s" % (pos_before, pos_after))
    ok = True
    for k, v in checks.items():
        log("CHECK %-34s %s" % (k, "PASS" if v else "FAIL"))
        ok = ok and bool(v)
    log("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        sys.exit(130)
