#!/usr/bin/env python3
"""A simulated `drone_agent.py prepare --resident` for the ground station's DEMO mode. Contacts nothing.

It speaks the same json-lines protocol as the real resident agent (tools/drone_agent.py resident_loop): one
`prepare` report, then commands on stdin -- ARM <token> [at], START <token> [at], STATUS, HOLD, LAND, ABORT rtl|land,
DISARM, QUIT -- each answered with a {"verb": ...} line. Demo-only words: DEMO_TAKEOVER (the pilot drops ch5),
DEMO_LINK (this aircraft stops answering), DEMO_NOTREADY is a flag (--demo-not-ready) for the barrier.

DEMO_SPEED (default 4) speeds the mission up.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

SPEED = float(os.environ.get("DEMO_SPEED", "4"))
WP_S = 6.0 / SPEED            # seconds per waypoint
END_S = 8.0 / SPEED           # RTL / land duration


def emit(d: dict) -> None:
    print(json.dumps(d), flush=True)


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("verb")
    ap.add_argument("--link"), ap.add_argument("--slot", type=int), ap.add_argument("--plan")
    ap.add_argument("--run-id"), ap.add_argument("--failsafe-s", type=float, default=60)
    ap.add_argument("--resident", action="store_true"), ap.add_argument("--expect-sysid", type=int)
    ap.add_argument("--demo-not-ready", action="store_true")
    a = ap.parse_args()
    sysid = a.expect_sysid or a.slot
    try:
        items = len(json.loads(Path(a.plan).read_text())["mission"]["items"])
    except (OSError, ValueError, KeyError):
        items = 10
    log(f"[link] {a.link} -- waiting for heartbeat")
    time.sleep(0.6)
    log(f"[link] connected, vehicle is sysid {sysid} (DEMO)")
    log(f"[plan] {Path(a.plan).name}: {items} items (DEMO upload)")
    time.sleep(1.0 + 0.4 * (sysid % 3))
    token = f"{a.run_id}:demo{sysid}"
    ready = not a.demo_not_ready
    checks = [dict(name=n, ok=True, detail="DEMO") for n in
              ("identity", "mission_uploaded", "fence_ceiling", "rtl_altitude", "rc_loss_action", "position_estimate",
               "battery")]
    checks.insert(5, dict(name="gps_fix", ok=ready, detail="DEMO fix_type=3" if ready else "DEMO fix_type=0"))
    emit(dict(verb="prepare", slot=a.slot, sysid=sysid, ready=ready, plan=a.plan, items=items, top_alt_m=30.0,
              token=token, checks=checks, reasons=[c["name"] for c in checks if not c["ok"]]))

    s = dict(armed=False, main=3, sub=None, wp=0, alt=0.0, t_wp=0.0, phase="ground", end_at=None, mute=False,
             batt=91)
    lock = threading.Lock()

    def tick():
        while True:
            time.sleep(0.2)
            with lock:
                now = time.time()
                if s["phase"] == "mission" and now - s["t_wp"] >= WP_S:
                    s["t_wp"] = now
                    s["wp"] += 1
                    s["alt"] = 30.0
                    if s["wp"] >= items - 1:
                        s.update(phase="rtl", sub=5, end_at=now + END_S)
                if s["phase"] in ("rtl", "land", "pilot") and s["end_at"] and now >= s["end_at"]:
                    s.update(phase="ground", armed=False, alt=0.0, end_at=None)
                    log("[demo] landed and disarmed")

    threading.Thread(target=tick, daemon=True).start()

    def wait_at(parts):
        at = float(parts[2]) if len(parts) > 2 else None
        if at and at > time.time():
            time.sleep(at - time.time())

    for raw in sys.stdin:
        parts = raw.split()
        if not parts:
            continue
        verb = parts[0].upper()
        with lock:
            mute = s["mute"]
        if verb in ("ARM", "START"):
            if (parts[1] if len(parts) > 1 else "") != token:
                emit(dict(verb=verb.lower(), slot=a.slot, sysid=sysid, armed=False, started=False,
                          error="token mismatch"))
                continue
            wait_at(parts)
            time.sleep(0.3 + 0.2 * (sysid % 2))
            with lock:
                s["armed"] = True
                if verb == "START":
                    s.update(main=4, sub=4, phase="mission", t_wp=time.time(), wp=1)
            log(f"[{verb.lower()}] armed" + (", MISSION_START accepted" if verb == "START" else ""))
            emit(dict(verb=verb.lower(), slot=a.slot, sysid=sysid, armed=True, started=verb == "START",
                      t_epoch=time.time()))
        elif verb == "STATUS":
            if mute:
                continue
            time.sleep(0.4)
            with lock:
                emit(dict(verb="status", slot=a.slot, sysid=sysid, armed=s["armed"], main_mode=s["main"],
                          sub_mode=s["sub"], waypoint=s["wp"], rel_alt_m=s["alt"], lat=1.4172, lon=103.8636,
                          battery_pct=s["batt"]))
        elif verb == "HOLD":
            with lock:
                if s["armed"]:
                    s.update(main=4, sub=3, phase="hold")
            log("[safety] operator said HOLD -> AUTO.LOITER (DEMO)")
        elif verb in ("ABORT", "LAND"):
            action = "land" if verb == "LAND" else (parts[1] if len(parts) > 1 else "rtl")
            with lock:
                if s["armed"]:
                    s.update(main=4, sub=5 if action == "rtl" else 6, phase="rtl" if action == "rtl" else "land",
                             end_at=time.time() + END_S)
            if verb == "ABORT":
                emit(dict(verb="abort", slot=a.slot, sysid=sysid, action=action, accepted=True))
        elif verb == "DISARM":
            time.sleep(0.3)
            with lock:
                s.update(armed=False, phase="ground", main=3, sub=None)
            emit(dict(verb="disarm", slot=a.slot, sysid=sysid, disarmed=True))
        elif verb == "DEMO_TAKEOVER":
            with lock:
                s.update(main=2, sub=None, phase="pilot", end_at=time.time() + 3 * END_S)
            emit(dict(event="rc_takeover", slot=a.slot, from_mode=4, to_mode=2, airborne=True))
            emit(dict(event="takeover_classified", slot=a.slot, source="pilot_switch", after_s=0.4, airborne=True))
            log("[safety] PILOT HAS IT (pilot switch) (DEMO)")
        elif verb == "DEMO_LINK":
            with lock:
                s["mute"] = True
            log("[demo] this aircraft stops answering (link lost)")
        elif verb == "QUIT":
            emit(dict(verb="quit", slot=a.slot, sysid=sysid))
            return 0
        else:
            emit(dict(verb="error", slot=a.slot, error=f"unknown command {verb!r}"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
