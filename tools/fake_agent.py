#!/usr/bin/env python3
"""A stand-in for `drone_agent.py offboard-flight` that talks to nothing. DEMO ONLY (GUI, 4 Nov 2026).

Takes the real agent's arguments (it ignores the link) and speaks its protocol: JSON events on
stdout, [tags] on stderr, `ARM` / `ABORT` / `LAND` ... on stdin, one result object at the end.
The GUI's demo mode runs this instead of ssh, so every screen can be built and tested without an
aircraft. It simulates the controller's plan from tools/gs_core.timeline; it measures nothing.

    DEMO_SPEED=4 python3 tools/fake_agent.py offboard-flight --stage disarm --height 0.8 ...
Extra stdin words: DEMO_TAKEOVER (the pilot drops ch5), DEMO_KILL (the pilot holds the kill).
"""
import argparse
import json
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gs_core  # noqa: E402

SPEED = float(os.environ.get("DEMO_SPEED", "4"))


def emit(o):
    print(json.dumps(o), flush=True)


def log(m):
    print(m, file=sys.stderr, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("verb")
    ap.add_argument("--stage", default="takeoff")
    ap.add_argument("--wait-s", type=float, default=15.0)
    ap.add_argument("--height", type=float, default=1.0)
    ap.add_argument("--hold-s", type=float, default=5.0)
    ap.add_argument("--takeoff-pause-m", type=float, default=0.3)
    ap.add_argument("--test-descent-rate", type=float, default=0.08)
    ap.add_argument("--final-descent-rate", type=float, default=0.05)
    ap.add_argument("--expect-sysid", type=int, default=113)
    ap.add_argument("--link", default="demo")
    ap.add_argument("--trace", default=None)
    a, _ = ap.parse_known_args()

    prof = gs_core.normalise(dict(name="demo", stage=a.stage, height_m=a.height, hold_s=a.hold_s, wait_s=a.wait_s,
                                  takeoff_pause_m=a.takeoff_pause_m, test_descent_rate=a.test_descent_rate,
                                  final_descent_rate=a.final_descent_rate))
    tl = gs_core.timeline(prof)
    log("[link] DEMO -- no aircraft -- waiting for heartbeat")
    time.sleep(0.6 / SPEED * 4)
    log(f"[link] connected, vehicle is sysid {a.expect_sysid} (simulated)")
    emit(dict(event="stage", slot=None, stage="prearm", under_test=False, height_m=0.0, t_epoch=time.time()))
    log("[ladder] stage: prearm")
    result = dict(verb="offboard-flight", stage=a.stage, slot=None, sysid=a.expect_sysid, outcome=None,
                  takeover_source=None, takeover_stage=None, simulator=True, demo=True)
    words = []

    def poll():
        while select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                words.append("EOF")
                return
            words.append(line.strip().upper())

    if a.stage == "prearm":
        end = time.time() + a.wait_s / SPEED
        while time.time() < end and "DEMO_TAKEOVER" not in words:
            poll(); time.sleep(0.1)
        result.update(outcome="takeover" if "DEMO_TAKEOVER" in words else "no_takeover", passed=True)
        emit(result)
        return 0

    emit(dict(event="ready", slot=None, sysid=a.expect_sysid, stage=a.stage))
    log(f"[ladder] READY for test '{a.stage}' -- type ARM once props/tether/area are confirmed")
    end = time.time() + 90
    while time.time() < end and not words:
        poll(); time.sleep(0.1)
    if not words or words[0] != "ARM":
        result.update(error=f"not authorised to arm ({words[0] if words else 'timeout'})")
        emit(result)
        return 2
    words.clear()
    emit(dict(event="stage", slot=None, stage="arm", under_test=False, height_m=0.0, t_epoch=time.time()))
    log("[ladder] stage: arm")
    time.sleep(1.2 / SPEED * 3)
    log("[vehicle] Armed by external command")
    t_arm = time.time()
    emit(dict(event="armed", slot=None, sysid=a.expect_sysid, armed_epoch=round(t_arm, 3)))
    names = {"spool": "arm", "climb": "takeoff", "hold": "hold", "descent": "descent", "landing": "landing"}
    last_stage, tick, last_batt = None, 0, -30.0
    total = tl["total_s"] if tl["flies"] else a.wait_s
    pilot = None
    while True:
        poll()
        t = (time.time() - t_arm) * SPEED
        for w in list(words):
            if w in ("DEMO_TAKEOVER", "DEMO_KILL"):
                pilot = w
            elif w in ("ABORT", "LAND"):
                log(f"[safety] operator said {w} -> LAND")
                emit(dict(event="safety", slot=None, action="land", carried_out="land", reasons=[f"operator said {w}"]))
                result.update(outcome="guard_acted", safety=[{"action": "land"}])
                t = total + 1
            words.remove(w)
        if pilot:
            src = "kill_switch" if pilot == "DEMO_KILL" else "pilot_switch"
            emit(dict(event="rc_takeover", slot=None, via=src, airborne=t > tl["marks"][0][2] if tl["flies"] else False))
            log("[safety] PILOT HAS IT (%s)." % src.replace("_", " "))
            time.sleep(0.4 / SPEED * 2)
            emit(dict(event="takeover_classified", slot=None, source=src, after_s=0.4))
            emit(dict(event="still_armed", slot=None, waited_s=0.0))
            log("[safety] *** AIRCRAFT IS STILL ARMED *** The pilot has it and the agent will not disarm it.")
            time.sleep(2.0 / SPEED * 2)
            emit(dict(event="pilot_disarmed", slot=None, waited_s=2.0))
            result.update(outcome="takeover", takeover_source=src, takeover_stage=last_stage,
                          takeover_after_s=round(t, 2), disarmed=True)
            break
        if tl["flies"] and t - last_batt >= 30:
            last_batt = t
            emit(dict(event="battery", slot=None, volts=round(16.2 - 0.0008 * t, 2), pct=max(0, round(95 - t / 40)),
                      stage=last_stage, t_s=round(t, 1), t_epoch=time.time()))
        if tl["flies"]:
            cur = next((n for n, lo, hi in tl["marks"] if lo <= t < hi), None)
            if cur and cur != "spool" and names[cur] != last_stage:
                last_stage = names[cur]
                emit(dict(event="stage", slot=None, stage=last_stage, under_test=(last_stage == a.stage), height_m=0.0,
                          t_epoch=time.time()))
                log(f"[ladder] stage: {last_stage}")
        if t >= total:
            if result["outcome"] is None:
                result["outcome"] = "landed_px4_disarmed" if tl["flies"] and a.stage == "disarm" else "no_takeover"
            if tl["flies"]:
                log("[vehicle] Landing detected")
                log("[vehicle] Disarmed by landing")
            break
        time.sleep(0.1)
    result.update(passed=result["outcome"] in ("takeover", "landed_px4_disarmed", "no_takeover", "guard_acted"),
                  link_losses=0, setpoints=dict(sent=int(total * 20), dropped_sim=0, hz=20.0),
                  max_height_m=round(prof["height_m"] if tl["flies"] else 0.0, 2), disarmed=True)
    emit(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
