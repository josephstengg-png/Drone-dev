#!/usr/bin/env python3
"""The staged response (descend, then cut) against disturbances, on the ARTEX model, indoors.

    python3 sim/staged_tests.py --instances 10 --out /tmp/staged.json [--only gust,tumble]

Runs ON jetson-2 next to PX4 SITL + Gazebo. SIMULATOR ONLY (sim_tests.Sim refuses a vehicle
without SIM_GZ_EN). Every case boots a fresh simulator configured as the real aircraft flies
indoors: GPS fusion off from power-up, pilot-flown Altitude mode, option A, plus the proposed
PX4 backstop (CBRK_FLIGHTTERM off, tumble 60 deg for 1.0 s).

The guard under test is the REAL one: drone_agent.Guard with --allow-kill, fed every message
the vehicle sends, carrying out its own LAND and CUT. Only the pilot is emulated (sticks over
MANUAL_CONTROL).

Disturbances, each randomised per instance (magnitude, direction, which motor):
  healthy_hover    nothing                                 expect: no action
  pilot_maneuvers  full-stick roll/pitch/yaw, short climb  expect: no action
  gust             sideways push, 0.5 s                    expect: recover, no cut
  weak_motor       one motor limited to ~70 % speed        expect: no cut
  motor_out        one motor dead (SIM_GZ_EC_MAX = min)    expect: cut (guard or PX4)
  tumble           roll/pitch torque pulse                 expect: cut
  surge            upward push, 4 s, stick centred         expect: descend, no cut
  ceiling          pilot holds the stick up past 2 m       expect: descend, no cut
  runaway          sustained lift > weight, nobody reacts   expect: descend, cut >= 5 s later
  runaway_pilot    same, pilot flips to Altitude 1.5 s in   expect: guard stands down, no cut
                   (the push stops at the takeover: the pilot has regained control)
Pushes go through Gazebo's ApplyLinkWrench (world frame, at the body origin).
"""
from __future__ import annotations

import argparse
import json
import math
import random
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path.home() / "FlightPlanner" / "tools"))
import sim_tests as T            # noqa: E402
import drone_agent as ag         # noqa: E402

WORLD = "default"
OMEGA_MIN, OMEGA_MAX = 150, 2939
# Gazebo frame (x forward, y LEFT): motor n+1 -> (x, y, spin)
MOTORS = {1: (0.0683, -0.100, "ccw"), 2: (-0.0683, 0.100, "ccw"),
          3: (0.0683, 0.100, "cw"), 4: (-0.0683, -0.100, "cw")}
INDOOR = {"EKF2_GPS_CTRL": 0, "CBRK_FLIGHTTERM": 0, "FD_FAIL_P": 60, "FD_FAIL_R": 60,
          "FD_FAIL_P_TTRI": 1.0, "FD_FAIL_R_TTRI": 1.0, "COM_FAIL_ACT_T": 0,
          "NAV_DLL_ACT": 3, "NAV_RCL_ACT": 3, "GF_ACTION": 5, "COM_RC_IN_MODE": 3,
          "COM_RC_LOSS_T": 0.5, "COM_DISARM_PRFLT": 10, "COM_DL_LOSS_T": 5}
SCENARIOS = ["healthy_hover", "pilot_maneuvers", "gust", "weak_motor", "motor_out",
             "tumble", "surge", "ceiling", "runaway", "runaway_pilot"]
EXPECT = {"healthy_hover": "none", "pilot_maneuvers": "none", "gust": "no_cut",
          "weak_motor": "no_cut", "motor_out": "cut", "tumble": "cut", "surge": "land",
          "ceiling": "land", "runaway": "cut_after_margin", "runaway_pilot": "pilot"}


class Pusher:
    """Pushes on the model through ONE persistent gz-transport node.

    26 Oct 2026: one-shot `gz topic -p` calls were lost before discovery finished, so some
    "clear" messages never arrived -- the push stayed on, lifted a disarmed aircraft to 14 m and
    spun tumbling ones until Gazebo's physics aborted. A node that lives for the whole case, and
    a clear sent three times, fixes both.
    """

    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        from gz.msgs10.entity_pb2 import Entity
        self.EW, self.E = EntityWrench, Entity
        self.node = Node()
        self.pub_push = self.node.advertise(f"/world/{WORLD}/wrench/persistent", EntityWrench)
        self.pub_clear = self.node.advertise(f"/world/{WORLD}/wrench/clear", Entity)
        time.sleep(1.0)                   # let discovery finish before the first publish

    def push(self, link, fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0):
        m = self.EW()
        m.entity.name, m.entity.type = link, self.E.LINK
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = fx, fy, fz
        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = tx, ty, tz
        self.pub_push.publish(m)

    def clear(self, link):
        m = self.E()
        m.name, m.type = link, self.E.LINK
        for _ in range(3):
            self.pub_clear.publish(m)
            time.sleep(0.05)


PUSHER: Pusher | None = None


def push(link, **kw):
    PUSHER.push(link, **kw)


def clear(link):
    PUSHER.clear(link)


def model_link():
    out = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=10).stdout
    names = [w.strip(" -") for w in out.split() if w.strip(" -").startswith("artex")]
    return f"{names[0] if names else 'artex_0'}::base_link"


def boot():
    T.stop_sim()
    T.start_sim(gui=True, params=INDOOR, ready="Startup script returned successfully")
    s = T.Sim()
    # 115 HIL_STATE_QUATERNION = Gazebo ground truth: the indoor estimator drifts after a tumble
    for msg_id, us in ((36, 50000), (132, 100000), (32, 50000), (30, 50000), (245, 200000), (115, 40000)):
        s.cmd(511, msg_id, us, wait=1.0)
    if s.param("EKF2_GPS_CTRL") != 0:
        raise RuntimeError("GPS fusion is not off")
    s.wait(lambda st: st["mode"] is not None, 20)
    s.pump(3.0)
    s.z0 = s.s["z"]; s.s["z"] = 0.0
    return s


class Case:
    def __init__(self, s, rng):
        self.s, self.rng, self.t0 = s, rng, time.time()
        self.events, self.trace = [], []
        args = SimpleNamespace(allow_kill=True, motor_range=f"{OMEGA_MIN},{OMEGA_MAX}",
                               ceiling_m=2.0, abort_flag="/tmp/staged_noflag", slot="sim",
                               no_safety=False, throttle_chan=3)
        self.g = ag.Guard(s.c, args)
        ag.emit = lambda ev: self.events.append(dict(ev, t=round(time.time() - self.t0, 2)))
        s.on_msg = self.g.observe
        self.motor_max = 0.0
        self.truth = None                 # (true height m, true vz up m/s, true tilt deg)
        self.alt0 = None
        s.on_msg = self._msg

    def _msg(self, m):
        self.g.observe(m)
        if m.get_type() == "HIL_STATE_QUATERNION":
            w, x, y, z = m.attitude_quaternion
            # tilt = angle between body z and world z
            cz = 1 - 2 * (x * x + y * y)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, cz))))
            if self.alt0 is None:
                self.alt0 = m.alt
            self.truth = ((m.alt - self.alt0) / 1000.0, -m.vz / 100.0, tilt)
        if m.get_type() == "SERVO_OUTPUT_RAW" and self.s.s["armed"]:
            outs = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw]
            self.motor_max = max(self.motor_max, max((v - OMEGA_MIN) / (OMEGA_MAX - OMEGA_MIN) for v in outs))

    def step(self, sec):
        end = time.time() + sec
        while time.time() < end:
            self.s.pump(0.05)
            self.g.wd.stick(self.s.mc[2] / 1000.0, time.time())
            self.g.check()
            st = self.s.s
            h, vz, tilt = self.truth or (st["z"], st["vz"], max(abs(st["roll"]), abs(st["pitch"])))
            self.trace.append((round(time.time() - self.t0, 2), st["armed"], st["mode"], round(h, 2),
                               round(vz, 2), round(tilt, 1), round(st["z"], 2)))


def run_case(name, rng):
    global PUSHER
    s = boot()
    link = model_link()
    PUSHER = Pusher()
    c = Case(s, rng)
    s.rc_on = True
    s.mode("ALTCTL")
    if not T.arm_when_ready(s):
        s.close()
        return dict(scenario=name, reached=False, verdict="NOT REACHED", why="would not arm",
                    texts=[t for _, _, t in s.texts][-4:])
    # PX4 starts a stick take-off in Altitude only above ~65 % throttle (650 sat on the line and
    # never left the ground, 26 Oct): 750 to lift off, ease to 600 near the height
    h = lambda: (c.truth or (s.s["z"],))[0]               # noqa: E731
    # PX4 spools slowly: full stick only until it leaves the ground (full stick to 0.9 m
    # overshot to the 2 m ceiling, 26 Oct), then a gentle climb, then hold
    s.mc[2] = 1000
    end = time.time() + 15
    while time.time() < end and h() < 0.15:
        c.step(0.1)
    s.mc[2] = 700                                       # 650 sits on PX4's take-off threshold
    end = time.time() + 10
    while time.time() < end and h() < 0.9:
        c.step(0.1)
    s.mc[2] = 500
    c.step(6.0)
    reached = s.s["armed"] and h() > 0.4
    detail, t_dist, h_before = {}, round(time.time() - c.t0, 2), round(h(), 2)
    if reached:
        detail = disturb(name, s, c, link, rng)
        c.step(25.0 if name != "pilot_maneuvers" else 8.0)
    clear(link)
    s.mc = [0, 0, 500, 0]
    if s.s["armed"]:
        s.land(); c.step(20.0)
    if s.s["armed"]:
        s.disarm(force=True); s.pump(2.0)
    crashed = time.time() - s.s["t_hb"] > 3.0          # Gazebo/PX4 stopped talking
    acts = [e for e in c.events if e.get("event") == "safety"]
    tr = [r for r in c.trace if r[0] >= t_dist]
    res = dict(scenario=name, reached=reached, disturbance=detail, t_disturbance=t_dist, height_before=h_before,
               guard=[dict(t=round(e["t"] - t_dist, 2), action=e["action"], did=e["carried_out"],
                           why=e["reasons"][-2:]) for e in acts],
               max_tilt=max((r[5] for r in tr), default=None),
               max_height=max((r[3] for r in tr), default=None),
               fastest_fall=min((r[4] for r in tr), default=None),
               motor_max=round(c.motor_max, 2),
               disarmed_at=next((round(r[0] - t_dist, 2) for r in tr if not r[1]), None),
               final=s.snapshot(), sim_crashed=crashed,
               final_true_tilt=round(c.truth[2], 1) if c.truth else None,
               estimator_height_max=max((r[6] for r in tr), default=None),
               texts=[t for _, _, t in s.texts if "logger" not in t][-8:])
    res["px4_terminated"] = any(r[2] == "10.0" for r in tr)    # main mode 10 = TERMINATION
    res["pilot_has_it"] = c.g.pilot_has_it
    res["countdown"] = [e["seconds"] for e in c.events if e.get("event") == "cut_pending"][:8]
    res["cut_by"] = ("guard" if any(a["action"] in ("cut", "kill") for a in res["guard"]) else
                     "px4" if res["px4_terminated"] else
                     "px4" if any("termination" in t.lower() or "failure detected" in t.lower()
                                  for t in res["texts"]) else "none")
    res["verdict"] = judge(name, res)
    s.close()
    try:                                                  # keep each case's PX4 + Gazebo log
        log = T.GZB_DIR / "sitl.log"
        Path(f"/tmp/staged_logs/{name}_{int(c.t0)}.log").parent.mkdir(exist_ok=True)
        Path(f"/tmp/staged_logs/{name}_{int(c.t0)}.log").write_bytes(log.read_bytes())
        res["log"] = f"/tmp/staged_logs/{name}_{int(c.t0)}.log"
    except OSError:
        pass
    return res


def disturb(name, s, c, link, rng):
    if name == "healthy_hover":
        return {}
    if name == "pilot_maneuvers":
        for x, y, z, r in ((0, 1000, 500, 0), (0, -1000, 500, 0), (1000, 0, 500, 0),
                           (-1000, 0, 500, 0), (0, 0, 500, 1000), (0, 0, 700, 0), (0, 0, 300, 0)):
            s.mc = [x, y, z, r]; c.step(1.5)
        s.mc = [0, 0, 500, 0]
        return {"sticks": "roll, pitch, yaw full; climb/descend"}
    if name == "gust":
        f, a = rng.uniform(3.0, 6.0), rng.uniform(0, 2 * math.pi)
        push(link, fx=f * math.cos(a), fy=f * math.sin(a)); c.step(0.5); clear(link)
        return {"force_N": round(f, 2), "dir_deg": round(math.degrees(a))}
    if name in ("weak_motor", "motor_out"):
        n = rng.randint(1, 4)
        if name == "weak_motor":
            frac = rng.uniform(0.65, 0.75)
            cap = int(OMEGA_MIN + frac * (OMEGA_MAX - OMEGA_MIN))
        else:
            frac, cap = 0.0, OMEGA_MIN
        s.param(f"SIM_GZ_EC_MAX{n}", cap)
        return {"motor": n, "max_speed_fraction": round(frac, 2), "cap_rad_s": cap}
    if name == "tumble":
        # a short knock past the motors' authority (~1.2 N m roll). The gz CLI adds ~0.1-0.3 s
        # of its own to every push, so the real duration is longer than dur_s.
        tq, a, dur = rng.uniform(1.0, 1.5), rng.uniform(0, 2 * math.pi), rng.uniform(0.05, 0.15)
        push(link, tx=tq * math.cos(a), ty=tq * math.sin(a)); c.step(dur); clear(link)
        return {"torque_Nm": round(tq, 2), "axis_deg": round(math.degrees(a)), "dur_s": round(dur, 2)}
    if name == "surge":
        # near the 4.9 N weight, 2.5 s: more than PX4 can cancel, but over before a 3 s descent.
        # (4 s of 5 N lifted it to 5 m at idle and the guard cut it, 26 Oct: a push stronger than
        # gravity outlasting the descent is not physical.)
        # 3-3.8 N: past what PX4's height loop cancels (2.8 N for 4 s rose only 0.5 m), short of the
        # 4.9 N weight (4-5 N threw it 9-14 m: a test of the push, not of the aircraft)
        f = rng.uniform(3.0, 3.8)
        push(link, fz=f); c.step(2.5); clear(link)
        return {"up_force_N": round(f, 2), "dur_s": 2.5}
    if name in ("runaway", "runaway_pilot"):
        f, t_land, took = rng.uniform(5.0, 5.5), None, None   # just over the weight: climbs at idle
        push(link, fz=f)
        end = time.time() + 14
        while time.time() < end and s.s["armed"]:
            c.step(0.1)
            if t_land is None and any(e.get("action") == "land" for e in c.events if e.get("event") == "safety"):
                t_land = time.time()
            if name == "runaway_pilot" and t_land and took is None and time.time() - t_land >= 1.5:
                s.mode("ALTCTL"); clear(link); took = round(time.time() - t_land, 2)
                break
        clear(link)
        return {"up_force_N": round(f, 2), "pilot_took_over_s_after_descend": took}
    if name == "ceiling":
        s.mc = [0, 0, 850, 0]                       # the pilot keeps climbing
        return {"stick": 0.85}
    raise ValueError(name)


def judge(name, r):
    if r.get("sim_crashed"):
        return "SIM CRASHED"
    if not r["reached"]:
        return "NOT REACHED"
    acts = {a["action"] for a in r["guard"]}
    cut = r["cut_by"] != "none"
    exp = EXPECT[name]
    if exp == "none":
        return "ok" if not acts - {"warn"} and not cut else "FALSE ALARM"
    if exp == "no_cut":
        return "ok" if not cut else "FALSE CUT"
    if exp == "land":
        if cut:
            return "CUT (expected descend only)"
        if "land" in acts:
            return "ok"
        rise = (r.get("max_height") or 0) - (r.get("height_before") or 0)
        return "ok (PX4 held height)" if rise < 0.5 else "MISSED"
    if exp == "cut_after_margin":
        land = next((a["t"] for a in r["guard"] if a["action"] == "land"), None)
        kill = next((a["t"] for a in r["guard"] if a["action"] in ("cut", "kill")), None)
        if kill is None:
            return "MISSED (no cut)"
        return "ok" if land is not None and kill - land >= 4.8 else f"CUT EARLY ({kill - (land or 0):.1f}s)"
    if exp == "pilot":
        return "ok" if not cut and r.get("pilot_has_it") else ("CUT DESPITE PILOT" if cut else "guard did not stand down")
    if exp == "cut":
        if cut:
            return "ok"
        flipped = (r.get("final_true_tilt") or 0) > 90
        return "MISSED (flipped, no cut)" if flipped else "ok (recovered upright, no cut needed)"
    return "?"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instances", type=int, default=10)
    ap.add_argument("--only", default="")
    ap.add_argument("--seed", type=int, default=26)
    ap.add_argument("--out", default="/tmp/staged.json")
    args = ap.parse_args()
    names = [n for n in SCENARIOS if not args.only or n in args.only.split(",")]
    rng = random.Random(args.seed)
    out = dict(started=time.strftime("%Y-%m-%d %H:%M:%S"), boot_params=INDOOR, cases=[])
    for i in range(1, args.instances + 1):
        for name in names:
            try:
                r = run_case(name, rng)
            except Exception as exc:
                r = dict(scenario=name, reached=False, verdict="ERROR", error=repr(exc))
            r["instance"] = i
            out["cases"].append(r)
            print(f"[{i:2d}] {name:16s} {r['verdict']:28s} guard={[(a['t'], a['action']) for a in r.get('guard', [])]} "
                  f"cut_by={r.get('cut_by')} tilt={r.get('max_tilt')} h={r.get('max_height')} "
                  f"motor={r.get('motor_max')} disarm={r.get('disarmed_at')}", flush=True)
            Path(args.out).write_text(json.dumps(out, indent=1, default=str))
    T.stop_sim()
    out["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    Path(args.out).write_text(json.dumps(out, indent=1, default=str))
    print("DONE", args.out, flush=True)


if __name__ == "__main__":
    main()
