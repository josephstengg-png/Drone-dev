#!/usr/bin/env python3
"""The takeover ladder in SITL (29 Oct 2026, docs/takeover-ladder-plan.md).

    python3 sim/takeover_tests.py --instances 10 --out /tmp/ladder.json [--only hold,descent]
                                  [--outage 10,0.5,1.5]

Runs ON jetson-2 (PX4 SITL + Gazebo, the ARTEX model). Stop elrs-link.service first: the relay
binds the same MAVLink ports.

Each case boots a fresh simulator and runs the REAL agent as a subprocess --
`drone_agent.py offboard-flight --stage <stage>` on the offboard port (14540), exactly the command
line the aircraft will get, with ELRS-like setpoint outages injected (--sim-outage). This script
is the PILOT, on the ground-station port (14550): it holds an RC transmitter through
RC_CHANNELS_OVERRIDE at 20 Hz (sticks centred, ch7 centre = Offboard, ch8 = kill off), and at the
stage under test flips ch7 to -100 (Altitude) or ch8 to kill, as the plan says. PX4 sees a real RC
mode switch and kill switch (RC_MAP_FLTMODE 7, RC_MAP_KILL_SW 8), so the agent's guard sees the
same RC_CHANNELS and mode changes it would on the aircraft.

Scenarios: the seven stages, plus `no_takeover` (stage hold, the pilot does nothing): the agent
must hold, land and disarm on its own, with no false alarm.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import subprocess
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_tests as T            # noqa: E402
from staged_tests import INDOOR  # noqa: E402

AGENT = Path.home() / "FlightPlanner" / "tools" / "drone_agent.py"
PILOT_LINK = "udpin:0.0.0.0:14550"
LOGS = Path("/tmp/ladder_logs")
SCENARIOS = ["prearm", "arm", "takeoff", "hold", "descent", "landing", "disarm", "no_takeover"]
KILL = {"prearm", "arm", "takeoff", "disarm"}       # the rest: ch7 to Altitude, then fly it
# The real handset's layout (113's parameters, 9 Oct; ch7 positions since 14 Oct). RC in SITL is
# the pilot's RC_CHANNELS_OVERRIDE, so RC only (COM_RC_IN_MODE 0), and no MANUAL_CONTROL.
RC = {"RC_MAP_ROLL": 1, "RC_MAP_PITCH": 2, "RC_MAP_THROTTLE": 3, "RC_MAP_YAW": 4,
      "RC_MAP_FLTMODE": 7, "RC_MAP_KILL_SW": 8, "COM_FLTMODE1": 1, "COM_FLTMODE4": 7,
      "COM_FLTMODE6": 2, "COM_RC_IN_MODE": 0, "COM_OF_LOSS_T": 1.0, "COM_OBL_RC_ACT": 0,
      "RC_CHAN_CNT": 8}   # 0 = "RC not calibrated": PX4 then ignores the mode switch (29 Oct)
LO, MID, HI = 1000, 1500, 2000


class Pilot:
    """The handset and the pilot's eyes, on the ground-station port."""

    def __init__(self):
        self.c = mavutil.mavlink_connection(PILOT_LINK, source_system=255, source_component=190)
        self.c.wait_heartbeat(timeout=30)
        self.ch = {3: MID, 7: MID, 8: LO}
        self.s = dict(armed=False, mode=None, z=None, landed=None, tilt=0.0, alt=None, t_hb=time.time())
        self.alt0, self.z0 = None, None
        self.modes = []                  # (t, main mode, armed)
        self.texts = []
        self.pvals, self.rc = {}, None
        self.max_tilt = 0.0
        # truth beside what the aircraft believes: (t, Gazebo height, rangefinder, estimator -z)
        self.track, self.max_alt, self.rng, self.est = [], 0.0, None, None
        self._stop = False
        self.lock = threading.Lock()
        threading.Thread(target=self._tx, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()
        for msg_id, us in ((115, 40000), (245, 200000), (32, 50000), (132, 50000)):
            with self.lock:
                self.c.mav.command_long_send(self.c.target_system, 1, 511, 0, msg_id, us, 0, 0, 0, 0, 0)

    def _tx(self):
        n = 0
        while not self._stop:
            ch = [MID, MID, self.ch[3], MID, MID, MID, self.ch[7], self.ch[8]]
            with self.lock:
                self.c.mav.rc_channels_override_send(self.c.target_system, 1, *ch)
                if n % 20 == 0:
                    self.c.mav.heartbeat_send(6, 8, 0, 0, 0)
            n += 1
            time.sleep(0.05)

    def _rx(self):
        while not self._stop:
            with self.lock:
                m = self.c.recv_match(blocking=False)
            if m is None:
                time.sleep(0.005)
                continue
            if m.get_srcSystem() != self.c.target_system:
                continue
            k, now = m.get_type(), time.time()
            if k == "HEARTBEAT" and m.type != 6:
                armed, mode = bool(m.base_mode & 128), (m.custom_mode >> 16) & 0xFF
                if (mode, armed) != (self.s["mode"], self.s["armed"]):
                    self.modes.append((now, mode, armed))
                self.s.update(armed=armed, mode=mode, t_hb=now)
            elif k == "EXTENDED_SYS_STATE":
                self.s["landed"] = m.landed_state
            elif k == "HIL_STATE_QUATERNION":          # Gazebo ground truth
                w, x, y, z = m.attitude_quaternion
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
                if self.alt0 is None:
                    self.alt0 = m.alt
                self.s.update(alt=(m.alt - self.alt0) / 1000.0, tilt=tilt)
                if self.s["armed"]:
                    self.max_tilt = max(self.max_tilt, tilt)
                    self.max_alt = max(self.max_alt, self.s["alt"])
                    self.track.append((round(now, 3), round(self.s["alt"], 3), self.rng, self.est))
            elif k == "DISTANCE_SENSOR":
                d = m.current_distance / 100.0
                self.rng = round(d, 3) if m.min_distance < m.current_distance < m.max_distance else None
            elif k == "LOCAL_POSITION_NED":
                self.est = round(-m.z, 3)
            elif k == "PARAM_VALUE":
                self.pvals[m.param_id] = m.param_value
            elif k == "RC_CHANNELS":
                self.rc = [getattr(m, f"chan{i}_raw") for i in range(1, 9)]
            elif k == "STATUSTEXT":
                t = m.text if isinstance(m.text, str) else m.text.decode(errors="replace")
                self.texts.append((round(now, 2), t.strip()))

    def height(self):
        return self.s["alt"] if self.s["alt"] is not None else 0.0

    def force_disarm(self):
        with self.lock:
            self.c.mav.command_long_send(self.c.target_system, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)

    def close(self):
        self._stop = True
        time.sleep(0.2)
        self.c.close()


class Agent:
    """drone_agent.py offboard-flight, as a subprocess; its JSON events collected."""

    def __init__(self, stage, seed, outage, wait_s, tag, extra=()):
        LOGS.mkdir(exist_ok=True)
        cmd = [sys.executable, str(AGENT), "offboard-flight", "--link", "udpin:0.0.0.0:14540",
               "--stage", stage, "--wait-s", str(wait_s), "--seed", str(seed), "--slot", "1",
               "--abort-flag", "/tmp/ladder_noflag", "--pilot-disarm-window-s", "30",
               "--trace", str(LOGS / f"{tag}.csv")]
        if outage:
            cmd += ["--sim-outage", outage]
        cmd += list(extra)
        self.stderr = open(LOGS / f"{tag}.agent.log", "w")
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=self.stderr, text=True)
        self.events, self.result = [], None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            try:
                ev = json.loads(line)
            except ValueError:
                continue
            ev["_t"] = time.time()
            if ev.get("verb") == "offboard-flight":
                self.result = ev
            else:
                self.events.append(ev)

    def stage(self, name):
        return next((e for e in self.events if e.get("event") == "stage" and e["stage"] == name), None)

    def stop(self):
        if self.p.poll() is None:
            self.p.terminate()          # its own numeric PID, never a pattern
            try:
                self.p.wait(10)
            except subprocess.TimeoutExpired:
                self.p.kill()
        self.stderr.close()


def wait(pred, timeout, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(step)
    return False


def run_case(scn, seed, outage, wait_s, extra=()):
    T.stop_sim()
    T.start_sim(gui=True, params=dict(INDOOR, **RC), ready="Startup script returned successfully")
    tag = f"{scn}_{seed}"
    pilot = Pilot()
    time.sleep(3.0)                     # RC present before anyone asks for a mode
    stage = "hold" if scn == "no_takeover" else scn
    agent = Agent(stage, seed, outage, wait_s, tag, extra)
    flip = None                         # (t, what, height at flip)
    t0 = time.time()

    def do_flip(what):
        nonlocal flip
        if what == "kill":
            pilot.ch[8] = HI
        else:
            pilot.ch[7] = LO
        flip = (time.time(), what, round(pilot.height(), 2))

    try:
        started = wait(lambda: agent.stage(stage) or agent.result, 90)
        st = agent.stage(stage)
        if scn != "no_takeover" and started and st:
            if scn == "prearm":
                wait(lambda: pilot.s["mode"] == 6, 5); time.sleep(2.0); do_flip("kill")
            elif scn == "arm":
                wait(lambda: pilot.s["armed"], 10); time.sleep(2.0); do_flip("kill")
            elif scn == "takeoff":
                # in the climb: at 0.2 m the kill raced the agent reaching its 0.3 m pause (29 Oct)
                if wait(lambda: pilot.height() >= 0.1 or agent.result, 15) and not agent.result:
                    do_flip("kill")
            elif scn == "hold":
                time.sleep(3.0)
                if not agent.result:
                    do_flip("altitude")
            elif scn == "descent":
                if wait(lambda: pilot.height() <= 0.6 or agent.result, 20) and not agent.result:
                    do_flip("altitude")
            elif scn == "landing":
                if wait(lambda: pilot.height() <= 0.2 or agent.result, 20) and not agent.result:
                    do_flip("altitude")
            elif scn == "disarm":
                if not agent.result:
                    do_flip("kill")
        if flip and flip[1] == "altitude":
            # fly it: hold 4 s, descend, and kill on the ground to finish
            time.sleep(4.0)
            pilot.ch[3] = 1150
            wait(lambda: pilot.s["landed"] == 1 or not pilot.s["armed"], 20)
            time.sleep(1.0)
            pilot.ch[8] = HI
        wait(lambda: agent.result is not None, 90)
    finally:
        agent.stop()
        if pilot.s["armed"]:
            pilot.force_disarm(); time.sleep(2.0)

    r = agent.result or {}
    # how long PX4 took to act on the switch: the first mode/armed change after the flip
    latency = None
    if flip:
        for t, mode, armed in pilot.modes:
            if t >= flip[0] and ((flip[1] == "kill" and not armed) or (flip[1] == "altitude" and mode == 2)):
                latency = round(t - flip[0], 2)
                break
    crashed = pilot.max_tilt > 60 or time.time() - pilot.s["t_hb"] > 3.0
    res = dict(scenario=scn, seed=seed, outage=outage, flipped=flip[1] if flip else None,
               flip_height_m=flip[2] if flip else None, px4_latency_s=latency,
               agent_outcome=r.get("outcome"), takeover_source=r.get("takeover_source"),
               takeover_stage=r.get("takeover_stage"), link_losses=r.get("link_losses"),
               guard_actions=[a.get("action") for a in r.get("safety", [])],
               passed_by_agent=r.get("passed"), abort_reason=r.get("abort_reason"),
               setpoints=r.get("setpoints"), max_height_m=r.get("max_height_m"),
               true_max_tilt=round(pilot.max_tilt, 1), true_max_height_m=round(pilot.max_alt, 2),
               crashed=crashed,
               disarmed=not pilot.s["armed"], error=r.get("error"),
               classified=[e.get("source") for e in agent.events if e.get("event") == "takeover_classified"],
               stages=[e["stage"] for e in agent.events if e.get("event") == "stage"],
               texts=[t for _, t in pilot.texts if "logger" not in t][-10:],
               duration_s=round(time.time() - t0, 1))
    res["verdict"] = judge(scn, res)
    pilot.close()
    (LOGS / f"{tag}.truth.csv").write_text("t,true_height_m,rangefinder_m,estimator_m\n" + "".join(
        ",".join("" if v is None else str(v) for v in r) + "\n" for r in pilot.track))
    try:
        (LOGS / f"{tag}.sitl.log").write_bytes((T.GZB_DIR / "sitl.log").read_bytes())
    except OSError:
        pass
    return res


def judge(scn, r):
    if r["crashed"]:
        return "CRASH"
    if r["error"]:
        return f"ERROR: {r['error']}"
    if scn == "no_takeover":
        if r["takeover_source"] == "link_loss":
            return "VOID (link loss)"
        if r["takeover_source"] or r["guard_actions"]:
            return "FALSE ALARM"
        return "PASS" if r["agent_outcome"] == "landed_px4_disarmed" and r["disarmed"] else \
            f"FAIL ({r['agent_outcome']})"
    if r["takeover_source"] == "link_loss" or (r["link_losses"] or 0) > 0:
        return "VOID (link loss)"
    if not r["flipped"]:
        return f"NOT REACHED ({r['agent_outcome']})"
    want = "kill_switch" if scn in KILL else "pilot_switch"
    if r["takeover_source"] != want:
        return f"FAIL (source {r['takeover_source']}, wanted {want})"
    if r["takeover_stage"] != scn:
        return f"FAIL (took over in {r['takeover_stage']})"
    if r["guard_actions"]:
        return "FALSE ALARM"
    return "PASS" if r["disarmed"] else "FAIL (left armed)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instances", type=int, default=10)
    ap.add_argument("--only", default=None)
    ap.add_argument("--outage", default="10,0.5,1.5", help="'' for a perfect link")
    ap.add_argument("--wait-s", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=2910)
    ap.add_argument("--out", default="/tmp/ladder.json")
    ap.add_argument("--agent-args", default="", help="extra drone_agent.py arguments, space-separated")
    a = ap.parse_args()
    scns = a.only.split(",") if a.only else SCENARIOS
    rng = random.Random(a.seed)
    out = dict(started=time.strftime("%Y-%m-%d %H:%M:%S"), outage=a.outage, rc_params=RC,
               boot_params=INDOOR, cases=[])
    for i in range(1, a.instances + 1):
        for scn in scns:
            try:
                r = run_case(scn, rng.randrange(1 << 30), a.outage or None, a.wait_s,
                             a.agent_args.split())
            except Exception as exc:          # one broken case must not end the campaign
                r = dict(scenario=scn, verdict=f"HARNESS ERROR: {type(exc).__name__}: {exc}")
            r["instance"] = i
            out["cases"].append(r)
            Path(a.out).write_text(json.dumps(out, indent=1, default=str))
            print(f"[{i:2d}] {scn:12s} {r['verdict']:28s} src={r.get('takeover_source')} "
                  f"stage={r.get('takeover_stage')} lat={r.get('px4_latency_s')} "
                  f"losses={r.get('link_losses')} outcome={r.get('agent_outcome')} "
                  f"hmax={r.get('max_height_m')} tilt={r.get('true_max_tilt')}", flush=True)
    T.stop_sim()
    print("DONE", a.out, flush=True)


if __name__ == "__main__":
    main()
