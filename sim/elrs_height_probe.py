#!/usr/bin/env python3
"""Which way can PX4 hold height over ELRS with the handset still in charge? SITL only.

    python3 sim/elrs_height_probe.py --out /tmp/probe.json [--only vz_clean_kill,joystick_first]

Runs ON jetson-2. Stop elrs-link.service first: the relay binds the same MAVLink ports.

WHY. 2 Nov 2026: the thrust loop cannot cross ELRS (history.md, 29 Oct), so the height loop has to
live on the flight controller. Two ways to ask for that:
  joystick  Altitude mode with the agent on MANUAL_CONTROL (COM_RC_IN_MODE 2). ManualControl.cpp
            reads the mode, kill and arm switches only while RC is the selected source, so this
            probes whether the handset still works.
  vz        OFFBOARD with a climb-rate setpoint and level attitude (zero horizontal acceleration),
            no parameter changes. PX4's own position controller closes the height loop.

The vz cases fly the ELRS link as measured: 5 setpoints/s, 0.35 s late, bursty gaps, height seen
every 2 s, so the agent predicts height from its own climb rate between samples. Truth is Gazebo's.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import threading
import time
from collections import deque
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import sim_tests as T                        # noqa: E402
from staged_tests import INDOOR              # noqa: E402
from takeover_tests import HI, LO, RC, SITL_SYSID, Pilot, wait   # noqa: E402
import offboard_control as oc                # noqa: E402

AGENT_LINK = "udpin:127.0.0.1:14540"     # loopback only: see PILOT_LINK in takeover_tests.py
# ignore position, vx, vy, az, yaw; use vz, ax, ay and yaw rate
VZ_MASK = 1 | 2 | 4 | 8 | 16 | 256 | 1024
# position only, yaw rate zero: PX4 closes every loop, so the agent needs no telemetry at all to fly
POS_MASK = 8 | 16 | 32 | 64 | 128 | 256 | 1024
ELRS = dict(hz=5.0, delay_s=0.35, outage=(10.0, 0.5, 1.5), tele_s=2.0)
CLEAN = dict(hz=20.0, delay_s=0.0, outage=None, tele_s=0.05)
OFFBOARD, ALTCTL = 6, 2


class Streamer(threading.Thread):
    """The agent's end of the radio: `value` sent `hz` times a second, late and with gaps."""

    def __init__(self, conn, send, value, hz, delay_s, outage, seed=1):
        super().__init__(daemon=True)
        self.c, self.send, self.value, self.hz, self.delay_s = conn, send, value, hz, delay_s
        self.outage = oc.OutageModel(*outage, random.Random(seed)) if outage else None
        self.on, self.sent, self.dropped = True, 0, 0

    def run(self):
        queue, next_at, beat = deque(), 0.0, 0.0
        while self.on:
            now = time.time()
            if now >= beat:
                self.c.mav.heartbeat_send(6, 8, 0, 0, 0)     # GCS, as the agent identifies itself
                beat = now + 1.0
            if now >= next_at:
                next_at = now + 1.0 / self.hz
                if self.outage and self.outage.dropped(now):
                    self.dropped += 1
                else:
                    queue.append((now + self.delay_s, self.value))
            while queue and queue[0][0] <= now:
                self.send(self.c, queue.popleft()[1])
                self.sent += 1
            time.sleep(0.005)


def agent_conn():
    c = mavutil.mavlink_connection(AGENT_LINK, source_system=250, source_component=190)
    c.wait_heartbeat(timeout=30)
    if c.target_system != SITL_SYSID:
        raise RuntimeError(f"agent link answered as sysid {c.target_system}, not the simulator")
    return c


def send_vz(c, up):
    c.mav.set_position_target_local_ned_send(int(time.monotonic() * 1000) & 0xFFFFFFFF, c.target_system, 1,
                                             1, VZ_MASK, 0, 0, 0, 0, 0, -up, 0, 0, 0, 0, 0)


def send_pos(c, xyz):
    c.mav.set_position_target_local_ned_send(int(time.monotonic() * 1000) & 0xFFFFFFFF, c.target_system, 1,
                                             1, POS_MASK, *xyz, 0, 0, 0, 0, 0, 0, 0, 0)


def send_stick(c, z):
    c.mav.manual_control_send(c.target_system, 0, 0, z, 0, 0)


def set_mode(c, main):
    c.mav.command_long_send(c.target_system, 1, 176, 0, 1, main, 0, 0, 0, 0, 0)


def arm(c, pilot, forced=False):
    for _ in range(16):
        c.mav.command_long_send(c.target_system, 1, 400, 0, 1, 21196 if forced else 0, 0, 0, 0, 0, 0)
        if wait(lambda: pilot.s["armed"], 2.5):
            return True
    return False


def summary(pilot, t_from=None):
    pts = [p for p in pilot.track if t_from is None or p[0] >= t_from]
    h = [p[1] for p in pts]
    return dict(n=len(h), min_m=min(h, default=None), max_m=max(h, default=None),
                last_m=h[-1] if h else None)


def joystick(first):
    """first='joystick': MANUAL_CONTROL is heard before RC. first='rc': the handset is heard first."""
    T.start_sim(gui=False, params=dict(INDOOR, **dict(RC, COM_RC_IN_MODE=2)),
                ready="Startup script returned successfully")
    res = dict(first=first)
    c = pilot = None
    if first == "rc":
        pilot = Pilot()
        time.sleep(3.0)
    c = agent_conn()
    st = Streamer(c, send_stick, 500, 10.0, 0.0, None)
    st.start()
    time.sleep(3.0)
    if pilot is None:
        pilot = Pilot()
        time.sleep(3.0)
    set_mode(c, ALTCTL)
    res["altitude_mode"] = wait(lambda: pilot.s["mode"] == ALTCTL, 6)
    res["armed"] = arm(c, pilot, forced=True)
    st.value = 800
    time.sleep(6.0)
    st.value = 500
    time.sleep(2.0)
    res["height_after_stick_up_m"] = round(pilot.height(), 2)
    t = time.time()
    pilot.ch[8] = HI
    res["killed_by_ch8"] = wait(lambda: not pilot.s["armed"], 7)
    res["kill_latency_s"] = round(time.time() - t, 2) if res["killed_by_ch8"] else None
    pilot.ch[8] = pilot.ch[8] if res["killed_by_ch8"] else LO
    if pilot.s["armed"]:
        t = time.time()
        pilot.ch[7] = 2000                      # Position, a mode it is not in, so a change would show
        res["ch7_changed_mode"] = wait(lambda: pilot.s["mode"] != ALTCTL, 4)
        pilot.ch[7] = 1500
        st.on = False                           # the joystick stops: does the handset come back?
        time.sleep(3.0)
        pilot.ch[8] = HI
        res["kill_after_joystick_stopped"] = wait(lambda: not pilot.s["armed"], 7)
    res["texts"] = [t for _, t in pilot.texts][-8:]
    st.on = False
    if pilot.s["armed"]:
        pilot.force_disarm()
        time.sleep(2.0)
    pilot.close()
    c.close()
    return res


def vz(link, action, target=0.5, climb=0.25, hold_s=8.0, seed=7, gps=False):
    params = dict(INDOOR, **RC)
    if gps:            # outdoors: the estimator has horizontal velocity, so PX4 accepts the setpoint
        del params["EKF2_GPS_CTRL"]
    T.start_sim(gui=False, params=params, ready="Startup script returned successfully")
    pilot = Pilot()
    time.sleep(3.0)
    c = agent_conn()
    st = Streamer(c, send_vz, 0.0, link["hz"], link["delay_s"], link["outage"], seed)
    st.start()
    res = dict(action=action, link={k: v for k, v in link.items()})
    time.sleep(3.0)
    for _ in range(10):
        set_mode(c, OFFBOARD)
        if wait(lambda: pilot.s["mode"] == OFFBOARD, 2.5):
            break
    res["offboard"] = wait(lambda: pilot.s["mode"] == OFFBOARD, 25)
    res["armed"] = bool(res["offboard"]) and arm(c, pilot)
    if not res["armed"]:
        res["texts"] = [t for _, t in pilot.texts][-8:]
        st.on = False
        pilot.close()
        c.close()
        return res
    time.sleep(3.0)
    res["height_armed_idle_m"] = round(pilot.height(), 3)     # vz 0: must not leave the floor

    est0 = pilot.est or 0.0
    seen, t_seen, pipe, next_tele = 0.0, time.time(), deque(), 0.0
    cmd = climb
    st.value = climb

    def tick():
        """What the agent knows of its height: a late, sparse sample, carried on by its own command."""
        nonlocal seen, t_seen, next_tele
        now = time.time()
        if now >= next_tele:
            next_tele = now + link["tele_s"]
            h = pilot.rng if pilot.rng is not None else (pilot.est or 0.0) - est0
            pipe.append((now + link["delay_s"], now, h))
        while pipe and pipe[0][0] <= now:
            _, t_seen, seen = pipe.popleft()
        return seen + cmd * (now - t_seen)

    end = time.time() + 25
    while time.time() < end and tick() < target and pilot.s["mode"] == OFFBOARD and pilot.s["armed"]:
        time.sleep(0.02)
    res["climb_finished"] = tick() >= target
    cmd = st.value = 0.0
    t_hold = time.time()
    hold_end = t_hold + hold_s
    while time.time() < hold_end and pilot.s["mode"] == OFFBOARD and pilot.s["armed"]:
        time.sleep(0.05)
    t_settle = t_hold + 2.0
    res["hold_true"] = summary(pilot, t_settle)
    res["peak_m"] = round(pilot.max_alt, 2)
    res["mode_at_action"] = pilot.s["mode"]
    t = time.time()
    if action == "kill":
        pilot.ch[8] = HI
        ok = wait(lambda: not pilot.s["armed"], 8)
        res["kill_latency_s"] = round(time.time() - t, 2) if ok else None
    elif action == "altitude":
        pilot.ch[7] = LO
        ok = wait(lambda: pilot.s["mode"] == ALTCTL, 4)
        res["altitude_latency_s"] = round(time.time() - t, 2) if ok else None
        st.on = False
        time.sleep(4.0)
        res["height_after_takeover_m"] = round(pilot.height(), 2)
        pilot.ch[3] = 1150
        wait(lambda: pilot.s["landed"] == 1 or not pilot.s["armed"], 20)
        time.sleep(1.0)
        pilot.ch[8] = HI
        wait(lambda: not pilot.s["armed"], 8)
    else:
        cmd = st.value = -0.2
        end = time.time() + 15
        while time.time() < end and tick() > 0.15 and pilot.s["mode"] == OFFBOARD and pilot.s["armed"]:
            time.sleep(0.02)
        st.on = False
        c.mav.command_long_send(c.target_system, 1, 21, 0, 0, 0, 0, 0, 0, 0, 0)
        res["landed_and_disarmed"] = wait(lambda: not pilot.s["armed"], 25)
    st.on = False
    res.update(sent=st.sent, dropped=st.dropped, max_tilt=round(pilot.max_tilt, 1),
               modes=[(round(t_ - t_hold, 1), m, a) for t_, m, a in pilot.modes],
               texts=[t for _, t in pilot.texts if "logger" not in t][-8:], disarmed=not pilot.s["armed"])
    if pilot.s["armed"]:
        pilot.force_disarm()
        time.sleep(2.0)
    Path(f"/tmp/probe_track_{action}_{link['hz']:.0f}.csv").write_text(
        "t,true_height_m,rangefinder_m,estimator_m,drift_m\n" +
        "".join(",".join("" if v is None else str(v) for v in p) + "\n" for p in pilot.track))
    res["true_max_drift_m"] = round(pilot.max_drift, 2)
    pilot.close()
    c.close()
    return res


def position(link, action, target=0.5, hold_s=10.0, seed=7):
    """GPS on: the agent repeats one position setpoint; the radio carries nothing that must be timely."""
    params = dict(INDOOR, **RC)
    del params["EKF2_GPS_CTRL"]
    T.start_sim(gui=False, params=params, ready="Startup script returned successfully")
    pilot = Pilot()
    time.sleep(3.0)
    c = agent_conn()
    start = None
    end = time.time() + 30
    while time.time() < end and start is None:
        m = c.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        if m and time.time() > end - 20:     # let the estimator settle first
            start = (m.x, m.y, m.z)
    res = dict(action=action, link=dict(link), start=start)
    st = Streamer(c, send_pos, start, link["hz"], link["delay_s"], link["outage"], seed)
    st.start()
    time.sleep(3.0)
    for _ in range(10):
        set_mode(c, OFFBOARD)
        if wait(lambda: pilot.s["mode"] == OFFBOARD, 2.5):
            break
    res["offboard"] = pilot.s["mode"] == OFFBOARD
    res["armed"] = res["offboard"] and arm(c, pilot)
    if res["armed"]:
        time.sleep(3.0)
        res["height_armed_idle_m"] = round(pilot.height(), 3)
        st.value = (start[0], start[1], start[2] - target)
        t0 = time.time()
        res["reached"] = wait(lambda: pilot.height() >= target - 0.05 or pilot.s["mode"] != OFFBOARD, 15)
        res["reached_after_s"] = round(time.time() - t0, 1)
        t_hold = time.time()
        wait(lambda: pilot.s["mode"] != OFFBOARD, hold_s)
        res["hold_true"] = summary(pilot, t_hold + 2.0)
        res["peak_m"] = round(pilot.max_alt, 2)
        res["mode_at_action"] = pilot.s["mode"]
        t = time.time()
        if action == "kill":
            pilot.ch[8] = HI
            res["height_1s_after_kill_m"] = (time.sleep(1.0), round(pilot.height(), 2))[1]
            wait(lambda: not pilot.s["armed"], 8)
            res["kill_disarm_s"] = round(time.time() - t, 2)
        elif action == "altitude":
            pilot.ch[7] = LO
            ok = wait(lambda: pilot.s["mode"] == ALTCTL, 4)
            res["altitude_latency_s"] = round(time.time() - t, 2) if ok else None
            st.on = False
            time.sleep(4.0)
            res["height_after_takeover_m"] = round(pilot.height(), 2)
            pilot.ch[3] = 1150
            wait(lambda: pilot.s["landed"] == 1 or not pilot.s["armed"], 20)
            time.sleep(1.0)
            pilot.ch[8] = HI
            wait(lambda: not pilot.s["armed"], 8)
        else:
            st.on = False
            c.mav.command_long_send(c.target_system, 1, 21, 0, 0, 0, 0, 0, 0, 0, 0)
            res["landed_and_disarmed"] = wait(lambda: not pilot.s["armed"], 25)
        res["modes"] = [(round(t_ - t_hold, 1), m, a) for t_, m, a in pilot.modes]
    st.on = False
    res.update(sent=st.sent, dropped=st.dropped, max_tilt=round(pilot.max_tilt, 1),
               true_max_drift_m=round(pilot.max_drift, 2), disarmed=not pilot.s["armed"],
               texts=[t for _, t in pilot.texts if "logger" not in t][-8:])
    if pilot.s["armed"]:
        pilot.force_disarm()
        time.sleep(2.0)
    pilot.close()
    c.close()
    return res


CASES = {"joystick_first": lambda: joystick("joystick"),
         "rc_first": lambda: joystick("rc"),
         "vz_clean_kill": lambda: vz(CLEAN, "kill"),
         "vz_elrs_kill": lambda: vz(ELRS, "kill"),
         "vz_elrs_altitude": lambda: vz(ELRS, "altitude"),
         "vz_elrs_land": lambda: vz(ELRS, "land"),
         "vz_gps_clean_kill": lambda: vz(CLEAN, "kill", gps=True),
         "vz_gps_elrs_kill": lambda: vz(ELRS, "kill", gps=True),
         "vz_gps_elrs_altitude": lambda: vz(ELRS, "altitude", gps=True),
         "vz_gps_elrs_land": lambda: vz(ELRS, "land", gps=True),
         "pos_gps_elrs_kill": lambda: position(ELRS, "kill"),
         "pos_gps_elrs_altitude": lambda: position(ELRS, "altitude"),
         "pos_gps_elrs_land": lambda: position(ELRS, "land"),
         "pos_gps_elrs_land2": lambda: position(ELRS, "land", seed=11)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", default=",".join(CASES))
    ap.add_argument("--out", default="/tmp/elrs_height_probe.json")
    a = ap.parse_args()
    out = {}
    for name in a.only.split(","):
        T.stop_sim()
        try:
            out[name] = CASES[name]()
        except Exception as exc:              # one broken case must not end the probe
            out[name] = dict(error=f"{type(exc).__name__}: {exc}")
        print(name, json.dumps(out[name], default=str), flush=True)
        Path(a.out).write_text(json.dumps(out, indent=1, default=str))
    T.stop_sim()
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
