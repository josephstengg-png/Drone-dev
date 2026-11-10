#!/usr/bin/env python3
"""Automated SITL test campaign on the ARTEX model: tests 2 and 3, plus mission and refusal checks.

    python3 sim/sim_tests.py --out /tmp/sim_results.json [--only stage_matrix|faults|mission|refusals]

Runs ON the Jetson that hosts PX4 SITL + Gazebo (jetson-2). SIMULATOR ONLY: refuses any vehicle
without the SIM_GZ_EN parameter. Restarts the simulator itself when a test leaves the aircraft
crashed or tipped, so every case starts from a clean aircraft on the ground.

The pilot's handset does not exist in SITL; it is emulated over MAVLink: MANUAL_CONTROL at 20 Hz
with centred sticks (throttle mid = hold height) stands in for a pilot, a mode change to Altitude
stands in for channel 7 low, and a forced disarm (param2 21196) stands in for the kill switch
(both cut the motors at once). Faults use PX4's own failure injection (MAV_CMD_INJECT_FAILURE).

Our own safety loop is evaluated in SHADOW: safety_rules.Watchdog is fed the same telemetry the
agent's guard would see and its decision is recorded, but it does not act. PX4's failsafes act.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path.home() / "FlightPlanner" / "tools"))
import safety_rules as sr  # noqa: E402

PX4_DIR = Path.home() / "PX4-Autopilot"
GZB_DIR = PX4_DIR / "build/px4_sitl_default/src/modules/simulation/gz_bridge"
LINK = "udpin:0.0.0.0:14540"
SITL_SYSID = 1
MODE = {"MANUAL": (1, 0), "ALTCTL": (2, 0), "POSCTL": (3, 0), "LOITER": (4, 3), "MISSION": (4, 4),
        "RTL": (4, 5), "LAND": (4, 6), "OFFBOARD": (6, 0), "TAKEOFF": (4, 2)}
NAMES = {v: k for k, v in MODE.items()}
# The real ARTEX safety settings (indoor profile, 19 Oct) plus what SITL needs to emulate a pilot.
ARTEX_PARAMS = {"NAV_DLL_ACT": 3, "NAV_RCL_ACT": 3, "GF_ACTION": 5, "GF_MAX_VER_DIST": 3.0,
                "COM_OBL_RC_ACT": 0, "COM_OF_LOSS_T": 1.0, "COM_DL_LOSS_T": 5,
                "BAT_LOW_THR": 0.20, "BAT_CRIT_THR": 0.15, "BAT_EMERGEN_THR": 0.10,
                "COM_LOW_BAT_ACT": 3, "EKF2_GPS_CHECK": 1023, "SYS_FAILURE_EN": 1,
                "COM_RC_IN_MODE": 3, "COM_RC_LOSS_T": 0.5, "COM_DISARM_PRFLT": 10.0,
                "MIS_TAKEOFF_ALT": 2.5}
ALT = 2.5   # test altitude, under the 3 m fence


# ======================================================================================
# simulator lifecycle
# ======================================================================================

def stop_sim():
    for f in ("/tmp/sitl_px4.pid", "/tmp/gzgui.pid"):
        try:
            os.kill(int(Path(f).read_text()), signal.SIGTERM)
        except Exception:
            pass
    time.sleep(3)
    # 20 Oct: a Gazebo server ignored SIGTERM and survived 40 min; later PX4 instances attached to
    # its world (and its crashed aircraft). TERM, then KILL whatever is left, and check.
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pat in ("^gz sim", "bin/px4 -d"):
            for pid in subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True).stdout.split():
                try:
                    os.kill(int(pid), sig)
                except Exception:
                    pass
        time.sleep(3)
    left = subprocess.run(["pgrep", "-f", "^gz sim|bin/px4 -d"], capture_output=True, text=True).stdout.split()
    if left:
        raise RuntimeError(f"simulator processes survived SIGKILL: {left}")


def start_sim(gui=True, wipe=True, params=None, ready="Ready for takeoff"):
    """params: {NAME: value} applied at boot (PX4_PARAM_* env, rcS) -- e.g. GPS off from power-up."""
    env = dict(os.environ, PX4_SYS_AUTOSTART="4099", PX4_SIM_MODEL="gz_artex",
               PX4_GZ_WORLD="default", GZ_IP="127.0.0.1", HEADLESS="1")
    env.update({f"PX4_PARAM_{k}": str(v) for k, v in (params or {}).items()})
    # Start every case from the airframe's own defaults: parameters persist in SITL, and on 20 Oct
    # an interrupted refusal test left EKF2_GPS_CTRL at 0, which silently broke every GPS test after it.
    for f in (PX4_DIR / "build/px4_sitl_default/rootfs").glob("parameters*.bson") if wipe else ():
        f.unlink()
    log = open(GZB_DIR / "sitl.log", "w")
    p = subprocess.Popen(["../../../../bin/px4", "-d"], cwd=GZB_DIR, env=env,
                         stdin=subprocess.DEVNULL, stdout=log, stderr=log)
    Path("/tmp/sitl_px4.pid").write_text(str(p.pid))
    for _ in range(90):
        time.sleep(1)
        if ready in (GZB_DIR / "sitl.log").read_text(errors="ignore"):
            break
    if gui:
        xa = f"/run/user/{os.getuid()}/gdm/Xauthority"
        if os.path.exists(xa):
            g = subprocess.Popen(["gz", "sim", "-g"], env=dict(os.environ, DISPLAY=":1", XAUTHORITY=xa),
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            Path("/tmp/gzgui.pid").write_text(str(g.pid))
    time.sleep(5)


# ======================================================================================
# vehicle interface
# ======================================================================================

class Sim:
    def __init__(self):
        self.lock = threading.Lock()
        self.c = mavutil.mavlink_connection(LINK, source_system=255, source_component=190)
        self.c.wait_heartbeat(timeout=30)
        # This harness sends MANUAL_CONTROL and writes parameters. On a real aircraft with
        # COM_RC_IN_MODE 3 that could lock out the handset until reboot (3 Nov 2026).
        if self.c.target_system != SITL_SYSID:
            raise RuntimeError(f"link answered as sysid {self.c.target_system}, not the simulator")
        self.s = dict(armed=False, mode=None, x=0.0, y=0.0, z=0.0, vz=0.0, roll=0.0, pitch=0.0,
                      lat=None, lon=None, landed=None, bat=None, t_hb=time.time())
        self.texts = []
        # pump() is the only reader: it files acks and parameter values here, so a command's
        # reply that arrives while pump() is running is kept (20 Oct: they were being discarded).
        self.acks, self.pvals = {}, {}
        self.z0 = 0.0   # local z of the ground (about -0.4 m in this world); heights are above it
        self.hb_on, self.rc_on = True, False
        self.mc = [0, 0, 500, 0]          # pilot sticks: x pitch, y roll, z throttle (500 = hold), r yaw
        self.on_msg = None                # callback(msg) for every message from the vehicle
        self.wd = sr.Watchdog(now=time.time())
        self.wd_max = (sr.NONE, [])
        self._stop = False
        threading.Thread(target=self._beat, daemon=True).start()
        if self.param("SIM_GZ_EN") is None:
            raise SystemExit("REFUSED: no SIM_GZ_EN -- not the simulator")
        # The offboard-API link does not stream everything by default (GLOBAL_POSITION_INT was
        # missing, 20 Oct): ask for what the tests read.
        for msg_id, us in ((33, 200000), (32, 50000), (30, 50000), (245, 500000), (1, 1000000)):
            self.cmd(511, msg_id, us, wait=1.0)

    # -- background senders -------------------------------------------------------------
    def _beat(self):
        n = 0
        while not self._stop:
            with self.lock:
                if self.rc_on:
                    self.c.mav.manual_control_send(self.c.target_system, *self.mc, 0)
                if self.hb_on and n % 20 == 0:
                    self.c.mav.heartbeat_send(6, 8, 0, 0, 0)
            n += 1
            time.sleep(0.05)

    # -- receive ---------------------------------------------------------------------------
    def pump(self, sec=0.0):
        end = time.time() + sec
        while True:
            with self.lock:
                m = self.c.recv_match(blocking=False)
            if m is None:
                if time.time() >= end:
                    return
                time.sleep(0.01)
                continue
            if m.get_srcSystem() != self.c.target_system:
                continue
            if self.on_msg is not None:
                self.on_msg(m)
            k, now = m.get_type(), time.time()
            if k == "HEARTBEAT" and m.type != 6:
                self.s["armed"] = bool(m.base_mode & 128)
                self.s["mode"] = NAMES.get(((m.custom_mode >> 16) & 0xFF, (m.custom_mode >> 24) & 0xFF),
                                           f"{(m.custom_mode >> 16) & 0xFF}.{(m.custom_mode >> 24) & 0xFF}")
                self.s["t_hb"] = now
                self.wd.heartbeat(now)
            elif k == "COMMAND_ACK":
                self.acks[m.command] = m.result
            elif k == "PARAM_VALUE":
                v = m.param_value
                if m.param_type <= 6:   # integer types travel bit-cast inside the float
                    import struct
                    v = struct.unpack("<i", struct.pack("<f", v))[0]
                self.pvals[m.param_id] = v
            elif k == "LOCAL_POSITION_NED":
                self.s.update(x=m.x, y=m.y, z=-m.z - self.z0, vz=-m.vz)
            elif k == "ATTITUDE":
                self.s.update(roll=math.degrees(m.roll), pitch=math.degrees(m.pitch))
                self.wd.attitude(self.s["roll"], self.s["pitch"], now)
            elif k == "GLOBAL_POSITION_INT":
                self.s.update(lat=m.lat / 1e7, lon=m.lon / 1e7)
            elif k == "EXTENDED_SYS_STATE":
                self.s["landed"] = m.landed_state       # 1 on ground, 2 in air, 3 takeoff, 4 landing
            elif k == "SYS_STATUS" and m.battery_remaining >= 0:
                self.s["bat"] = m.battery_remaining
                self.wd.battery(m.battery_remaining, now)
            elif k == "STATUSTEXT":
                t = m.text if isinstance(m.text, str) else m.text.decode(errors="replace")
                self.texts.append((round(now, 2), m.severity, t.strip()))
                self.wd.statustext(m.severity, t, now)
            v = self.wd.evaluate(now)
            if sr.RUNG[v.action] > sr.RUNG[self.wd_max[0]]:
                self.wd_max = (v.action, v.reasons[-2:])

    # -- commands -----------------------------------------------------------------------------
    def cmd(self, command, *p, wait=3.0):
        p = list(p) + [0] * (7 - len(p))
        self.acks.pop(command, None)
        with self.lock:
            self.c.mav.command_long_send(self.c.target_system, 1, command, 0, *p)
        end = time.time() + wait
        while time.time() < end:
            self.pump(0.05)
            if command in self.acks:
                return self.acks.pop(command)
        return None

    def param(self, name, value=None):
        self.pvals.pop(name, None)
        with self.lock:
            if value is not None:
                ptype = 6 if isinstance(value, int) else 9
                payload = value
                if ptype == 6:
                    import struct
                    payload = struct.unpack("<f", struct.pack("<i", int(value)))[0]
                self.c.mav.param_set_send(self.c.target_system, 1, name.encode(), payload, ptype)
            else:
                self.c.mav.param_request_read_send(self.c.target_system, 1, name.encode(), -1)
        end = time.time() + 3
        while time.time() < end:
            self.pump(0.05)
            if name in self.pvals:
                return self.pvals.pop(name)
        return None

    def mode(self, name):
        main, sub = MODE[name]
        return self.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, main, sub)

    def arm(self):
        return self.cmd(400, 1)

    def disarm(self, force=False):
        return self.cmd(400, 0, 21196 if force else 0)

    def takeoff(self, alt=ALT):
        # param7 is an AMSL altitude; NaN makes PX4 climb to MIS_TAKEOFF_ALT (= ALT) above home
        return self.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float("nan"),
                        float("nan"), float("nan"), float("nan"))

    def land(self):
        return self.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, float("nan"), float("nan"), float("nan"), 0)

    def inject(self, unit, ftype, instance=0):
        return self.cmd(420, unit, ftype, instance)

    def wait(self, pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            if pred(self.s):
                return True
        return False

    def upload_square(self, side=8.0, alt=ALT, home=None):
        lat, lon = home or (self.s["lat"], self.s["lon"])
        dn = lambda m: m / 111320.0                                         # noqa: E731
        de = lambda m: m / (111320.0 * math.cos(math.radians(lat)))       # noqa: E731
        pts = [(dn(side), 0), (dn(side), de(side)), (0, de(side)), (0, 0)]
        items = [(22, lat, lon)] + [(16, lat + a, lon + b) for a, b in pts] + [(21, lat, lon)]
        with self.lock:
            self.c.mav.mission_count_send(self.c.target_system, 1, len(items), 0)
        end = time.time() + 15
        while time.time() < end:
            with self.lock:
                m = self.c.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                      blocking=True, timeout=0.5)
            if m is None:
                continue
            if m.get_type() == "MISSION_ACK":
                return m.type, [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
            cmdid, la, lo = items[m.seq]
            with self.lock:
                self.c.mav.mission_item_int_send(self.c.target_system, 1, m.seq, 3, cmdid,
                                                 1 if m.seq == 0 else 0, 1, 0, 0.5, 0, float("nan"),
                                                 int(la * 1e7), int(lo * 1e7), alt, 0)
        return None, None

    def snapshot(self):
        return {k: (round(v, 2) if isinstance(v, float) else v) for k, v in self.s.items()
                if k in ("armed", "mode", "z", "vz", "roll", "pitch", "landed", "bat")}

    def close(self):
        self._stop = True
        self.c.close()


def clean(sim) -> bool:
    """On the ground, disarmed, upright?"""
    sim.pump(1.0)
    return (not sim.s["armed"]) and sim.s["z"] < 0.4 and abs(sim.s["roll"]) < 20 and abs(sim.s["pitch"]) < 20


def fresh(sim=None, gui=True, attempts=3):
    """Restart the simulator and PROVE it answers before any test uses it.

    20 Oct: a harness that connected to a half-started PX4 recorded 'no reply' for every case.
    Now: position must arrive and a harmless command must be acknowledged, or it restarts again.
    """
    for attempt in range(1, attempts + 1):
        if sim is not None:
            sim.close(); sim = None
        stop_sim()
        start_sim(gui)
        try:
            s = Sim()
        except Exception as exc:
            print(f"[fresh] attempt {attempt}: no vehicle ({exc!r})", flush=True); continue
        for k, v in ARTEX_PARAMS.items():
            s.param(k, v)
        ok_pos = s.wait(lambda st: st["lat"] is not None and st["mode"] is not None, 30)
        s.pump(2.0); s.z0 = s.s["z"]; s.s["z"] = 0.0
        ack = s.mode("POSCTL")
        if ok_pos and ack == 0:
            print(f"[fresh] simulator ready (attempt {attempt})", flush=True)
            return s
        print(f"[fresh] attempt {attempt}: position={ok_pos} mode ack={ack}; restarting", flush=True)
        sim = s
    raise RuntimeError("simulator never became ready")


def observe(sim, seconds):
    """Watch after an action: record falls, mode changes, disarm time."""
    t0, modes, min_vz, max_tilt, t_disarm = time.time(), [], 0.0, 0.0, None
    last = sim.s["mode"]
    while time.time() - t0 < seconds:
        sim.pump(0.05)
        if sim.s["mode"] != last:
            modes.append((round(time.time() - t0, 1), sim.s["mode"])); last = sim.s["mode"]
        min_vz = min(min_vz, sim.s["vz"])
        max_tilt = max(max_tilt, abs(sim.s["roll"]), abs(sim.s["pitch"]))
        if t_disarm is None and not sim.s["armed"]:
            t_disarm = round(time.time() - t0, 1)
        if t_disarm is not None and sim.s["z"] < 0.3 and time.time() - t0 > t_disarm + 1:
            break
    return dict(mode_changes=modes, fastest_descent_ms=round(min_vz, 2), max_tilt_deg=round(max_tilt, 1),
                disarmed_after_s=t_disarm, final=sim.snapshot())


# ======================================================================================
# test 2: every action at every stage
# ======================================================================================

def arm_when_ready(sim, timeout=30):
    """Arm, retrying while the estimator settles: right after boot PX4 refuses with
    'ekf2 missing data' for several seconds (20 Oct, every case once restarts became per-case)."""
    end = time.time() + timeout
    while time.time() < end:
        if sim.arm() == 0 and sim.wait(lambda s: s["armed"], 3):
            return True
        sim.pump(2.0)
    return False


def to_stage(sim, stage):
    """Bring the aircraft to a stage. Returns True when it is there."""
    sim.rc_on = True
    if stage == "ground_armed":
        sim.mode("POSCTL"); return arm_when_ready(sim)
    sim.mode("POSCTL")
    if not arm_when_ready(sim):
        return False
    if stage == "climbing":
        sim.takeoff(); return sim.wait(lambda s: s["z"] > 1.0, 20)
    if stage == "mid_mission":
        sim.disarm(); sim.wait(lambda s: not s["armed"], 5)
        res, _ = sim.upload_square()
        if res != 0:
            return False
        sim.mode("MISSION")
        if not arm_when_ready(sim):
            return False
        return sim.wait(lambda s: s["armed"] and s["z"] > 2.0 and math.hypot(s["x"], s["y"]) > 3.0, 40)
    sim.takeoff()
    ok = sim.wait(lambda s: s["z"] > ALT - 0.3, 25) and sim.wait(lambda s: s["mode"] == "LOITER", 10)
    sim.pump(3.0)
    if stage == "hover":
        return ok
    if stage == "landing":
        sim.land(); return sim.wait(lambda s: s["mode"] == "LAND" and s["z"] < 1.5 and s["vz"] < -0.2, 25)
    return False


def do_action(sim, action):
    if action == "land":
        return sim.land()
    if action == "disarm":
        return sim.disarm(force=False)
    if action == "kill":
        return sim.disarm(force=True)
    if action == "pilot_takeover":
        return sim.mode("ALTCTL")


def stage_matrix(results, sim):
    out = []
    for stage in ("ground_armed", "climbing", "hover", "mid_mission", "landing"):
        for action in ("land", "disarm", "kill", "pilot_takeover"):
            sim = fresh(sim)    # every case from a new simulator: a kill leaves the airframe flipped
            sim.texts.clear()
            reached = to_stage(sim, stage)
            before = sim.snapshot()
            if not reached:
                print(f"[matrix] could not reach {stage}: state {before} texts {[t for _, _, t in sim.texts][-3:]}", flush=True)
            ack = do_action(sim, action) if reached else None
            obs = observe(sim, 25 if stage != "ground_armed" else 12)
            if sim.s["armed"]:                           # leave nothing armed: land, then force
                sim.land(); sim.wait(lambda s: not s["armed"], 25)
                if sim.s["armed"]:
                    sim.disarm(force=True); sim.wait(lambda s: not s["armed"], 5)
            r = dict(stage=stage, action=action, reached_stage=reached, before=before,
                     ack={0: "accepted", 1: "temporarily rejected", 2: "denied", 4: "failed",
                          None: "no reply"}.get(ack, str(ack)), **obs,
                     px4_text=[t for _, _, t in sim.texts][-5:])
            print(f"[matrix] {stage:12s} {action:14s} ack={r['ack']:10s} disarmed_after={obs['disarmed_after_s']} "
                  f"modes={obs['mode_changes']} descent={obs['fastest_descent_ms']} tilt={obs['max_tilt_deg']}", flush=True)
            out.append(r)
    results["stage_matrix"] = out
    return sim


# ======================================================================================
# test 3: faults, with our safety loop in shadow
# ======================================================================================

# MAV_CMD_INJECT_FAILURE does nothing to the GPS, magnetometer or barometer in a Gazebo build
# (only simulator_mavlink and the battery simulator act on it; checked in the v1.16 source, 20 Oct),
# so sensor loss is modelled from the estimator's side: its fusion of that sensor is switched off.
# A motor "failure" only forges an ESC report there; Gazebo keeps spinning the motor.
FAULTS = [  # name, how, spec, where
    ("gps_loss", "param", ("EKF2_GPS_CTRL", 0), "mid_mission"),
    ("mag_failure", "param", ("EKF2_MAG_TYPE", 5), "hover"),
    ("baro_failure", "param", ("EKF2_BARO_CTRL", 0), "hover"),
    ("gcs_link_loss", "heartbeats", None, "hover"),
    ("rc_loss", "manual", None, "hover_altctl"),
    ("low_battery", "battery", None, "hover"),
]


def faults(results, sim):
    out = []
    for name, how, spec, where in FAULTS:
        sim = fresh(sim)
        sim.texts.clear(); sim.wd = sr.Watchdog(now=time.time()); sim.wd_max = (sr.NONE, [])
        stage = "hover" if where == "hover_altctl" else where
        reached = to_stage(sim, stage)
        if where == "hover_altctl" and reached:
            sim.mode("ALTCTL"); sim.pump(2.0)
        before = sim.snapshot(); ack = None
        if reached:
            if how == "param":
                ack = sim.param(*spec)
            elif how == "heartbeats":
                sim.hb_on = False
            elif how == "manual":
                sim.rc_on = False
            elif how == "battery":
                sim.param("SIM_BAT_MIN_PCT", 5.0); sim.param("SIM_BAT_DRAIN", 15.0)
        obs = observe(sim, 45)
        sim.hb_on = True; sim.rc_on = True
        if sim.s["armed"]:
            sim.land(); sim.wait(lambda s: not s["armed"], 25)
            if sim.s["armed"]:
                sim.disarm(force=True)
        r = dict(fault=name, during=where, reached_stage=reached, before=before,
                 inject_ack=ack, **obs, px4_text=[t for _, _, t in sim.texts][-6:],
                 our_guard_shadow=dict(max_rung=sim.wd_max[0], reasons=sim.wd_max[1]))
        print(f"[fault] {name:14s} during {where:12s} ack={ack} modes={obs['mode_changes']} "
              f"disarmed_after={obs['disarmed_after_s']} guard={sim.wd_max[0]}", flush=True)
        out.append(r)
    results["faults"] = out
    return sim


# ======================================================================================
# test 1b and refusals
# ======================================================================================

def mission(results, sim):
    sim = fresh(sim)
    sim.rc_on = True
    res, legs = sim.upload_square()
    sim.mode("MISSION"); sim.arm()
    track, t0 = [], time.time()
    while time.time() - t0 < 120:
        sim.pump(0.1)
        track.append((sim.s["x"], sim.s["y"], sim.s["z"]))
        if time.time() - t0 > 10 and not sim.s["armed"]:
            break
    def xt(p):  # distance from the nearest leg of the square
        best = 1e9
        for (a, b), (c, d) in zip(legs, legs[1:]):
            vx, vy, wx, wy = c - a, d - b, p[0] - a, p[1] - b
            t = max(0, min(1, (wx * vx + wy * vy) / (vx * vx + vy * vy)))
            best = min(best, math.hypot(wx - t * vx, wy - t * vy))
        return best
    air = [p for p in track if p[2] > 1.5]
    xs = [xt(p) for p in air]
    results["mission"] = dict(upload_ack=res, duration_s=round(time.time() - t0, 1), samples=len(air),
                              cross_track_max_m=round(max(xs), 2) if xs else None,
                              cross_track_mean_m=round(sum(xs) / len(xs), 2) if xs else None,
                              alt_range_m=(round(min(p[2] for p in air), 2), round(max(p[2] for p in air), 2)) if air else None,
                              ended_disarmed=not sim.s["armed"], px4_text=[t for _, _, t in sim.texts][-6:])
    print("[mission]", results["mission"], flush=True)
    return sim


def refusals(results, sim):
    """With GNSS fusion off (indoor), Position, Mission and Hold must be refused."""
    sim = fresh(sim)
    home = (sim.s["lat"], sim.s["lon"])
    sim.param("EKF2_GPS_CTRL", 0)
    sim.cmd(245, 1)          # save now: PX4 writes parameters lazily, and stop_sim() would beat it
    sim.pump(3.0)
    # Boot WITHOUT GNSS fusion, as the real aircraft does indoors. Switching it off mid-session
    # (first version) leaves the estimator dead-reckoning a position it already had.
    sim.close(); stop_sim(); start_sim(wipe=False)
    sim = Sim()
    out = {"EKF2_GPS_CTRL_after_reboot": sim.param("EKF2_GPS_CTRL")}
    if out["EKF2_GPS_CTRL_after_reboot"] != 0:
        raise RuntimeError(f"GPS fusion still on after reboot: {out}")
    sim.rc_on = True
    sim.wait(lambda s: False, 20)
    try:
        _refusal_checks(sim, out, home)
    finally:
        sim.param("EKF2_GPS_CTRL", 7)   # the next fresh() wipes parameters anyway
    results["refusals_without_gps"] = out
    print("[refusals]", out, flush=True)
    return sim


def _refusal_checks(sim, out, home):
    """For each mode: does PX4 accept the switch, which mode does it then report, and will it arm?"""
    word = {0: "ACCEPTED", 1: "temporarily rejected", 2: "denied", 4: "failed", None: "no reply"}
    out["global_position_published"] = sim.s["lat"] is not None
    res, _ = sim.upload_square(home=home)
    out["mission_upload"] = word.get(res, f"rejected ({res})")
    for m in ("ALTCTL", "POSCTL", "LOITER", "MISSION"):
        sim.texts.clear()
        ack = sim.mode(m); sim.pump(1.5)
        reported = sim.s["mode"]
        arm = sim.arm(); sim.pump(1.5)
        armed = sim.s["armed"]
        if armed:
            sim.disarm(force=True); sim.wait(lambda s: not s["armed"], 5)
        out[m] = dict(switch=word.get(ack, str(ack)), mode_reported=reported, arm=word.get(arm, str(arm)),
                      armed=armed, px4_text=[t for _, _, t in sim.texts if "logger" not in t][-2:])
        sim.pump(2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="/tmp/sim_results.json")
    ap.add_argument("--only", choices=["stage_matrix", "faults", "mission", "refusals"])
    args = ap.parse_args()
    results = dict(started=time.strftime("%Y-%m-%d %H:%M:%S"), params=ARTEX_PARAMS)
    sim = fresh(None)
    steps = [("refusals", refusals), ("mission", mission), ("stage_matrix", stage_matrix), ("faults", faults)]
    for name, fn in steps:
        if args.only and name != args.only:
            continue
        try:
            sim = fn(results, sim)
        except Exception as exc:          # record and carry on with the next test
            results[name + "_error"] = repr(exc)
            print(f"[{name}] ERROR {exc!r}", flush=True)
            sim = fresh(sim)
        Path(args.out).write_text(json.dumps(results, indent=1, default=str))
    results["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    Path(args.out).write_text(json.dumps(results, indent=1, default=str))
    sim.close()
    print("DONE", args.out, flush=True)


if __name__ == "__main__":
    main()
