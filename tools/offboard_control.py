#!/usr/bin/env python3
"""Indoor offboard hover: hold height and stay level, with nothing but IMU and height.

    python3 tools/offboard_control.py --link udpin://0.0.0.0:14540 --height 1.0 --hold-s 20

SIMULATOR ONLY. The runner refuses any vehicle that does not have the parameter
SIM_GZ_EN, which exists in PX4 SITL with Gazebo and does not exist on the ARTEX firmware.
Real aircraft come after the simulation and HITL rounds (see docs/history.md, 19 Oct 2026).

WHAT IT CAN AND CANNOT DO. Indoors there is no horizontal position, so PX4 only accepts
offboard commands as attitude plus thrust. This controller therefore:
  - holds HEIGHT by closing a height -> climb-rate -> thrust loop itself, and
  - holds LEVEL attitude (yaw held at its value when the run started).
It does NOT hold position. The aircraft will drift sideways; see history.md for the numbers.

WHY IT IS BUILT LIKE THIS. Everything that decides is in HeightHold, a pure state machine:
no MAVLink, no sockets, no clock of its own. The runner feeds it measurements and sends what
it returns. Two lessons from the 15 Oct ARTEX113 incident are written into it:
  - the integrator is FROZEN until the aircraft is airborne, and a takeoff that does not
    leave the ground in time ABORTS with minimum thrust. On 15 Oct a controller that believed
    the aircraft was flying wound the motors up on a bench.
  - every abort ends in a request for PX4's own LAND, never in silence.

HEIGHT SOURCE. The caller passes height above the takeoff point and climb rate from the EKF
(LOCAL_POSITION_NED, which is still valid vertically with GNSS fusion off). Do not feed the
TFmini directly: it reads nothing useful below its 0.40 m minimum.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

IDLE, TAKEOFF, HOLD, LAND, DONE, ABORT = "idle", "takeoff", "hold", "land", "done", "abort"


@dataclass
class Limits:
    """Every number in one place. Defaults are for the ARTEX as it is configured today."""

    target_height_m: float = 1.0      # the operator's indoor hover height (19 Oct)
    ceiling_m: float = 1.5            # above this the run aborts; the PX4 fence is 3 m
    climb_rate_ms: float = 0.4
    descent_rate_ms: float = 0.3
    final_zone_m: float = 0.3         # below this height setpoint the descent slows to...
    final_descent_rate_ms: float = 0.3   # ...this (the takeover ladder's landing test, 29 Oct)
    kp_height: float = 1.0            # height error (m) -> climb-rate setpoint (m/s)
    kp_vz: float = 0.25               # climb-rate error (m/s) -> thrust
    ki_vz: float = 0.30              # 0.10 took over 20 s to absorb a 0.08 hover-guess error
                                      # in SITL (20 Oct: hold drifted 1.08 -> 1.05 m)
    i_limit: float = 0.25             # integrator may move thrust by at most this much.
                                      # 0.15 left an 8 cm steady error in SITL (20 Oct) when
                                      # the hover guess (0.5) was 0.17 above the truth (0.33)
    hover_thrust: float = 0.50        # MPC_THR_HOVER; the ARTEX has never learned its own
    thrust_min: float = 0.12          # MPC_THR_MIN
    thrust_max: float = 0.80          # headroom kept for PX4's attitude control
    spool_s: float = 2.0              # ramp from min towards hover + margin on the ground
    spool_margin: float = 0.2         # how far above the assumed hover thrust the ground spool
                                      # may go. 0.1 left a heavier aircraft stuck on the floor
                                      # (true hover 0.6 vs assumed 0.5) in the 19 Oct tests.
    lift_vz_ms: float = 0.15          # first sign of lift: climbing this fast, or risen
    lift_rise_m: float = 0.05         # this far above the lowest height seen on the floor.
                                      # The spool freezes there (29 Oct: it ramped on while the
                                      # estimate dipped 0.1 m under prop wash, lifted off at
                                      # 2.5 m/s and hit the 1.5 m ceiling in 5 of 80 SITL runs).
    takeoff_timeout_s: float = 6.0    # must be airborne by then, or abort
    airborne_height_m: float = 0.15
    land_height_m: float = 0.10
    landed_dwell_s: float = 1.0
    height_timeout_s: float = 0.5
    tilt_abort_deg: float = 25.0
    saturation_abort_s: float = 2.0


@dataclass
class Command:
    phase: str
    thrust: float                     # 0..1, what PX4 receives as collective thrust
    q: tuple                          # attitude quaternion (w, x, y, z): level, held yaw
    request: str | None = None        # "land" (PX4 LAND mode) or "disarm"
    reason: str = ""
    height_sp: float | None = None
    vz_sp: float | None = None


def level_quaternion(yaw_rad: float) -> tuple:
    return (math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class HeightHold:
    limits: Limits = field(default_factory=Limits)
    yaw_rad: float = 0.0
    hold_s: float = 10.0
    phase: str = IDLE
    _t_phase: float = 0.0
    _height_sp: float = 0.0
    _i: float = 0.0
    _last_valid: float | None = None
    _sat_since: float | None = None
    _landed_since: float | None = None
    _floor_min: float | None = None
    _lift_thrust: float | None = None
    airborne: bool = False
    reason: str = ""

    # ---- operator events -------------------------------------------------------------

    def start(self, now: float, height: float) -> None:
        """Begin the takeoff from the current height."""
        if self.phase == IDLE:
            self._enter(TAKEOFF, now)
            self._height_sp = height
            self._i = 0.0

    def land(self, now: float) -> None:
        if self.phase in (TAKEOFF, HOLD):
            self._enter(LAND, now)

    # ---- the loop ----------------------------------------------------------------------

    def step(self, now: float, dt: float, height: float | None, height_valid: bool,
             vz: float | None, tilt_deg: float | None) -> Command:
        L = self.limits
        q = level_quaternion(self.yaw_rad)
        if height_valid and height is not None:
            self._last_valid = now

        if self.phase in (IDLE, DONE):
            return Command(self.phase, 0.0, q, request="disarm" if self.phase == DONE else None,
                           reason=self.reason)
        if self.phase == ABORT:
            return Command(ABORT, L.thrust_min, q, request="land", reason=self.reason)

        # ---- abort checks, before any thrust is computed --------------------------------
        if self._last_valid is None or now - self._last_valid > L.height_timeout_s:
            return self._abort(now, "height estimate lost")
        if height is not None and height > L.ceiling_m:
            return self._abort(now, f"above the {L.ceiling_m:.1f} m ceiling ({height:.2f} m)")
        if tilt_deg is not None and abs(tilt_deg) > L.tilt_abort_deg:
            return self._abort(now, f"tilt {tilt_deg:.0f} deg over {L.tilt_abort_deg:.0f}")
        if self.phase == TAKEOFF and not self.airborne and \
                now - self._t_phase > L.takeoff_timeout_s:
            return self._abort(now, "did not leave the ground (props off, or not enough thrust)")

        if height >= L.airborne_height_m:
            self.airborne = True

        # ---- phase logic: move the height setpoint ---------------------------------------
        if self.phase == TAKEOFF and not self.airborne:
            # the setpoint waits on the floor: one that ran ahead is a climb demand at lift-off
            self._height_sp = max(self._height_sp, height)
        elif self.phase == TAKEOFF:
            self._height_sp = min(L.target_height_m, self._height_sp + L.climb_rate_ms * dt)
            if self._height_sp >= L.target_height_m and \
                    abs(height - L.target_height_m) < 0.1:
                self._enter(HOLD, now)
        elif self.phase == HOLD:
            self._height_sp = L.target_height_m
            if now - self._t_phase >= self.hold_s:
                self._enter(LAND, now)
        elif self.phase == LAND:
            rate = L.descent_rate_ms if self._height_sp > L.final_zone_m else L.final_descent_rate_ms
            self._height_sp = max(0.0, self._height_sp - rate * dt)
            if height <= L.land_height_m:
                self._landed_since = self._landed_since or now
                if now - self._landed_since >= L.landed_dwell_s:
                    self._enter(DONE, now)
                    return Command(DONE, 0.0, q, request="disarm", reason="landed")
            else:
                self._landed_since = None

        # ---- height -> climb rate -> thrust ---------------------------------------------
        vz_sp = clamp(L.kp_height * (self._height_sp - height),
                      -L.descent_rate_ms, L.climb_rate_ms)
        err = vz_sp - (vz if vz is not None else 0.0)
        if self.airborne:            # frozen on the ground: no wind-up against the floor
            self._i = clamp(self._i + L.ki_vz * err * dt, -L.i_limit, L.i_limit)
        thrust = L.hover_thrust + L.kp_vz * err + self._i
        if not self.airborne:
            # On the ground the closed loop is not used: the integrator is frozen (no wind-up
            # against the floor) and the proportional term alone cannot lift an aircraft
            # heavier than the hover guess. So ramp thrust steadily from minimum to
            # hover + margin, bounded, until the aircraft leaves the floor; then the loop
            # takes over. With no lift at all it sits at that bound until the timeout aborts.
            # At the first sign of lift the ramp stops where it is: that thrust already beats
            # the weight. A false sign leaves it short of lift-off, and the timeout aborts.
            spool = clamp((now - self._t_phase) / L.spool_s, 0.0, 1.0)
            thrust = L.thrust_min + spool * (L.hover_thrust + L.spool_margin - L.thrust_min)
            self._floor_min = height if self._floor_min is None else min(self._floor_min, height)
            if self._lift_thrust is None and ((vz or 0.0) > L.lift_vz_ms
                                              or height - self._floor_min > L.lift_rise_m):
                self._lift_thrust = thrust
            if self._lift_thrust is not None:
                thrust = self._lift_thrust
        thrust = clamp(thrust, L.thrust_min, L.thrust_max)

        if self.airborne and thrust >= L.thrust_max:
            self._sat_since = self._sat_since or now
            if now - self._sat_since > L.saturation_abort_s:
                return self._abort(now, "thrust saturated at maximum")
        else:
            self._sat_since = None
        return Command(self.phase, thrust, q, height_sp=self._height_sp, vz_sp=vz_sp)

    # ---- internals ---------------------------------------------------------------------

    def _enter(self, phase: str, now: float) -> None:
        self.phase, self._t_phase = phase, now

    def _abort(self, now: float, reason: str) -> Command:
        self.reason = reason
        self._enter(ABORT, now)
        return Command(ABORT, self.limits.thrust_min, level_quaternion(self.yaw_rad),
                       request="land", reason=reason)


# ======================================================================================
# The takeover ladder (29 Oct 2026, docs/takeover-ladder-plan.md)
# ======================================================================================

STAGES = ("prearm", "arm", "takeoff", "hold", "descent", "landing", "disarm")
FLIGHT_STAGES = STAGES[2:]


def stage_profile(stage: str, *, height_m: float, wait_s: float, hold_s: float = 3.0,
                  takeoff_pause_m: float = 0.3, test_descent_rate_ms: float = 0.08,
                  final_descent_rate_ms: float = 0.05) -> dict:
    """How to fly so that the stage under test lasts long enough for a pilot to act in it.

    take-off: climb to a low pause height and stay there for the wait window
    hold:     the whole wait window at full height
    descent:  a slow descent (1 m at 0.08 m/s is about 10 s)
    landing:  normal descent, then a slow final 0.3 m
    disarm:   a normal flight; the window is PX4's own COM_DISARM_LAND after touchdown
    When the window passes with no takeover the controller simply carries on and lands.
    """
    if stage not in FLIGHT_STAGES:
        raise ValueError(f"{stage!r} does not fly")
    p = dict(target_height_m=height_m, hold_s=hold_s, descent_rate_ms=0.3,
             final_descent_rate_ms=0.3)
    if stage == "takeoff":
        p.update(target_height_m=takeoff_pause_m, hold_s=wait_s)
    elif stage == "hold":
        p.update(hold_s=wait_s)
    elif stage == "descent":
        p.update(descent_rate_ms=test_descent_rate_ms, final_descent_rate_ms=test_descent_rate_ms)
    elif stage == "landing":
        p.update(final_descent_rate_ms=final_descent_rate_ms)
    return p


def stage_of(phase: str, height_sp: float | None, final_zone_m: float) -> str:
    """The ladder's name for where the controller is."""
    if phase == TAKEOFF:
        return "takeoff"
    if phase == HOLD:
        return "hold"
    if phase == LAND:
        return "descent" if (height_sp or 0.0) > final_zone_m else "landing"
    if phase == DONE:
        return "landing"
    return phase


@dataclass
class OutageModel:
    """SIMULATOR ONLY: drop setpoints the way the ELRS uplink does, in bursts.

    Measured 19 Oct 2026: gaps over 1 s several times per run. Outages start at random
    (exponential, mean every_s apart) and last a uniform min_s..max_s.
    """
    every_s: float
    min_s: float
    max_s: float
    rng: object
    _start: float | None = None
    _end: float = 0.0

    def dropped(self, now: float) -> bool:
        if self._start is None:
            self._start = now + self.rng.expovariate(1.0 / self.every_s)
        if now >= self._start:
            self._end = self._start + self.rng.uniform(self.min_s, self.max_s)
            self._start = self._end + self.rng.expovariate(1.0 / self.every_s)
        return now < self._end


# ======================================================================================
# Runner: simulator only
# ======================================================================================

def main() -> int:  # pragma: no cover -- exercised against SITL, not in unit tests
    import argparse
    import csv
    import sys
    import time
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import drone_agent as agent
    from pymavlink import mavutil

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udpin://0.0.0.0:14540")
    ap.add_argument("--height", type=float, default=1.0)
    ap.add_argument("--hold-s", type=float, default=20.0)
    ap.add_argument("--rate-hz", type=float, default=50.0)
    ap.add_argument("--csv", default="offboard_run.csv")
    ap.add_argument("--hover", type=float, default=None,
                    help="hover thrust guess; default: the vehicle's own MPC_THR_HOVER. A fixed 0.5 "
                         "left the SITL x500 (hover above 0.7) unable to lift on 20 Oct. For the "
                         "real ARTEX, PX4's own estimate in past flights was 0.41-0.44.")
    args = ap.parse_args()

    conn = agent.connect(args.link, None)
    if agent.read_param(conn, "SIM_GZ_EN") is None:
        print("[offboard] REFUSED: no SIM_GZ_EN parameter -- this is not the simulator. "
              "This controller does not fly real aircraft yet.", flush=True)
        return 2
    if agent.vehicle_armed(conn) is not False:
        print("[offboard] REFUSED: vehicle armed or silent", flush=True)
        return 2

    st = dict(z=None, vz=None, z_valid=False, t_lp=None, yaw=0.0, tilt=0.0, armed=False,
              mode=None)

    def pump(timeout=0.0):
        m = conn.recv_match(blocking=timeout > 0, timeout=timeout or None)
        while m is not None:
            k = m.get_type()
            if k == "LOCAL_POSITION_NED":
                st.update(z=m.z, vz=m.vz, t_lp=time.time(), z_valid=True)
            elif k == "ATTITUDE":
                st["yaw"] = m.yaw
                st["tilt"] = math.degrees(math.acos(clamp(math.cos(m.roll) * math.cos(m.pitch),
                                                          -1.0, 1.0)))
            elif k == "HEARTBEAT" and m.get_srcSystem() == conn.target_system \
                    and m.type != mavutil.mavlink.MAV_TYPE_GCS:
                st["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                st["mode"] = agent.vehicle_main_mode(m)
            elif k == "STATUSTEXT":
                print(f"[vehicle] {m.text}", flush=True)
            m = conn.recv_match(blocking=False)

    def send(cmd: Command):
        conn.mav.set_attitude_target_send(int(time.monotonic() * 1000) & 0xFFFFFFFF,
                                          conn.target_system, 1, 0b00000111, list(cmd.q),
                                          0.0, 0.0, 0.0, cmd.thrust)

    t_end = time.time() + 3
    while time.time() < t_end:
        pump(0.1)
    if st["z"] is None:
        print("[offboard] REFUSED: no LOCAL_POSITION_NED (no height estimate)", flush=True)
        return 2
    z0 = st["z"]
    hover = args.hover if args.hover is not None else agent.read_param(conn, "MPC_THR_HOVER")
    hover = float(hover) if hover else Limits().hover_thrust
    print(f"[offboard] hover thrust guess {hover:.2f}", flush=True)
    ctl = HeightHold(Limits(target_height_m=args.height, hover_thrust=hover),
                     yaw_rad=st["yaw"], hold_s=args.hold_s)
    idle = Command(IDLE, 0.0, level_quaternion(st["yaw"]))
    for _ in range(int(args.rate_hz)):            # PX4 needs setpoints before OFFBOARD
        send(idle); pump(); time.sleep(1.0 / args.rate_hz)
    conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                               0, 1, agent.OFFBOARD_MAIN_MODE, 0, 0, 0, 0, 0)
    conn.mav.command_long_send(conn.target_system, 1,
                               mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    t_arm = time.time()
    while not st["armed"] and time.time() - t_arm < 10:
        send(idle); pump(); time.sleep(1.0 / args.rate_hz)
    if not st["armed"]:
        print("[offboard] vehicle did not arm", flush=True)
        return 2

    rows, t_prev = [], time.time()
    ctl.start(t_prev, 0.0)
    while True:
        pump()
        now = time.time(); dt = now - t_prev; t_prev = now
        if st["mode"] not in (None, agent.OFFBOARD_MAIN_MODE) and ctl.phase not in (DONE, ABORT):
            print(f"[offboard] left OFFBOARD (mode {st['mode']}): the pilot or PX4 has it. "
                  f"Standing down.", flush=True)
            break
        h = None if st["z"] is None else -(st["z"] - z0)
        valid = st["t_lp"] is not None and now - st["t_lp"] < 0.3
        cmd = ctl.step(now, dt, h, valid, None if st["vz"] is None else -st["vz"], st["tilt"])
        rows.append((round(now - t_arm, 3), cmd.phase, h, cmd.height_sp,
                     None if st["vz"] is None else -st["vz"], cmd.vz_sp, round(cmd.thrust, 3)))
        if cmd.request == "land":
            print(f"[offboard] ABORT: {cmd.reason} -> PX4 LAND", flush=True)
            conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_NAV_LAND,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            break
        if cmd.request == "disarm":
            # Hand the touchdown to PX4's own LAND: it confirms ground contact with its land
            # detector and disarms itself (COM_DISARM_LAND). Disarming ourselves at our own
            # 'landed' was refused ("Disarming denied: not landed") in SITL on 20 Oct, and
            # stopping the stream raised an offboard-loss failsafe.
            print("[offboard] near the ground; handing touchdown to PX4 LAND", flush=True)
            conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_NAV_LAND,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            t_land = time.time()
            while st["armed"] and time.time() - t_land < 15:
                pump(); time.sleep(0.05)
            if st["armed"]:
                print("[offboard] PX4 has not disarmed after 15 s; sending disarm", flush=True)
                for _ in range(10):
                    conn.mav.command_long_send(conn.target_system, 1,
                                               mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                               0, 0, 0, 0, 0, 0, 0, 0)
                    for _ in range(10):
                        pump(); time.sleep(0.05)
                    if not st["armed"]:
                        break
            break
        send(cmd)
        time.sleep(max(0.0, 1.0 / args.rate_hz - (time.time() - now)))

    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phase", "height_m", "height_sp_m", "vz_ms", "vz_sp_ms", "thrust"])
        w.writerows(rows)
    print(f"[offboard] {len(rows)} steps written to {args.csv}; final phase {ctl.phase} "
          f"{ctl.reason}; armed={st['armed']}", flush=True)
    return 0 if ctl.phase == DONE else 1


if __name__ == "__main__":
    raise SystemExit(main())
