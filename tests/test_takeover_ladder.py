"""The takeover ladder (29 Oct 2026, docs/takeover-ladder-plan.md): telling a pilot takeover from
a lost link, the stage profiles that hold each stage open, and the simulator's ELRS outages."""
import math
import random
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import drone_agent as agent  # noqa: E402
import offboard_control as oc  # noqa: E402
import safety_rules as sr  # noqa: E402

G = 9.81


# ---- pilot or link? -------------------------------------------------------------------

def test_switch_off_centre_is_the_pilot():
    assert sr.classify_mode_exit([1500, 1010]) == "pilot_switch"
    assert sr.classify_mode_exit([1990]) == "pilot_switch"


def test_switch_still_centred_is_the_link():
    assert sr.classify_mode_exit([1500, 1495, 1503]) == "link_loss"


def test_no_reading_is_unknown_not_a_guess():
    assert sr.classify_mode_exit([]) == "unknown"


class FakeMav:
    def __init__(self):
        self.sent = []

    def command_long_send(self, target, comp, command, confirm, *params):
        self.sent.append(command)


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _msg(kind, **fields):
    m = types.SimpleNamespace(**fields)
    m.get_type = lambda: kind
    m.get_srcSystem = lambda: 1
    return m


def _hb(mode):
    return _msg("HEARTBEAT", type=2, base_mode=128, custom_mode=mode << 16)


def _rc(ch7=1500, ch8=1000, thr=1500):
    return _msg("RC_CHANNELS", chan3_raw=thr, chan7_raw=ch7, chan8_raw=ch8)


def _guard(tmp_path, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(agent.time, "time", clock)
    events = []
    monkeypatch.setattr(agent, "emit", events.append)
    args = types.SimpleNamespace(slot=1, no_safety=False, allow_kill=False,
                                 abort_flag=str(tmp_path / "ABORT"))
    conn = types.SimpleNamespace(target_system=1, mav=FakeMav())
    g = agent.Guard(conn, args)
    g.observe(_hb(6))                     # in OFFBOARD
    g.observe(_rc())
    return g, clock, events, conn


def _classified(events):
    return [e["source"] for e in events if e.get("event") == "takeover_classified"]


def test_guard_calls_a_switch_flip_a_pilot_takeover_at_once(tmp_path, monkeypatch):
    g, clock, events, conn = _guard(tmp_path, monkeypatch)
    clock.t += 1
    g.observe(_hb(2))                     # -> ALTCTL
    assert g.pilot_has_it and g.stop_streaming
    clock.t += 0.2
    g.observe(_rc(ch7=1000))
    g.check()
    assert g.takeover_source == "pilot_switch"
    assert _classified(events) == ["pilot_switch"]
    assert conn.mav.sent == []            # hands off: nothing sent


def test_guard_calls_a_mode_drop_with_the_switch_centred_a_lost_link(tmp_path, monkeypatch):
    g, clock, events, conn = _guard(tmp_path, monkeypatch)
    clock.t += 1
    g.observe(_hb(2))
    clock.t += 0.3
    g.observe(_rc(ch7=1500))
    g.check()
    assert g.takeover_source is None      # still waiting: a switch reading may be late
    clock.t += 1.5
    g.observe(_rc(ch7=1502))
    g.check()
    assert g.takeover_source == "link_loss" and g.link_losses == 1
    assert g.pilot_has_it                 # but still hands off: nothing is resumed
    assert conn.mav.sent == []


def test_a_late_switch_reading_still_counts_as_the_pilot(tmp_path, monkeypatch):
    g, clock, events, _ = _guard(tmp_path, monkeypatch)
    clock.t += 1
    g.observe(_hb(2))
    clock.t += 0.4
    g.observe(_rc(ch7=1500))              # sampled before the flip, delivered after
    g.check()
    clock.t += 0.5
    g.observe(_rc(ch7=1000))
    g.check()
    assert g.takeover_source == "pilot_switch"


def test_no_rc_at_all_is_unknown_and_hands_off(tmp_path, monkeypatch):
    g, clock, events, conn = _guard(tmp_path, monkeypatch)
    g._mode_pwm.clear()
    clock.t += 1
    g.observe(_hb(2))
    clock.t += 2
    g.check()
    assert g.takeover_source == "unknown" and g.pilot_has_it
    assert conn.mav.sent == []


def test_kill_switch_seen_on_the_rc_channel(tmp_path, monkeypatch):
    g, clock, events, conn = _guard(tmp_path, monkeypatch)
    g.observe(_rc(ch8=2000))
    assert g.pilot_has_it and g.killed_by_pilot
    assert g.takeover_source == "kill_switch"
    assert conn.mav.sent == []


def test_agents_climb_intent_replaces_the_throttle_stick(tmp_path, monkeypatch):
    g, clock, events, _ = _guard(tmp_path, monkeypatch)
    g.stick_override = 0.9
    g.wd.stick(0.9, clock.t)
    g.observe(_rc(thr=1500))              # the real stick, centred, must not overwrite it
    assert g.wd._stick == 0.9


# ---- stage profiles ---------------------------------------------------------------------

def test_each_flight_stage_is_held_open_for_the_pilot():
    p = oc.stage_profile("takeoff", height_m=1.0, wait_s=15)
    assert p["target_height_m"] == 0.3 and p["hold_s"] == 15
    assert oc.stage_profile("hold", height_m=1.0, wait_s=15)["hold_s"] == 15
    d = oc.stage_profile("descent", height_m=1.0, wait_s=15)
    assert d["descent_rate_ms"] == 0.08 and (1.0 - 0.3) / d["descent_rate_ms"] > 8
    assert oc.stage_profile("landing", height_m=1.0, wait_s=15)["final_descent_rate_ms"] == 0.05


def test_ground_stages_do_not_fly():
    for s in ("prearm", "arm"):
        try:
            oc.stage_profile(s, height_m=1.0, wait_s=15)
        except ValueError:
            continue
        raise AssertionError(f"{s} returned a flight profile")


def test_stage_names_follow_the_controller():
    assert oc.stage_of(oc.TAKEOFF, 0.1, 0.3) == "takeoff"
    assert oc.stage_of(oc.HOLD, 1.0, 0.3) == "hold"
    assert oc.stage_of(oc.LAND, 0.6, 0.3) == "descent"
    assert oc.stage_of(oc.LAND, 0.2, 0.3) == "landing"


def _fly_landing(lim, seconds=40.0, dt=0.02, true_hover=0.42):
    ctl = oc.HeightHold(lim, hold_s=2.0)
    h, v, t = 0.0, 0.0, 0.0
    ctl.start(t, h)
    zone_entered = None
    for _ in range(int(seconds / dt)):
        cmd = ctl.step(t, dt, h, True, v, 0.0)
        if cmd.phase == oc.LAND and cmd.height_sp is not None and cmd.height_sp <= 0.3 \
                and zone_entered is None:
            zone_entered = t
        if cmd.phase in (oc.DONE, oc.ABORT):
            return cmd.phase, t - (zone_entered or t)
        a = G * (cmd.thrust / true_hover - 1.0) - 0.5 * v
        v += a * dt
        h = max(0.0, h + v * dt)
        if h == 0.0:
            v = max(0.0, v)
        t += dt
    return ctl.phase, None


def test_slow_final_zone_lands_and_lasts():
    fast = _fly_landing(oc.Limits(hover_thrust=0.42))
    slow = _fly_landing(oc.Limits(hover_thrust=0.42, final_descent_rate_ms=0.05))
    assert fast[0] == oc.DONE and slow[0] == oc.DONE
    assert slow[1] > fast[1] + 3.0        # the landing stage is held open for seconds


# ---- the simulator's ELRS outages ---------------------------------------------------------

def test_outages_come_in_bursts_of_the_given_length():
    m = oc.OutageModel(10.0, 0.5, 1.5, random.Random(1))
    t, dt, gaps, run = 0.0, 0.05, [], 0.0
    while t < 600:
        if m.dropped(t):
            run += dt
        elif run:
            gaps.append(run)
            run = 0.0
        t += dt
    assert 30 < len(gaps) < 100           # about one per 10 s
    assert all(0.4 <= g <= 1.6 for g in gaps)


# ---- why flight stages need a wired link (29 Oct 2026) -------------------------------------

def _fly_delayed(delay_s, hz=5.0, true_hover=0.42, seconds=30.0, dt=0.01):
    """The 1-D model with the radio in the loop: setpoints at `hz`, arriving `delay_s` late."""
    ctl = oc.HeightHold(oc.Limits(hover_thrust=0.42), hold_s=15.0)
    h = v = t = 0.0
    ctl.start(t, 0.0)
    queue, applied, last_tx, peak = [], 0.12, -1.0, 0.0
    for i in range(int(seconds / dt)):
        if i % 5 == 0:
            cmd = ctl.step(t, 0.05, h, True, v, 0.0)
            if cmd.phase == oc.ABORT:
                return ctl.reason, peak
            if t - last_tx >= 1.0 / hz - 1e-9:
                last_tx = t
                queue.append((t + delay_s, cmd.thrust))
        while queue and queue[0][0] <= t:
            applied = queue.pop(0)[1]
        v += (G * (applied / true_hover - 1.0) - 0.5 * v) * dt
        h = max(0.0, h + v * dt)
        if h == 0.0:
            v = max(0.0, v)
        peak = max(peak, h)
        t += dt
    return None, peak


def test_the_height_loop_holds_with_no_radio_in_it():
    reason, peak = _fly_delayed(0.0)
    assert reason is None and peak < 1.05


def test_elrs_delay_drives_the_height_loop_through_the_ceiling():
    reason, peak = _fly_delayed(0.35)
    assert reason and "ceiling" in reason and peak > 1.5


def test_flight_stages_are_refused_on_a_real_aircraft_over_the_radio():
    src = (Path(agent.__file__)).read_text()
    assert 'wired = args.link.startswith(("serial:", "/dev/"))' in src


# ---- fixes after the 29 Oct SITL campaign ---------------------------------------------------

def test_guard_height_is_measured_from_the_agents_zero(tmp_path, monkeypatch):
    g, clock, events, _ = _guard(tmp_path, monkeypatch)
    g.z0 = -0.9                           # the EKF origin had drifted 0.9 m by take-off
    g.observe(_msg("LOCAL_POSITION_NED", z=-2.0, vz=0.0))
    assert abs(g.wd._height - 1.1) < 1e-9   # 1.1 m above the floor, not 2.0 over the ceiling


def test_the_takeoff_pause_counts_as_takeoff():
    src = (Path(agent.__file__)).read_text()
    assert 'enter("takeoff" if stage == "takeoff" and name == "hold" else name)' in src


def test_ground_stages_do_not_need_a_height_estimate():
    src = (Path(agent.__file__)).read_text()
    assert 'if state["z"] is None and upto >= oc.STAGES.index("takeoff"):' in src


# ---- height from the rangefinder (29 Oct 2026, after ground truth) ---------------------------

def test_a_fresh_rangefinder_reading_is_the_height():
    hs = oc.HeightSource()
    hs.range(2.0, 10.0)
    assert hs.height(1.1, 10.1) == 2.0    # the estimator read 0.9 m low; the range is truth


def test_below_the_rangefinder_the_estimator_carries_its_last_offset():
    hs = oc.HeightSource()
    hs.range(0.45, 10.0)
    assert hs.height(0.25, 10.0) == 0.45
    h = hs.height(0.15, 11.0)             # range stale: descended under its 0.4 m floor
    assert abs(h - 0.35) < 1e-9           # 0.15 + the 0.2 m offset, no step at the hand-over


def test_no_reading_at_all_is_no_height():
    assert oc.HeightSource().height(None, 0.0) is None
    hs = oc.HeightSource()
    assert hs.height(0.3, 0.0) == 0.3     # estimator alone until the rangefinder speaks


def test_the_agent_flies_on_the_height_source():
    src = (Path(agent.__file__)).read_text()
    assert "return heights.height(estimate(), time.time())" in src
    assert "heights.range(msg.current_distance / 100.0, time.time())" in src


# ---- gentle take-off: accel_max 1.0 m/s^2 (operator, 29 Oct 2026) ----------------------------

def _takeoff(true_hover, guess=0.42, target=0.3, dip=0.1, lag=0.15, seconds=12.0, dt=0.005):
    """1-D model with SITL's faults: 20 Hz loop, delayed telemetry, an estimate that dips on the
    floor as thrust rises. Returns (peak height, peak upward accel, peak speed) once airborne."""
    ctl = oc.HeightHold(oc.Limits(hover_thrust=guess, target_height_m=target, ceiling_m=1.2),
                        hold_s=5.0)
    ctl.start(0.0, 0.0)
    h = v = t = nxt = 0.0
    applied, hist, peak, a_up, v_max = 0.12, [], 0.0, 0.0, 0.0
    while t < seconds:
        hist.append((t, h, v, applied))
        if t >= nxt:
            nxt += 0.05
            _, hh, vv, th = ([x for x in hist[-100:] if x[0] <= t - lag] or hist[:1])[-1]
            est = hh - (dip * min(1.0, th / 0.5) if hh < 0.15 else 0.0)
            cmd = ctl.step(t, 0.05, est, True, vv, 0.0)
            assert cmd.phase != oc.ABORT, ctl.reason
            applied = cmd.thrust
        a = G * (applied / true_hover - 1.0) - 0.5 * v
        a = 0.0 if h <= 0.0 and a < 0.0 else a
        v += a * dt
        h = max(0.0, h + v * dt)
        v = max(0.0, v) if h == 0.0 else v
        if h > 0.005:
            a_up, v_max = max(a_up, a), max(v_max, v)
        peak, t = max(peak, h), t + dt
    return peak, a_up, v_max


def test_takeoff_is_gentle_when_the_hover_guess_is_right():
    peak, a_up, v_max = _takeoff(0.42)
    assert peak < 0.36 and a_up < 1.5 and v_max < 0.7   # before: 0.50 m, 6.3 m/s^2, 1.38 m/s


def test_takeoff_stays_gentle_with_the_hover_guess_off_by_ten_percent():
    for true_hover in (0.38, 0.46):
        peak, a_up, v_max = _takeoff(true_hover)
        assert peak < 0.36 and a_up < 2.0 and v_max < 0.7


def test_a_heavy_aircraft_still_gets_off_the_floor_inside_the_timeout():
    peak, _, _ = _takeoff(0.60, seconds=14.0)          # hover 43 % above the guess
    assert peak > 0.25


# ---- T3 run 1 fixes (29 Oct 2026), each behind its own switch --------------------------------

def _spool_until(lim, vz_of_t, seconds=8.0):
    ctl = oc.HeightHold(lim, hold_s=5.0)
    ctl.start(0.0, 0.0)
    t = 0.0
    while t < seconds and not ctl.airborne:
        ctl.step(t, 0.05, 0.0, True, vz_of_t(t), 0.0)  # the estimate never reaches 0.15 m
        t += 0.05
    return ctl


def test_fix1_lift_off_is_seen_from_a_sustained_climb_rate():
    lim = oc.Limits(hover_thrust=0.42, lift_by_climb=True)
    ctl = _spool_until(lim, lambda t: 0.08 if t >= 4.0 else 0.0)
    assert ctl.airborne and 4.15 <= ctl.airborne_at <= 4.35


def test_fix1_off_by_default_so_the_height_rule_alone_applies():
    ctl = _spool_until(oc.Limits(hover_thrust=0.42), lambda t: 0.08 if t >= 4.0 else 0.0)
    assert not ctl.airborne


def test_fix1_a_climb_rate_glitch_at_low_thrust_is_not_lift():
    lim = oc.Limits(hover_thrust=0.42, lift_by_climb=True)
    ctl = _spool_until(lim, lambda t: 0.4 if t < 0.5 else 0.0, seconds=1.0)
    assert not ctl.airborne


def test_fix1_the_floor_is_zero_whatever_the_estimate_drifted_to():
    hs = oc.HeightSource()
    hs.floor(-0.27, 10.0)                 # ARTEX113's estimate fell 0.27 m under the spool
    assert abs(hs.height(-0.27, 10.0)) < 1e-9
    assert abs(hs.height(-0.17, 10.5) - 0.10) < 1e-9   # then it climbs 0.10 m


def test_fix1_the_rangefinder_still_wins_when_fresh():
    hs = oc.HeightSource()
    hs.range(0.45, 10.0)
    hs.floor(-0.27, 10.1)                 # fresh range: the floor call changes nothing
    assert hs.height(-0.27, 10.1) == 0.45


def test_fix3_attitude_quaternion_matches_level_and_holds_the_angles():
    assert all(abs(a - b) < 1e-12 for a, b in zip(oc.attitude_quaternion(0, 0, 0.7),
                                                   oc.level_quaternion(0.7)))
    w, x, y, z = oc.attitude_quaternion(0.0, math.radians(-1.3), 0.0)
    pitch = math.degrees(math.asin(2 * (w * y - z * x)))
    assert abs(pitch + 1.3) < 1e-9


def test_fix_switches_are_wired_into_the_agent():
    src = (Path(agent.__file__)).read_text()
    assert "lift_by_climb=args.lift_by_climb" in src
    assert "heights.floor(estimate(), now)" in src
    assert "cmd.q = oc.attitude_quaternion(k * floor_att[0], k * floor_att[1], ctl.yaw_rad)" in src
