"""The takeover ladder (29 Oct 2026, docs/takeover-ladder-plan.md): telling a pilot takeover from
a lost link, the stage profiles that hold each stage open, and the simulator's ELRS outages."""
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


def test_spool_freezes_at_the_first_sign_of_lift():
    ctl = oc.HeightHold(oc.Limits(hover_thrust=0.5), hold_s=5.0)
    ctl.start(0.0, 0.0)
    t, thrusts = 0.0, []
    for i in range(40):                   # 2 s on the floor; climbing from 1 s
        vz = 0.2 if t >= 1.0 else 0.0
        thrusts.append(ctl.step(t, 0.05, 0.0, True, vz, 0.0).thrust)
        t += 0.05
    assert thrusts[19] > thrusts[10]      # ramping before lift
    assert len(set(thrusts[21:])) == 1    # frozen after it
    assert thrusts[-1] < 0.5 + 0.2        # short of the full spool


def test_a_dip_on_the_floor_does_not_hide_lift():
    ctl = oc.HeightHold(oc.Limits(hover_thrust=0.5), hold_s=5.0)
    ctl.start(0.0, 0.0)
    for i, h in enumerate([0.0, -0.05, -0.10, -0.10, -0.04, -0.03]):
        cmd = ctl.step(i * 0.05, 0.05, h, True, 0.0, 0.0)
    assert ctl._lift_thrust is not None   # risen 0.07 m from the lowest reading


def test_takeoff_setpoint_waits_on_the_floor():
    ctl = oc.HeightHold(oc.Limits(hover_thrust=0.5), hold_s=5.0)
    ctl.start(0.0, 0.0)
    for i in range(30):                   # 1.5 s stuck on the floor
        cmd = ctl.step(i * 0.05, 0.05, 0.0, True, 0.0, 0.0)
    assert cmd.height_sp == 0.0           # not 0.6 m of climb demand waiting at lift-off


def test_the_takeoff_pause_counts_as_takeoff():
    src = (Path(agent.__file__)).read_text()
    assert 'enter("takeoff" if stage == "takeoff" and name == "hold" else name)' in src
