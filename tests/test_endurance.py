"""The endurance hover (4 Nov 2026): up to 2.0 m, holds of 30 min and more, battery logging and thresholds."""
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import drone_agent as agent  # noqa: E402
import gs_core as core  # noqa: E402
import offboard_control as oc  # noqa: E402


# ---- the agent's height limits for a real aircraft ----

def test_the_default_cap_and_ceiling_are_unchanged():
    assert agent.real_height_limits(0.8, None, 5, 3.0) == (1.2, None)
    ceiling, err = agent.real_height_limits(1.1, None, 5, 3.0)
    assert ceiling is None and "1.05 m cap" in err and "--max-height-m" in err


def test_max_height_raises_the_cap_and_the_ceiling_follows():
    assert agent.real_height_limits(2.0, 2.0, 5, 3.0) == (2.15, None)
    assert agent.real_height_limits(1.5, 1.6, 5, 3.0) == (1.75, None)
    assert agent.real_height_limits(2.0, 2.0, 5, 3.0)[0] > 2.0          # the abort ceiling is above the hover


def test_nothing_goes_above_two_metres():
    ceiling, err = agent.real_height_limits(2.0, 2.5, 5, 3.0)
    assert ceiling is None and "above the 2 m limit" in err
    assert agent.real_height_limits(2.1, 2.0, 5, 3.0)[0] is None


def test_px4s_own_geofence_is_checked_first():
    ceiling, err = agent.real_height_limits(2.0, 2.0, 5, 2.2)           # fence 2.2 m, hover 2.0 m
    assert ceiling is None and "GF_MAX_VER_DIST" in err
    assert agent.real_height_limits(2.0, 2.0, 5, 3.0)[1] is None         # ARTEX113 today: 3.0 m
    assert agent.real_height_limits(2.0, 2.0, 0, 1.0)[1] is None         # GF_ACTION 0: fence off
    assert agent.real_height_limits(2.0, 2.0, 5, 0.0)[1] is None         # no vertical limit set
    assert agent.real_height_limits(0.8, None, 5, 1.0)[0] is None         # the old 1.0 m fence vs a 0.8 m hover


def test_the_guard_ceiling_follows_the_controllers_so_a_2m_hover_is_not_a_false_alarm():
    src = Path(agent.__file__).read_text()
    assert "args.ceiling_m = max(2.0, ceiling + 0.15)" in src
    # default behaviour: 1.2 + 0.15 < 2.0, so the guard ceiling stays at its old 2.0 m
    assert max(2.0, 1.2 + 0.15) == 2.0 and max(2.0, 2.15 + 0.15) == 2.3


def test_the_controller_holds_a_two_metre_hover_for_more_than_thirty_minutes():
    lim = oc.Limits(target_height_m=2.0, ceiling_m=2.15, hover_thrust=0.42)
    ctl = oc.HeightHold(lim, yaw_rad=0.0, hold_s=2400.0)
    ctl.start(0.0, 0.0)
    h = v = t = 0.0
    held = None
    for i in range(int(2500 / 0.05)):
        cmd = ctl.step(t, 0.05, h, True, v, 0.0)
        if cmd.phase == oc.ABORT:
            raise AssertionError(f"aborted at {t:.0f}s: {ctl.reason}")
        # a crude plant: thrust above hover climbs
        v += (9.81 * (cmd.thrust / 0.42 - 1.0) - 0.5 * v) * 0.05
        h = max(0.0, h + v * 0.05)
        if cmd.phase == oc.HOLD and held is None and abs(h - 2.0) < 0.1:
            held = t
        if held is not None and t - held > 2300:
            break
        t += 0.05
    assert held is not None and t - held > 2300 and abs(h - 2.0) < 0.15


# ---- the battery log ----

def _guard(tmp_path, monkeypatch):
    monkeypatch.setattr(agent, "emit", lambda e: None)
    args = types.SimpleNamespace(slot=1, no_safety=False, allow_kill=False, abort_flag=str(tmp_path / "ABORT"))
    return agent.Guard(types.SimpleNamespace(target_system=1, mav=types.SimpleNamespace()), args)


def _sys(mv, pct):
    m = types.SimpleNamespace(voltage_battery=mv, battery_remaining=pct)
    m.get_type = lambda: "SYS_STATUS"
    m.get_srcSystem = lambda: 1
    return m


def test_the_guard_remembers_volts_and_percent_for_the_endurance_log(tmp_path, monkeypatch):
    g = _guard(tmp_path, monkeypatch)
    assert g.batt_v is None and g.batt_pct is None
    g.observe(_sys(15712, 81))
    assert g.batt_v == pytest.approx(15.712) and g.batt_pct == 81
    g.observe(_sys(65535, -1))                    # "no battery" / no estimate is unknown, not 65 V or -1 %
    assert g.batt_v is None and g.batt_pct is None


def test_the_trace_and_the_event_carry_the_battery():
    src = Path(agent.__file__).read_text()
    assert '"t_s,phase,height_m,height_sp_m,thrust,mode,batt_v,batt_pct\\n"' in src
    assert 'emit(dict(event="battery"' in src and "now - last_batt >= 30.0" in src


def test_the_guard_battery_thresholds_come_from_the_command_line():
    a = types.SimpleNamespace(battery_hold_pct=12.0, battery_land_pct=8.0)
    lim = agent.limits_from_args(a)
    assert (lim.battery_hold_pct, lim.battery_land_pct) == (12.0, 8.0)
    d = agent.limits_from_args(types.SimpleNamespace())
    assert (d.battery_hold_pct, d.battery_land_pct) == (25.0, 15.0)      # defaults untouched


# ---- the GUI core ----

def prof(**kw):
    base = dict(name="Endurance", flight_type="tether", link="usb", stage="disarm", height_m=2.0, hold_s=2400)
    base.update(kw)
    return core.normalise(base)


def test_a_two_metre_forty_minute_profile_is_runnable_and_asks_for_the_cap():
    p = prof()
    assert core.validate(p) == []
    a = core.agent_argv(p, "/tmp/t.csv")
    assert a[a.index("--max-height-m") + 1] == "2" and a[a.index("--hold-s") + 1] == "2400"
    assert "--max-height-m" not in core.agent_argv(prof(height_m=0.8, hold_s=5), "/tmp/t.csv")
    assert core.validate(prof(height_m=2.1)) and core.validate(prof(hold_s=7300))


def test_battery_thresholds_validate_and_reach_the_command():
    assert core.validate(prof(battery_hold_pct=12, battery_land_pct=8)) == []
    assert core.validate(prof(battery_hold_pct=5, battery_land_pct=8))        # hold below land
    assert core.validate(prof(battery_land_pct=90))
    a = core.agent_argv(prof(battery_hold_pct=12, battery_land_pct=8), "/tmp/t.csv")
    assert a[a.index("--battery-hold-pct") + 1] == "12" and a[a.index("--battery-land-pct") + 1] == "8"
    assert "--battery-land-pct" not in core.agent_argv(prof(), "/tmp/t.csv")
    assert "--allow-kill" not in a


def test_a_long_deep_battery_hover_lists_what_is_being_accepted():
    ids = {i["id"] for i in core.assess(prof(battery_land_pct=8), dict(preflight_go=True, preflight_age_s=5))
           if i["level"] == "accept"}
    assert {"long_hover", "battery_deep"} <= ids
    short = {i["id"] for i in core.assess(prof(height_m=0.8, hold_s=5), {}) if i["level"] == "accept"}
    assert "long_hover" not in short and "battery_deep" not in short


def test_the_chart_covers_a_forty_minute_hold():
    tl = core.timeline(prof())
    assert tl["total_s"] > 2400 and max(h for _, h in tl["points"]) == 2.0


# ---- a late ARM must not be left behind (4 Nov 2026) ----

def test_a_late_arm_is_met_with_disarm_and_reported():
    state = dict(armed=False)
    sent = []

    def pump(_s):
        if len(sent) == 2:           # the ARM that was queued in the radio finally lands
            state["armed"] = True
        elif len(sent) == 4:         # and the DISARM sent behind it lands after
            state["armed"] = False

    r = agent.withdraw_arm(lambda: sent.append("DISARM"), pump, state, seconds=30)
    assert r == dict(armed_late=True, armed_now=False) and len(sent) >= 4


def test_a_quiet_link_sends_disarm_for_the_whole_window_and_reports_not_late(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(agent.time, "time", lambda: clock[0])
    sent = []

    def pump(s):
        clock[0] += s

    r = agent.withdraw_arm(lambda: sent.append(1), pump, dict(armed=False), seconds=10)
    assert r == dict(armed_late=False, armed_now=False) and 9 <= len(sent) <= 11


def test_the_agent_withdraws_before_giving_up_and_never_forces():
    src = Path(agent.__file__).read_text()
    assert "withdraw_arm(lambda: conn.mav.command_long_send(conn.target_system, 1," in src
    assert "21196" not in src.split("def withdraw_arm")[1].split("def do_disarm")[0]    # no force-disarm


def test_setpoint_rate_is_a_profile_field_that_reaches_the_command():
    p = core.normalise(dict(name="Rate test", flight_type="bench", link="radio", stage="arm", setpoint_hz=5))
    assert core.validate(p) == []
    a = core.agent_argv(p, "/tmp/t.csv")
    assert a[a.index("--setpoint-hz") + 1] == "5"
    assert "--setpoint-hz" not in core.agent_argv(core.normalise(dict(name="x1", stage="arm")), "/tmp/t.csv")
    assert core.validate(core.normalise(dict(name="x1", setpoint_hz=80)))


# ---- the height feed must be able to keep the controller alive (4 Nov 2026) ----

def test_a_radio_height_feed_that_is_slower_than_the_timeout_is_refused():
    # the 13:54 run: readings every 3.3 s against a 3 s timeout
    stamps = [0.0, 3.3, 6.6, 9.9]
    ok, worst = agent.height_feed_check(stamps, 10.0, 3.0)
    assert not ok and worst == pytest.approx(3.3)


def test_a_feed_faster_than_the_timeout_passes_and_the_open_gap_counts():
    assert agent.height_feed_check([0, 1, 2, 3, 4, 5, 6, 7, 8, 9], 9.5, 3.0)[0]
    ok, worst = agent.height_feed_check([0, 1, 2], 9.0, 3.0)         # then it went quiet
    assert not ok and worst == pytest.approx(7.0)
    assert agent.height_feed_check([], 9.0, 3.0) == (False, float("inf"))


def test_the_check_runs_before_the_gate_for_real_flight_stages_only():
    src = Path(agent.__file__).read_text()
    i, j = src.index("height_feed_ready(stamps, time.time(), timeout)"), src.index('emit(dict(event="ready", slot=args.slot, sysid=conn.target_system, stage=stage))')
    assert i < j                                                    # before READY, so before any ARM
    assert "if not sim and upto >= oc.STAGES.index(\"takeoff\"):" in src


# ---- the radio height feed (4 Nov 2026) ----

class _Cmd:
    def __init__(self):
        self.sent = []
        self.target_system = 113
        self.mav = self

    def command_long_send(self, *a):
        self.sent.append(a)


def test_the_agent_asks_for_the_height_feed_and_can_put_the_default_back():
    c = _Cmd()
    agent.set_height_feed(c, 10)
    agent.set_height_feed(c, 0)
    (sys1, comp1, cmd1, conf1, mid1, us1, *rest1), (_, _, _, _, mid2, us2, *_) = c.sent
    assert cmd1 == 511 and sys1 == 113 and mid1 == 32 and us1 == 100000        # LOCAL_POSITION_NED every 100 ms
    assert mid2 == 32 and us2 == 0                                              # 0 = back to the default rate


def test_the_height_feed_is_asked_for_only_over_a_radio_link_and_given_back():
    src = Path(agent.__file__).read_text()
    assert 'radio_link = not args.link.startswith(("serial:", "/dev/"))' in src
    assert "if radio_link and time.time() - last_ask >= 3.0:" in src          # re-asked: a command can be lost on the radio
    assert "set_height_feed(conn, 0)" in src                                  # and restored at exit


def test_only_the_recent_feed_counts_so_a_slow_start_is_forgiven():
    # the request for a faster feed takes seconds to land: 0.4 Hz for 12 s, then about 3 Hz
    slow = [0, 2.5, 5, 7.5, 10]
    fast = [12 + i / 3 for i in range(30)]
    ok, worst, n = agent.height_feed_ready(slow + fast, 22.0, 3.0)
    assert ok and worst < 1.0 and n >= 6
    assert not agent.height_feed_ready(slow, 11.0, 3.0)[0]


def test_a_steady_slow_feed_never_passes_even_in_a_lucky_window():
    stamps = [i * 2.2 for i in range(20)]                     # 0.45 Hz, the measured default
    assert not any(agent.height_feed_ready(stamps, 6.0 + k, 3.0)[0] for k in range(0, 30))
    assert not agent.height_feed_ready([0.0, 3.0, 6.0], 6.5, 3.0)[0]       # gaps just under the timeout, but only 0.5 Hz


def test_no_top_level_definition_in_the_agent_is_defined_twice():
    # 4 Nov 2026: a text splice duplicated a block of drone_agent.py; it compiled and the old copy silently won
    import ast
    names = [n.name for n in ast.parse(Path(agent.__file__).read_text()).body
             if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    dupes = sorted({n for n in names if names.count(n) > 1})
    assert dupes == []


# ---- the setpoint rate over a radio (4 Nov 2026) ----

def test_the_setpoint_rate_is_low_over_a_radio_and_high_on_a_wire_or_in_the_simulator():
    assert agent.default_setpoint_hz("udpout://127.0.0.1:14540", False, None) == 3.0          # ELRS relay
    assert agent.default_setpoint_hz("serial:/dev/ttyACM0", False, None) == 20.0               # USB
    assert agent.default_setpoint_hz("/dev/ttyACM0", False, None) == 20.0
    assert agent.default_setpoint_hz("udpin://:14540", True, None) == 20.0                     # SITL keeps its rate
    assert agent.default_setpoint_hz("udpout://127.0.0.1:14540", False, 8) == 8                # an explicit value wins


def test_the_agent_applies_the_default_before_anything_transmits():
    src = Path(agent.__file__).read_text()
    assert "args.setpoint_hz = default_setpoint_hz(args.link, sim, args.setpoint_hz)" in src
    assert src.index("args.setpoint_hz = default_setpoint_hz(") < src.index("1.0 / args.setpoint_hz")


# ---- stale ARM: retry slowly over a radio, and undo a late one (4 Nov 2026) ----

def test_arm_is_retried_slowly_over_a_radio_and_quickly_on_a_wire():
    assert agent.arm_retry_s("udpout://127.0.0.1:14540", False) == 8.0
    assert agent.arm_retry_s("serial:/dev/ttyACM0", False) == 3.0
    assert agent.arm_retry_s("udpin://:14540", True) == 3.0               # the simulator keeps its quick retry


def _watch(monkeypatch, events, seconds=6):
    """Run stale_arm_guard against a scripted timeline: events[k] = (armed, new_text or None) at the k-th pump."""
    clock = [0.0]
    monkeypatch.setattr(agent.time, "time", lambda: clock[0])
    state, texts, sent, k = dict(armed=False), ["Disarmed by RC switch"], [], [0]

    def pump(s):
        clock[0] += s
        if k[0] < len(events):
            armed, text = events[k[0]]
            state["armed"] = armed
            if text:
                texts.append(text)
            k[0] += 1

    r = agent.stale_arm_guard(pump, state, lambda: texts, lambda: sent.append(1), seconds=seconds)
    return r, sent


def test_a_late_external_arm_after_the_pilots_disarm_is_answered_with_disarm(monkeypatch):
    r, sent = _watch(monkeypatch, [(False, None), (True, "Armed by external command"), (True, None), (False, None)])
    assert r["stale_arm_seen"] and r["disarms_sent"] == 2 and len(sent) == 2


def test_the_pilots_own_re_arm_by_switch_is_never_fought(monkeypatch):
    r, sent = _watch(monkeypatch, [(False, None), (True, "Armed by RC switch"), (True, None)])
    assert not r["stale_arm_seen"] and sent == []


def test_an_arm_whose_message_was_lost_and_a_quiet_link_cause_no_action(monkeypatch):
    assert _watch(monkeypatch, [(False, None)] * 4)[1] == []
    assert _watch(monkeypatch, [(True, None), (True, None)])[1] == []      # armed, but no message: do not guess


def test_the_disarm_message_itself_is_not_mistaken_for_an_arm(monkeypatch):
    # "Disarmed by RC switch" contains "armed by": the regex must not match it
    r, sent = _watch(monkeypatch, [(True, "Disarmed by external command")])
    assert sent == []


def test_the_agent_counts_its_arm_commands_and_guards_after_a_disarm_without_losing_the_result():
    src = Path(agent.__file__).read_text()
    assert 'result["arm_commands_sent"], result["arm_retry_s"] = arms_sent, arm_gap' in src
    assert "if armed_ever[\"on\"] and result.get(\"disarmed\") is True:" in src
    assert "except Exception as exc:       # a dead link here must never cost the result" in src
