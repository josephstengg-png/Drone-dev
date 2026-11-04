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
