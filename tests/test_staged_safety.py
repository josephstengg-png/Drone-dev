"""The staged response (26 Oct 2026): descend first, cut after a set time, and the pre-arm
false alarm that latched HOLD before every SITL take-off."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import safety_rules as sr  # noqa: E402


def flying(cut=True, **kw):
    wd = sr.Watchdog(sr.Limits(cut_enabled=cut, **kw), now=0.0)
    wd.armed(False, 0.0)
    wd.armed(True, 0.0)
    wd.airborne(True, 0.0)
    wd.stick(0.5, 0.0)
    wd.vertical(1.5, 0.0, 0.0)
    wd.motors([0.4] * 4, 0.0)
    wd.attitude(0, 0, 0.0)
    return wd


def run(wd, t0, t1, step=0.1, **feed):
    t = t0
    v = None
    while t <= t1 + 1e-9:
        wd.heartbeat(t)
        if "tilt" in feed:
            wd.attitude(feed["tilt"], 0, t)
        if "vz" in feed:
            wd.vertical(None, feed["vz"], t)
        if "motor" in feed:
            wd.motors([feed["motor"], 0.4, 0.4, 0.4], t)
        if "height" in feed:
            wd.vertical(feed["height"], None, t)
        v = wd.evaluate(t)
        t += step
    return v


def test_prearm_chatter_is_not_an_error():
    wd = sr.Watchdog(now=0.0)
    wd.armed(False, 0.0)
    for i in range(20):
        wd.statustext(3, "Preflight Fail: ekf2 missing data", i)
        wd.statustext(2, "Arming denied: Resolve system health failures first", i)
    wd.armed(True, 20.0)
    wd.statustext(3, "Disarming denied: not landed", 21.0)
    wd.heartbeat(21.0)
    assert wd.evaluate(21.0).action == sr.NONE


def test_px4_failsafe_is_reported_as_warn_not_acted_on():
    wd = flying()
    wd.statustext(2, "Failsafe activated", 1.0)
    v = run(wd, 1.0, 2.0)
    assert v.action == sr.WARN


def test_brisk_pilot_flying_triggers_nothing():
    wd = flying()
    wd.stick(0.9, 0.0)                    # the pilot is asking for the climb
    assert run(wd, 0.0, 20.0, tilt=18, motor=0.70, vz=0.9).action == sr.NONE


def test_runaway_climb_is_cut_only_after_the_pilot_margin():
    wd = flying()                         # stick at centre, still climbing
    # stage 1 at 1.0 s; cut not before 1.0 + 3 + 2 (pilot margin)
    assert run(wd, 0.0, 5.9, vz=0.9).action == sr.LAND
    assert 0.0 < wd.cut_in(5.9) <= 0.2
    assert run(wd, 6.0, 6.5, vz=0.9).action == sr.CUT


def test_tilt_cut_gets_no_pilot_margin():
    wd = flying()
    assert run(wd, 0.0, 3.6, tilt=35).action == sr.CUT   # 0.5 s stage 1 + 3 s


def test_stage1_tilt_needs_half_a_second():
    wd = flying()
    assert run(wd, 0.0, 0.35, tilt=35).action == sr.NONE
    assert run(wd, 0.4, 0.7, tilt=35).action == sr.LAND


def test_stage1_saturation_and_ceiling():
    assert run(flying(), 0.0, 1.0, motor=0.9).action == sr.LAND
    assert run(flying(), 0.0, 0.2, height=2.3).action == sr.LAND


def test_cut_after_set_time_when_not_recovering():
    wd = flying()
    v = run(wd, 0.0, 3.4, tilt=35)        # stage 1 at 0.5 s; 3 s later still tilted
    assert v.action == sr.LAND
    v = run(wd, 3.5, 4.0, tilt=35)
    assert v.action == sr.CUT


def test_no_cut_when_the_descent_recovers():
    wd = flying()
    run(wd, 0.0, 1.0, tilt=35)            # stage 1
    v = run(wd, 1.1, 10.0, tilt=5, vz=-0.6)   # upright, descending
    assert v.action == sr.LAND


def test_cut_on_sustained_tumble():
    assert run(flying(), 0.0, 1.7, tilt=50).action == sr.CUT


def test_no_cut_without_allow_kill():
    v = run(flying(cut=False), 0.0, 10.0, tilt=50)
    assert v.action == sr.LAND


def test_cut_stays_cut_on_the_ground_and_disarm_never_in_air():
    assert sr.on_ground_action(sr.CUT, True) == sr.CUT
    assert sr.on_ground_action(sr.DISARM, True) == sr.LAND


def test_nothing_fires_while_disarmed():
    wd = sr.Watchdog(sr.Limits(cut_enabled=True), now=0.0)
    wd.armed(False, 0.0)
    assert run(wd, 0.0, 5.0, tilt=50, motor=0.95, height=3.0).action == sr.NONE
