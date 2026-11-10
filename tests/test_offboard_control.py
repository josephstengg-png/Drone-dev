"""The indoor height-hold controller, flown against a 1-D vertical model.

The model: up-acceleration = g * (thrust / true_hover - 1) - drag * vz, sitting on the floor
when that would push it below zero. `props=False` gives no lift at all: the 15 Oct 2026
bench situation, where a controller that thought it was flying wound the motors up.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import offboard_control as oc  # noqa: E402

G = 9.81


def fly(ctl, seconds, true_hover=0.5, props=True, dt=0.02, height_valid=lambda t: True,
        tilt=lambda t: 0.0, land_at=None, h0=0.0):
    h, v, t = h0, 0.0, 1000.0
    ctl.start(t, h)
    trace = []
    for _ in range(int(seconds / dt)):
        if land_at is not None and abs(t - 1000.0 - land_at) < dt / 2:
            ctl.land(t)
        cmd = ctl.step(t, dt, h, height_valid(t - 1000.0), v, tilt(t - 1000.0))
        a = (G * (cmd.thrust / true_hover - 1.0) - 0.5 * v) if props else -G
        v += a * dt
        h += v * dt
        if h <= 0.0:
            h, v = 0.0, max(0.0, v)
        trace.append((t - 1000.0, cmd.phase, h, cmd.thrust, cmd.request))
        t += dt
        if cmd.phase in (oc.DONE, oc.ABORT):
            break
    return trace


def hold_samples(trace):
    return [h for (_, ph, h, _, _) in trace if ph == oc.HOLD]


def test_takes_off_and_holds_one_metre():
    tr = fly(oc.HeightHold(hold_s=8.0), 25.0)
    hs = hold_samples(tr)
    assert hs, "never reached HOLD"
    assert all(abs(h - 1.0) < 0.08 for h in hs[len(hs) // 4:]), "did not settle at 1 m"
    assert max(h for (_, _, h, _, _) in tr) < 1.25, "overshoot past 1.25 m"


def test_lands_and_asks_for_disarm():
    tr = fly(oc.HeightHold(hold_s=4.0), 40.0)
    assert tr[-1][1] == oc.DONE and tr[-1][4] == "disarm"
    assert tr[-1][2] <= 0.1


def test_wrong_hover_guess_is_absorbed_by_the_integrator():
    """The ARTEX's 0.5 hover thrust is a default; a heavier aircraft needs more."""
    tr = fly(oc.HeightHold(hold_s=10.0), 30.0, true_hover=0.6)
    hs = hold_samples(tr)
    assert hs and abs(hs[-1] - 1.0) < 0.1


def test_props_off_aborts_instead_of_winding_up():
    """15 Oct lesson: no lift must end in an abort at low thrust, not a wound-up integrator."""
    ctl = oc.HeightHold()
    tr = fly(ctl, 20.0, props=False)
    assert tr[-1][1] == oc.ABORT and tr[-1][4] == "land"
    assert "did not leave the ground" in ctl.reason
    lim = ctl.limits
    assert max(th for (_, _, _, th, _) in tr) <= lim.hover_thrust + lim.spool_margin + 1e-9, \
        "thrust climbed past the spool limit"
    assert ctl._i == 0.0, "integrator moved while on the ground"


def test_lost_height_estimate_aborts_to_px4_land():
    ctl = oc.HeightHold(hold_s=30.0)
    tr = fly(ctl, 30.0, height_valid=lambda t: t < 8.0)
    assert tr[-1][1] == oc.ABORT and "height estimate lost" in ctl.reason
    assert tr[-1][4] == "land"


def test_tilt_beyond_limit_aborts():
    ctl = oc.HeightHold(hold_s=30.0)
    tr = fly(ctl, 30.0, tilt=lambda t: 40.0 if t > 6.0 else 0.0)
    assert tr[-1][1] == oc.ABORT and "tilt" in ctl.reason


def test_ceiling_breach_aborts():
    ctl = oc.HeightHold(oc.Limits(target_height_m=1.0, ceiling_m=1.5), hold_s=30.0)
    fly(ctl, 1.0)                                       # get it going
    cmd = ctl.step(2000.0, 0.02, 1.8, True, 0.0, 0.0)  # a sudden 1.8 m reading
    assert cmd.phase == oc.ABORT and "ceiling" in cmd.reason


def test_thrust_always_within_limits():
    lim = oc.Limits()
    for hover in (0.4, 0.5, 0.6):
        for (_, _, _, th, _) in fly(oc.HeightHold(lim, hold_s=5.0), 30.0, true_hover=hover):
            assert lim.thrust_min - 1e-9 <= th <= lim.thrust_max + 1e-9 or th == 0.0


def test_operator_land_from_hold():
    tr = fly(oc.HeightHold(hold_s=60.0), 40.0, land_at=12.0)
    assert tr[-1][1] == oc.DONE


def test_attitude_is_level_with_the_held_yaw():
    q = oc.level_quaternion(math.radians(90))
    assert abs(q[0] - math.cos(math.pi / 4)) < 1e-9 and q[1] == 0.0 and q[2] == 0.0
    cmd = oc.HeightHold(yaw_rad=math.radians(90)).step(0.0, 0.02, 0.0, True, 0.0, 0.0)
    assert cmd.q == q and cmd.thrust == 0.0          # idle: no thrust until started
