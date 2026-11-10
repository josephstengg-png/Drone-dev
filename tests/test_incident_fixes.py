"""Fixes from the 15 Oct 2026 ARTEX113 incident (log read 19 Oct).

The agent printed "disarming", none of its disarms reached PX4, its heartbeat had stopped
silently, and it exited with the aircraft armed. These pin the four fixes.
"""
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import drone_agent as agent  # noqa: E402
import safety_rules as sr  # noqa: E402


# ---- 1. the heartbeat thread must survive a failed send ------------------------------

def test_heartbeat_keeps_beating_after_a_send_error():
    calls = []

    def heartbeat_send(*a):
        calls.append(time.time())
        if len(calls) == 1:
            raise OSError("transient send failure")

    conn = types.SimpleNamespace(write=lambda b: None,
                                 mav=types.SimpleNamespace(heartbeat_send=heartbeat_send))
    agent.start_heartbeat(conn, interval_s=0.01)
    time.sleep(0.2)
    assert len(calls) >= 5, "the heartbeat stopped after its first failure"
    assert conn.hb_failures == 1 and conn.hb_last_ok is not None


# ---- 3. a disarm counts only when the aircraft says so, recently -----------------------

def test_disarm_needs_a_fresh_heartbeat():
    assert agent.confirmed_disarmed({"armed": False, "last_hb": 100.0}, 101.0)
    assert not agent.confirmed_disarmed({"armed": False, "last_hb": 100.0}, 110.0), \
        "a stale 'disarmed' from a dead link is not a confirmation"
    assert not agent.confirmed_disarmed({"armed": True, "last_hb": 100.0}, 100.5)
    assert not agent.confirmed_disarmed({"armed": False}, 100.0)


# ---- 2. never just leave an armed aircraft ---------------------------------------------

class Clock:
    def __init__(self): self.t = 1000.0
    def __call__(self): return self.t


class G:
    pilot_has_it = False


def run_hold(disarm_after_s=None, pilot_after_s=None, link_dead=False, window=20.0):
    clk, state, guard, sent = Clock(), {"armed": True, "last_hb": 1000.0}, G(), []

    def pump(seconds, until=None):
        clk.t += seconds
        if not link_dead:
            state["last_hb"] = clk.t
        if disarm_after_s is not None and clk.t - 1000.0 >= disarm_after_s:
            state["armed"] = False
        if pilot_after_s is not None and clk.t - 1000.0 >= pilot_after_s:
            guard.pilot_has_it = True

    out = agent.hold_until_disarmed(lambda: sent.append(clk.t), pump, state, guard,
                                    window_s=window, clock=clk)
    return out, sent


def test_keeps_retrying_until_the_aircraft_confirms():
    out, sent = run_hold(disarm_after_s=7.0)
    assert out["outcome"] == "disarmed"
    assert 3 <= len(sent) <= 5            # every 2 s, not a burst


def test_stops_sending_when_the_pilot_takes_over():
    out, sent = run_hold(pilot_after_s=3.0)
    assert out["outcome"] == "pilot"
    assert all(t - 1000.0 <= 3.0 for t in sent)


def test_reports_left_armed_instead_of_exiting_quietly():
    out, sent = run_hold(window=10.0)
    assert out["outcome"] == "left_armed" and len(sent) >= 5


def test_a_dead_link_is_not_mistaken_for_a_disarm():
    """The last heartbeat said disarmed, but it is 10 s old and nothing has come since."""
    clk, guard, sent = Clock(), G(), []
    state = {"armed": False, "last_hb": clk.t - 10.0}

    def pump(seconds, until=None):
        clk.t += seconds                  # link dead: last_hb never refreshes

    out = agent.hold_until_disarmed(lambda: sent.append(clk.t), pump, state, guard,
                                    window_s=10.0, clock=clk)
    assert out["outcome"] == "left_armed" and len(sent) >= 5


# ---- 4. the guard does not score PX4's own failsafe and link reports -------------------

def test_guard_ignores_px4_consequence_texts():
    wd = sr.Watchdog(now=0.0)
    for i, text in enumerate(["Failsafe activated", "Connection to ground station lost",
                              "Preflight Fail: No connection to the ground control station",
                              "GCS connection regained"]):
        wd.statustext(4, text, now=float(i))
    assert wd.error_rate(now=5.0) == 0.0


def test_guard_still_scores_real_errors():
    wd = sr.Watchdog(now=0.0)
    wd.statustext(2, "Accel #0 fail: TIMEOUT!", now=1.0)
    assert wd.error_rate(now=2.0) > 0.0
