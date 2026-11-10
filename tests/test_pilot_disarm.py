"""After a pilot takeover the agent sends nothing and makes sure the pilot knows.

Option C, chosen 14 Oct 2026. The wait is a pure function over a fake pump and a fake
clock, so every branch is exact and nothing is timing-dependent.
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import drone_agent as agent  # noqa: E402


class FakeWorld:
    """A clock the fake pump advances, and an aircraft that disarms at a chosen moment."""

    def __init__(self, disarm_at=None, armed=True):
        self.t = 1000.0
        self.disarm_at = None if disarm_at is None else self.t + disarm_at
        self.state = {"armed": armed}
        self.pumps = 0

    def clock(self):
        return self.t

    def pump(self, seconds, until=None):
        self.pumps += 1
        self.t += seconds
        if self.disarm_at is not None and self.t >= self.disarm_at:
            self.state["armed"] = False
        return True


def _events(capsys):
    out = capsys.readouterr().out
    return [json.loads(l) for l in out.splitlines() if l.startswith("{")]


def _run(w, **kw):
    kw.setdefault("killed", False)
    kw.setdefault("slot", 1)
    kw.setdefault("window_s", 20.0)
    return agent.await_pilot_disarm(w.pump, w.state, clock=w.clock, **kw)


def test_already_disarmed_returns_at_once_and_says_nothing(capsys):
    w = FakeWorld(armed=False)
    r = _run(w)
    assert r["disarmed"] is True and r["waited_s"] == 0.0
    assert w.pumps == 0
    assert _events(capsys) == []


def test_the_pilot_disarms_and_the_agent_reports_it(capsys):
    w = FakeWorld(disarm_at=7.0)
    r = _run(w)
    ev = _events(capsys)
    assert r["disarmed"] is True
    assert 6.5 <= r["waited_s"] <= 8.0
    assert ev[-1]["event"] == "pilot_disarmed"


def test_it_keeps_saying_so_while_the_aircraft_is_armed(capsys):
    w = FakeWorld(disarm_at=12.0)
    _run(w, warn_every_s=5.0)
    still = [e for e in _events(capsys) if e["event"] == "still_armed"]
    assert len(still) >= 3          # at 0, ~5 and ~10 s: a pilot cannot miss it


def test_nobody_disarms_so_it_leaves_armed_and_says_so(capsys):
    w = FakeWorld(disarm_at=None)
    r = _run(w, window_s=20.0)
    ev = _events(capsys)
    assert r["disarmed"] is False
    assert ev[-1]["event"] == "left_armed"
    assert any(e["event"] == "still_armed" for e in ev)


def test_a_kill_is_not_nagged_about(capsys):
    """PX4 disarms a kill by itself; shouting 'still armed' at the operator is noise."""
    w = FakeWorld(disarm_at=5.0)
    r = _run(w, killed=True)
    ev = _events(capsys)
    assert r["disarmed"] is True
    assert not any(e["event"] == "still_armed" for e in ev)


def test_the_wait_is_bounded(capsys):
    w = FakeWorld(disarm_at=None)
    _run(w, window_s=10.0)
    assert w.t - 1000.0 <= 11.0     # never an unbounded hang on an aircraft that stays armed


def test_it_takes_no_connection_so_it_cannot_send_anything():
    """The signature is the guarantee: pump and state only, no `conn` to command with."""
    import inspect
    params = set(inspect.signature(agent.await_pilot_disarm).parameters)
    assert "conn" not in params and "args" not in params


# --- the disarm loop must stop the moment the pilot has it -------------------------------

class _Guard:
    def __init__(self):
        self.pilot_has_it = False


def test_disarm_loop_stops_sending_once_the_pilot_has_it():
    """The kill lands while the agent's own disarm loop is running: no further disarm."""
    sent, state, guard = [], {"armed": True}, _Guard()

    def send():
        sent.append(1)

    def pump(seconds, until=None):
        if len(sent) == 1:                 # the kill arrives during the first wait
            guard.pilot_has_it = True

    out = agent.disarm_unless_pilot(send, pump, state, guard)
    assert out == "pilot"
    assert len(sent) == 1, "no disarm may be sent after the pilot has taken over"


def test_disarm_loop_sends_nothing_if_pilot_already_has_it():
    sent, state, guard = [], {"armed": True}, _Guard()
    guard.pilot_has_it = True
    out = agent.disarm_unless_pilot(lambda: sent.append(1), lambda *a, **k: None, state, guard)
    assert out == "pilot" and sent == []


def test_disarm_loop_still_disarms_when_nobody_takes_over():
    sent, state, guard = [], {"armed": True}, _Guard()

    def pump(seconds, until=None):
        if len(sent) == 2:
            state["armed"] = False         # takes two tries, like a lossy link

    out = agent.disarm_unless_pilot(lambda: sent.append(1), pump, state, guard)
    assert out == "disarmed" and len(sent) == 2


def test_disarm_loop_reports_failure_after_all_attempts():
    sent, state, guard = [], {"armed": True}, _Guard()
    out = agent.disarm_unless_pilot(lambda: sent.append(1), lambda *a, **k: None, state, guard,
                                    attempts=3)
    assert out == "failed" and len(sent) == 3


def test_disarm_loop_sends_nothing_to_an_aircraft_that_is_already_disarmed():
    """15 Oct 2026: the arm switch disarmed 113, and the agent still sent a disarm."""
    sent, state, guard = [], {"armed": False}, _Guard()
    out = agent.disarm_unless_pilot(lambda: sent.append(1), lambda *a, **k: None, state, guard)
    assert out == "disarmed" and sent == []
