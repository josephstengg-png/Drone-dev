"""Whether the RC switch can take the aircraft is a yes/no question with specific causes.

The most important case is the PX4 default: COM_RC_OVERRIDE=1 looks enabled but leaves
OFFBOARD -- the mode every arming test uses -- immune to the sticks.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import drone_agent as agent  # noqa: E402


def _good(**over):
    p = {"COM_RC_IN_MODE": 0, "COM_RC_OVERRIDE": 3, "COM_RC_STICK_OV": 30.0,
         "RC_MAP_FLTMODE": 5, "RC_MAP_KILL_SW": 7, "COM_RCL_EXCEPT": 0,
         "COM_FLTMODE1": 2, "COM_FLTMODE2": -1, "COM_FLTMODE3": -1,
         "COM_FLTMODE4": -1, "COM_FLTMODE5": -1, "COM_FLTMODE6": 7}
    p.update(over)
    return p


def _findings(p, rc_seen=True):
    return agent.rc_takeover_findings(p, rc_seen)


def test_a_properly_configured_aircraft_has_nothing_to_report():
    assert _findings(_good()) == []


def test_the_px4_default_cannot_break_offboard():
    f = _findings(_good(COM_RC_OVERRIDE=1))        # PX4's shipped default
    assert len(f) == 1
    assert "CANNOT break OFFBOARD" in f[0]


def test_override_disabled_entirely_is_reported_twice():
    f = _findings(_good(COM_RC_OVERRIDE=0))
    assert any("OFFBOARD" in x for x in f)
    assert any("AUTO" in x for x in f)


def test_offboard_only_override_still_warns_about_auto():
    f = _findings(_good(COM_RC_OVERRIDE=2))
    assert len(f) == 1 and "AUTO" in f[0]


def test_stick_input_disabled_is_caught():
    assert any("does not accept RC stick input" in x
               for x in _findings(_good(COM_RC_IN_MODE=4)))


def test_joystick_only_is_not_good_enough_for_a_real_transmitter():
    assert any("COM_RC_IN_MODE=1" in x for x in _findings(_good(COM_RC_IN_MODE=1)))


def test_rc_and_joystick_is_accepted():
    assert _findings(_good(COM_RC_IN_MODE=2)) == []


def test_no_mode_channel_is_caught():
    assert any("RC_MAP_FLTMODE=0" in x for x in _findings(_good(RC_MAP_FLTMODE=0)))


def test_a_mode_switch_that_never_hands_over_is_caught():
    # every slot is still the autopilot flying: Mission, Hold, Return, Offboard
    f = _findings(_good(COM_FLTMODE1=3, COM_FLTMODE6=5))
    assert any("hand-flown mode" in x for x in f)


def test_each_hand_flown_mode_counts():
    for mode in (0, 1, 2, 6, 8, 9):
        assert _findings(_good(COM_FLTMODE1=mode)) == [], f"mode {mode} should hand over"


def test_autopilot_modes_do_not_count_as_handing_over():
    for mode in (3, 4, 5, 7, 10, 11):
        assert any("hand-flown" in x for x in _findings(_good(COM_FLTMODE1=mode, COM_FLTMODE6=-1)))


def test_a_missing_kill_switch_is_reported():
    assert any("RC_MAP_KILL_SW=0" in x for x in _findings(_good(RC_MAP_KILL_SW=0)))


def test_a_transmitter_that_was_never_heard_is_the_headline():
    assert any("no RC_CHANNELS heard" in x for x in _findings(_good(), rc_seen=False))


def test_an_unreadable_override_is_reported_not_assumed_good():
    assert any("could not be read" in x for x in _findings(_good(COM_RC_OVERRIDE=None)))
