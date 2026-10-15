"""The baseline tool's decisions, without an aircraft."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import param_baseline as pb  # noqa: E402


def test_plan_marks_change_keep_and_unreadable():
    cur = {"GF_ACTION": 2, "GF_MAX_VER_DIST": 1.0, "NAV_DLL_ACT": None}
    plan = {p["name"]: p["action"] for p in pb.make_plan(cur, {
        "GF_ACTION": 5, "GF_MAX_VER_DIST": 1.0, "NAV_DLL_ACT": 3})}
    assert plan == {"GF_ACTION": "change", "GF_MAX_VER_DIST": "keep",
                    "NAV_DLL_ACT": "skip-unreadable"}


def test_float_noise_is_not_a_change():
    cur = {"BAT_LOW_THR": 0.20000000298023224}
    assert pb.make_plan(cur, {"BAT_LOW_THR": 0.20})[0]["action"] == "keep"


def test_the_baseline_itself_has_ordered_battery_thresholds():
    assert pb.battery_order_ok(pb.INDOOR_BASELINE)


def test_the_bad_113_values_are_rejected_by_the_order_check():
    assert not pb.battery_order_ok({"BAT_LOW_THR": 0.4, "BAT_CRIT_THR": 0.25,
                                    "BAT_EMERGEN_THR": 0.5})


def test_baseline_lands_in_place_on_every_loss():
    assert pb.INDOOR_BASELINE["GF_ACTION"] == 5
    assert pb.INDOOR_BASELINE["NAV_DLL_ACT"] == 3
    assert pb.INDOOR_BASELINE["GF_MAX_VER_DIST"] == 1.0
