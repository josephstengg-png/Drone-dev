"""The baseline tool's decisions, without an aircraft."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import param_baseline as pb  # noqa: E402


def test_plan_marks_change_keep_and_unreadable():
    cur = {"GF_ACTION": 2, "GF_MAX_VER_DIST": 3.0, "NAV_DLL_ACT": None}
    plan = {p["name"]: p["action"] for p in pb.make_plan(cur, {
        "GF_ACTION": 5, "GF_MAX_VER_DIST": 3.0, "NAV_DLL_ACT": 3})}
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
    assert pb.INDOOR_BASELINE["GF_MAX_VER_DIST"] == 3.0


def test_indoor_and_outdoor_share_the_common_safety_settings():
    for sysid in (113, 115):
        for name in ("indoor", "outdoor"):
            v = pb.profile(name, sysid)
            assert v["GF_ACTION"] == 5 and v["NAV_DLL_ACT"] == 3
            assert v["EKF2_GPS_CHECK"] == 1023
            assert pb.battery_order_ok(v)


def test_indoor_turns_gnss_fusion_off_and_outdoor_restores_each_aircraft():
    assert pb.profile("indoor", 115)["EKF2_GPS_CTRL"] == 0
    assert pb.profile("indoor", 113)["EKF2_GPS_CTRL"] == 0
    assert pb.profile("outdoor", 115)["EKF2_GPS_CTRL"] == 7
    assert pb.profile("outdoor", 113)["EKF2_GPS_CTRL"] == 5


def test_export_uses_the_dump_types(tmp_path):
    dump = tmp_path / "d.params"
    rows = ["# x", "115\t1\tGF_ACTION\t2\t6", "115\t1\tNAV_DLL_ACT\t0\t6",
            "115\t1\tBAT_LOW_THR\t0.4\t9", "115\t1\tBAT_CRIT_THR\t0.25\t9",
            "115\t1\tBAT_EMERGEN_THR\t0.5\t9", "115\t1\tEKF2_GPS_CHECK\t0\t6",
            "115\t1\tGF_MAX_VER_DIST\t7.0\t9", "115\t1\tEKF2_GPS_CTRL\t7\t6"]
    dump.write_text("\n".join(rows) + "\n")
    out = tmp_path / "o.params"

    class A:
        export_from, out_, profile, sysid = str(dump), None, "indoor", 115
    a = A(); a.out = str(out)
    assert pb.export_profile(a) == 0
    text = out.read_text()
    assert "115\t1\tEKF2_GPS_CTRL\t0\t6" in text
    assert "115\t1\tGF_MAX_VER_DIST\t3.0\t9" in text
