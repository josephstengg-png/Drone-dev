"""The ground-station core (4 Nov 2026): profiles, the responsibility assessment, the agent command."""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import gs_core as core  # noqa: E402


@pytest.fixture(autouse=True)
def _dirs(tmp_path, monkeypatch):
    monkeypatch.setattr(core, "PROFILE_DIR", tmp_path / "profiles")
    monkeypatch.setattr(core, "RUN_DIR", tmp_path / "runs")


def prof(**kw):
    base = dict(name="Hover test", flight_type="tether", link="radio", stage="disarm", height_m=0.8)
    base.update(kw)
    return core.normalise(base)


GO = dict(preflight_go=True, preflight_age_s=30.0)


def full_body(p, ctx=GO, **kw):
    b = dict(operator="J Leong", aircraft=f"artex{p['sysid']}", phrase=core.PHRASE.lower(),
             accepted=[i["id"] for i in core.assess(p, ctx) if i["level"] == "accept"],
             confirmed=[c["id"] for c in core.confirmations(p)])
    b.update(kw)
    return b


# ---- profiles ----

def test_a_good_profile_validates_and_a_bad_one_says_why():
    assert core.validate(prof()) == []
    assert any("Hover height" in e for e in core.validate(prof(height_m=1.2)))     # the agent refuses > 1.05
    assert any("name" in e.lower() for e in core.validate(prof(name="")))
    assert any("Stage" in e for e in core.validate(prof(stage="orbit")))
    assert any("not known" in e for e in core.validate(prof(sysid=999)))
    assert core.validate(prof(height_m="abc"))


def test_profiles_save_list_and_archive_never_delete():
    saved = core.save_profile(dict(name="Hover 0.8 m!", height_m=0.8))
    assert saved["id"] == "hover-0-8-m"
    assert [p["id"] for p in core.list_profiles()] == ["hover-0-8-m"]
    assert core.delete_profile("hover-0-8-m")
    assert core.list_profiles() == []
    assert len(list((core.PROFILE_DIR / "deleted").glob("*.json"))) == 1
    with pytest.raises(ValueError):
        core.save_profile(dict(name="x", height_m=5))


def test_the_chart_follows_the_controllers_own_numbers():
    tl = core.timeline(prof(height_m=0.8, hold_s=5))
    assert tl["flies"] and tl["points"][0] == [0.0, 0.0]
    assert max(h for _, h in tl["points"]) == 0.8 and tl["points"][-1][1] == 0.0
    assert [m[0] for m in tl["marks"]] == ["spool", "climb", "hold", "descent", "landing"]
    # take-off stage only holds at the pause height
    assert max(h for _, h in core.timeline(prof(stage="takeoff", takeoff_pause_m=0.5))["points"]) == 0.5
    assert core.timeline(prof(stage="arm"))["flies"] is False


# ---- the command ----

def test_the_command_is_the_real_agents_and_never_cuts_motors():
    a = core.agent_argv(prof(), "/tmp/t.csv")
    assert a[:3] == ["python3", "tools/drone_agent.py", "offboard-flight"]
    for f in ("--real-aircraft", "--gate", "--allow-radio-flight", "--lift-by-climb", "--ground-attitude"):
        assert f in a
    assert a[a.index("--expect-sysid") + 1] == "113" and a[a.index("--height") + 1] == "0.8"
    assert "--allow-kill" not in a                          # DShot scaling unconfirmed (golden rule)
    assert "udpout://127.0.0.1:14540" in a


def test_usb_does_not_unlock_the_radio_and_bench_arm_has_no_flight_flags():
    usb = core.agent_argv(prof(link="usb"), "/tmp/t.csv")
    assert "--allow-radio-flight" not in usb and "serial:/dev/ttyACM0" in usb
    arm = core.agent_argv(prof(stage="arm", flight_type="bench"), "/tmp/t.csv")
    assert "--height" not in arm and "--allow-radio-flight" not in arm


# ---- the responsibility assessment ----

def ids(items, level):
    return {i["id"] for i in items if i["level"] == level}


def test_props_on_over_the_radio_lists_the_real_risks_to_accept():
    acc = ids(core.assess(prof(), GO), "accept")
    assert {"untested", "drift", "radio_loop", "gcs_failsafe"} <= acc
    assert "untethered" not in acc and "usb_cable" not in acc


def test_free_flight_and_usb_add_their_own_risks():
    assert "untethered" in ids(core.assess(prof(flight_type="free"), GO), "accept")
    assert "usb_cable" in ids(core.assess(prof(link="usb"), GO), "accept")
    assert "radio_loop" not in ids(core.assess(prof(link="usb"), GO), "accept")


def test_bench_has_nothing_to_accept_only_notes():
    items = core.assess(prof(flight_type="bench"), {})
    assert ids(items, "accept") == set() and ids(items, "block") == set()
    assert "bench" in ids(items, "note") and "handset" in ids(items, "note")


def test_radio_props_on_is_blocked_without_a_fresh_preflight_go():
    assert "preflight" in ids(core.assess(prof(), {}), "block")
    assert "preflight" in ids(core.assess(prof(), dict(preflight_go=False, preflight_age_s=5)), "block")
    assert "preflight" in ids(core.assess(prof(), dict(preflight_go=True, preflight_age_s=700)), "block")
    assert "preflight" not in ids(core.assess(prof(), GO), "block")
    assert "preflight" not in ids(core.assess(prof(link="usb"), {}), "block")      # radio quality is not USB's


# ---- authorise: responsibility is complete or it is nothing ----

def test_a_complete_statement_authorises():
    p = prof()
    assert core.authorise(p, GO, full_body(p)) == (True, "")


@pytest.mark.parametrize("change,word", [
    (dict(operator=""), "operator"), (dict(operator=" "), "operator"),
    (dict(aircraft="ARTEX115"), "ARTEX113"), (dict(phrase="yes"), "Type exactly"),
    (dict(accepted=[]), "ticked"), (dict(confirmed=[]), "confirmation"),
])
def test_anything_missing_refuses(change, word):
    p = prof()
    ok, why = core.authorise(p, GO, full_body(p, **change))
    assert not ok and word.lower() in why.lower()


def test_one_unticked_risk_refuses_and_names_it():
    p = prof()
    acc = [i for i in full_body(p)["accepted"] if i != "radio_loop"]
    ok, why = core.authorise(p, GO, full_body(p, accepted=acc))
    assert not ok and "radio_loop" in why


def test_responsibility_does_not_clear_a_block():
    p = prof()
    ok, why = core.authorise(p, {}, full_body(p, ctx={}))
    assert not ok and "Preflight" in why
    bad = prof(height_m=9)
    assert not core.authorise(bad, GO, full_body(bad))[0]


def test_bench_authorises_with_just_the_confirmations_and_the_typed_statement():
    p = prof(flight_type="bench", stage="takeoff")
    assert core.authorise(p, {}, full_body(p, ctx={}))[0]


# ---- tokens ----

def test_a_token_is_bound_to_the_exact_profile_and_expires():
    now = [1000.0]
    t = core.Tokens(now=lambda: now[0])
    p = prof()
    tok = t.issue(p, "J Leong", {"x": 1})
    assert t.get(tok, p)["operator"] == "J Leong"
    assert t.get(tok, prof(height_m=0.9)) is None           # edited after authorising
    assert t.get("nope", p) is None
    now[0] += core.TOKEN_TTL_S + 1
    assert t.get(tok, p) is None


# ---- records ----

def test_run_records_round_trip_and_make_a_history_block():
    p = prof()
    rec = dict(id="20261104-101500", started="2026-11-04 10:15:00", mode="live", profile=p, operator="J Leong",
               accepted=["radio_loop"], state="done",
               result=dict(outcome="takeover", passed=True, takeover_source="pilot_switch", takeover_after_s=7.2,
                           setpoints=dict(sent=400), link_losses=0, max_height_m=0.1))
    core.save_run(rec)
    assert core.list_runs()[0]["outcome"] == "takeover" and core.load_run("20261104-101500")["operator"] == "J Leong"
    assert core.load_run("../etc/passwd") is None
    h = core.history_block(rec)
    assert "J Leong" in h and "pilot_switch" in h and "runs/20261104-101500.json" in h
