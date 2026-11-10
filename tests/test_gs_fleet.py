"""Multi-aircraft waypoint runs in the ground station (10 Nov 2026): the pure core, then the server in DEMO mode."""
import http.client
import json
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import gs_core as core  # noqa: E402
import gs_fleet as fleet  # noqa: E402
import gs_server as srv  # noqa: E402

P1 = "missions/seletar/mission_drones/drone-1.plan"
P2 = "missions/seletar/mission_drones/drone-2.plan"
OTHER = next((str(p.relative_to(core.ROOT)) for p in sorted((core.ROOT / "missions").glob("**/drone-1.plan"))
              if "seletar" not in str(p)), None)


def prof(**kw):
    base = dict(name="Seletar pair", mode="mission", flight_type="props_on",
                aircraft=[dict(sysid=113, plan=P2, link="radio"), dict(sysid=115, plan=P1, link="radio")])
    base.update(kw)
    return fleet.normalise(base)


GO = {113: dict(preflight_go=True, preflight_age_s=30.0), 115: dict(preflight_go=True, preflight_age_s=30.0)}


# ---- profiles and validation ------------------------------------------------------------

def test_a_seletar_pair_is_valid_and_its_agents_are_resident_prepares():
    p = prof()
    assert fleet.validate(p) == []
    argv = fleet.agent_argv(p, p["aircraft"][0], "gsX")
    assert argv[:3] == ["python3", "tools/drone_agent.py", "prepare"]
    assert "--resident" in argv and argv[argv.index("--expect-sysid") + 1] == "113"
    assert argv[argv.index("--plan") + 1] == P2 and "--allow-kill" not in argv


def test_the_rules_that_refuse_a_fleet_profile():
    assert any("two aircraft" in e for e in fleet.validate(prof(aircraft=[dict(sysid=113, plan=P2)])))
    assert any("own plan" in e for e in fleet.validate(prof(aircraft=[dict(sysid=113, plan=P1), dict(sysid=115, plan=P1)])))
    assert any("props-off bench" in e for e in fleet.validate(prof(flight_type="bench")))
    assert any("Arm hold" in e for e in fleet.validate(prof(arm_hold_s=30)))
    assert any("choose a plan" in e for e in fleet.validate(prof(aircraft=[dict(sysid=113, plan="/etc/passwd"),
                                                                         dict(sysid=115, plan=P1)])))


@pytest.mark.skipif(OTHER is None, reason="needs a second planning run under missions/")
def test_plans_from_different_planning_runs_cannot_fly_together_but_may_be_uploaded():
    mixed = [dict(sysid=113, plan=P2), dict(sysid=115, plan=OTHER)]
    assert any("different planning runs" in e for e in fleet.validate(prof(aircraft=mixed)))
    assert fleet.validate(prof(aircraft=mixed, mode="upload", flight_type="bench")) == []


# ---- responsibility -----------------------------------------------------------------------

def test_props_on_over_the_radio_needs_a_fresh_preflight_go_on_every_aircraft():
    blocks = [i for i in fleet.assess(prof(), {113: GO[113]}) if i["level"] == "block"]
    assert blocks and "ARTEX115" in blocks[0]["text"] and "ARTEX113" not in blocks[0]["text"]
    assert not [i for i in fleet.assess(prof(), GO) if i["level"] == "block"]
    accepts = {i["id"] for i in fleet.assess(prof(), GO) if i["level"] == "accept"}
    assert {"multi_untested", "hold_position", "separation"} <= accepts
    assert not [i for i in fleet.assess(prof(mode="upload", flight_type="bench"), {}) if i["level"] != "note"]


def body(p, **kw):
    b = dict(operator="J Leong", pilots={"113": "Ann", "115": "Ben"}, aircraft="ARTEX115, artex113", phrase=core.PHRASE,
             accepted=[i["id"] for i in fleet.assess(p, GO) if i["level"] == "accept"],
             confirmed=[c["id"] for c in fleet.confirmations(p)])
    b.update(kw)
    return b


def test_one_named_pilot_per_aircraft_and_every_aircraft_typed():
    p = prof()
    assert fleet.authorise(p, GO, body(p)) == (True, "")
    assert "same person" in fleet.authorise(p, GO, body(p, pilots={"113": "Ann", "115": "ann"}))[1]
    assert "ARTEX115" in fleet.authorise(p, GO, body(p, pilots={"113": "Ann"}))[1]
    assert "Type every aircraft" in fleet.authorise(p, GO, body(p, aircraft="ARTEX113"))[1]
    assert "ticked" in fleet.authorise(p, GO, body(p, accepted=[]))[1]


# ---- reading the agents -----------------------------------------------------------------

def test_what_counts_as_a_fault_after_start():
    mission = dict(armed=True, main_mode=4, sub_mode=4, waypoint=3)
    assert fleet.member_fault(mission, 11, 0) is None
    assert fleet.member_fault(dict(mission, armed=False, _was_rtl=True), 11, 0) is None        # landed after RTL
    assert fleet.member_fault(dict(mission, armed=False, waypoint=10), 11, 0) is None           # reached the last item
    assert "mid-mission" in fleet.member_fault(dict(mission, armed=False), 11, 0)
    assert "ALTITUDE" in fleet.member_fault(dict(mission, main_mode=2), 11, 0)                 # pilot took over
    assert "link lost" in fleet.member_fault(mission, 11, fleet.STATUS_MISSES_FAULT)


def test_others_hold_skips_the_faulted_the_held_and_the_finished():
    flying = dict(status=dict(armed=True, main_mode=4, sub_mode=4), started=True)
    members = {113: dict(flying), 115: dict(flying), 117: dict(flying, held=True), 119: dict(flying, finished=True),
               121: dict(status=dict(armed=True, main_mode=4, sub_mode=5), started=True)}         # already on RTL
    assert fleet.hold_targets(members, 113) == [115]


def test_the_history_block_reads_a_fleet_record():
    rec = dict(id="20261110-120000", kind="fleet", started="2026-11-10 12:00:00", mode="demo", profile=prof(),
               operator="J", pilots={"113": "Ann", "115": "Ben"}, accepted=["separation"],
               result=dict(outcome="every aircraft completed its mission", passed=True,
                           aircraft={"113": dict(ready=True, armed=False, started=True, held=False, fault=None)}))
    text = core.history_block(rec)
    assert "fleet run" in text and "ARTEX113 Ann" in text and "ARTEX115" in text


# ---- the server, DEMO ---------------------------------------------------------------------

@pytest.fixture()
def gs(tmp_path, monkeypatch):
    monkeypatch.setattr(core, "PROFILE_DIR", tmp_path / "profiles")
    monkeypatch.setattr(core, "RUN_DIR", tmp_path / "runs")
    monkeypatch.setattr(fleet, "FLEET_DIR", tmp_path / "profiles" / "fleet")
    monkeypatch.setenv("DEMO_SPEED", "40")
    app = srv.App(demo=True)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv.make_handler(app))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    port = httpd.server_address[1]

    def call(method, path, b=None):
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        h = {"Host": f"127.0.0.1:{port}"}
        if method == "POST":
            h.update({"X-GS-Client": "ui", "Content-Type": "application/json"})
        c.request(method, path, json.dumps(b) if b is not None else None, h)
        r = c.getresponse()
        data = json.loads(r.read())
        c.close()
        return r.status, data

    call.app = app
    for s in (113, 115):
        call("POST", "/api/preflight", dict(sysid=s))
    yield call
    httpd.shutdown()


def wait(call, states, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        r = call("GET", "/api/fleet")[1]
        if r.get("state") in states:
            return r
        time.sleep(0.1)
    raise AssertionError(f"never reached {states}: {call('GET', '/api/fleet')[1].get('state')}")


def start(call, p):
    s, a = call("POST", "/api/fleet/authorise", dict(profile=p, **body(p)))
    assert s == 200, a
    tok = a["token"]
    assert call("POST", "/api/fleet/start", dict(profile=p, token=tok))[0] == 200
    return tok


def test_two_gates_then_a_fault_on_one_aircraft_holds_the_other(gs):
    p = prof()
    tok = start(gs, p)
    wait(gs, ("ready",))
    assert gs("POST", "/api/fleet/start_all", dict(token=tok, confirm="START ALL"))[0] == 409   # gate 2 before gate 1
    assert gs("POST", "/api/fleet/arm_all", dict(token=tok, confirm="ARM"))[0] == 403            # wrong confirmation
    assert gs("POST", "/api/fleet/arm_all", dict(token=tok, confirm="ARM ALL"))[0] == 200
    wait(gs, ("armed",))
    assert gs("POST", "/api/fleet/start_all", dict(token=tok, confirm="START ALL"))[0] == 200
    wait(gs, ("flying",))
    assert gs("POST", "/api/fleet/word", dict(word="DEMO_TAKEOVER", target="115"))[1]["sent"] == [115]
    r = wait(gs, ("fault",))
    m = {x["sysid"]: x for x in r["members"]}
    assert m[115]["fault"] and m[113]["held"] and not m[113]["fault"]
    gs("POST", "/api/fleet/word", dict(word="RTL", target="all"))
    r = wait(gs, ("done", "ended"), 40)
    assert "faults on ARTEX115" in r["result"]["outcome"]
    rec = core.load_run(r["id"])
    assert rec["kind"] == "fleet" and rec["pilots"] == {"113": "Ann", "115": "Ben"} and rec["arm_spread_s"] is not None


def test_a_missed_start_window_disarms_everyone(gs, monkeypatch):
    p = prof()
    tok = start(gs, p)
    wait(gs, ("ready",))
    monkeypatch.setattr(fleet, "START_WINDOW_S", 0.5)       # after authorising: the profile's arm hold must fit the window
    gs("POST", "/api/fleet/arm_all", dict(token=tok, confirm="ARM ALL"))
    r = wait(gs, ("done", "ended"))
    assert r["result"]["outcome"] == "not started -- disarmed"
    assert not any(x["armed"] for x in r["members"])


def test_one_aircraft_not_ready_means_nobody_arms(gs, monkeypatch):
    real = fleet.agent_argv
    monkeypatch.setattr(fleet, "agent_argv", lambda p, m, rid: real(p, m, rid) + (["--demo-not-ready"]
                                                                                   if m["sysid"] == 115 else []))
    tok = start(gs, prof())
    r = wait(gs, ("done", "ended"))
    assert r["state"] == "ended" and "NOBODY armed" in r["result"]["outcome"]
    assert gs("POST", "/api/fleet/arm_all", dict(token=tok, confirm="ARM ALL"))[0] == 409


def test_upload_mode_never_arms_and_a_single_run_cannot_start_meanwhile(gs):
    p = prof(mode="upload", flight_type="bench")
    start(gs, p)
    assert gs("GET", "/api/fleet")[1]["state"] in ("starting", "preparing")
    sp = dict(name="one", sysid=113, flight_type="bench", link="radio", stage="arm")
    assert gs("POST", "/api/run/start", dict(profile=sp, token="x"))[0] in (403, 409)
    r = wait(gs, ("done", "ended"))
    assert r["result"]["outcome"] == "missions uploaded and verified"
    assert not any(x["armed"] for x in r["members"])
