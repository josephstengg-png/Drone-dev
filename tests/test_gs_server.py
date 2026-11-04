"""The ground-station server, in DEMO mode (a simulated aircraft; nothing is contacted)."""
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
import gs_server as srv  # noqa: E402


@pytest.fixture()
def gs(tmp_path, monkeypatch):
    monkeypatch.setattr(core, "PROFILE_DIR", tmp_path / "profiles")
    monkeypatch.setattr(core, "RUN_DIR", tmp_path / "runs")
    monkeypatch.setenv("DEMO_SPEED", "40")
    app = srv.App(demo=True)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv.make_handler(app))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    port = httpd.server_address[1]

    def call(method, path, body=None, headers=None, host=None):
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        h = {"Host": host or f"127.0.0.1:{port}"}
        if method == "POST":
            h["X-GS-Client"] = "ui"
            h["Content-Type"] = "application/json"
        h.update(headers or {})
        c.request(method, path, json.dumps(body) if body is not None else None, h)
        r = c.getresponse()
        data = r.read()
        c.close()
        try:
            return r.status, json.loads(data)
        except ValueError:
            return r.status, data

    call.app = app
    yield call
    httpd.shutdown()


def wait(call, state, timeout=20):
    end = time.time() + timeout
    while time.time() < end:
        _, r = call("GET", "/api/run")
        if r.get("state") in (state if isinstance(state, tuple) else (state,)):
            return r
        time.sleep(0.1)
    raise AssertionError(f"never reached {state}: {call('GET', '/api/run')[1].get('state')}")


def authorise(call, p, **kw):
    prev = call("POST", "/api/profiles/preview", p)[1]
    body = dict(profile=p, operator="J Leong", aircraft=f"ARTEX{p['sysid']}", phrase=core.PHRASE,
                accepted=[i["id"] for i in prev["assessment"] if i["level"] == "accept"],
                confirmed=[c["id"] for c in prev["confirmations"]])
    body.update(kw)
    return call("POST", "/api/run/authorise", body)


BENCH = dict(name="t bench", sysid=113, flight_type="bench", link="radio", stage="takeoff", wait_s=15, height_m=0.8)


def test_the_page_and_state_are_served(gs):
    s, page = gs("GET", "/")
    assert s == 200 and b"Ground Station" in page
    s, st = gs("GET", "/api/state")
    assert st["demo"] is True and st["phrase"] == core.PHRASE and 113 in st["aircraft"]


def test_other_hosts_and_foreign_posts_are_refused(gs):
    assert gs("GET", "/api/state", host="evil.example:80")[0] == 403            # DNS rebinding
    s, _ = gs("POST", "/api/run/word", {"word": "ABORT"}, headers={"X-GS-Client": ""})
    assert s == 403
    assert gs("POST", "/api/run/word", {"word": "ABORT"}, headers={"Origin": "http://evil.example"})[0] == 403


def test_a_profile_can_be_created_previewed_and_removed(gs):
    s, saved = gs("POST", "/api/profiles", dict(BENCH, name="Mine"))
    assert s == 200 and saved["id"] == "mine"
    s, prev = gs("POST", "/api/profiles/preview", dict(BENCH, height_m=0.9, link="usb", flight_type="tether"))
    assert prev["errors"] == [] and "usb_cable" in {i["id"] for i in prev["assessment"]}
    assert prev["argv"][prev["argv"].index("--height") + 1] == "0.9"
    assert gs("POST", "/api/profiles", dict(BENCH, name="bad", height_m=9))[0] == 400
    assert gs("POST", "/api/profiles/delete", {"id": "mine"})[1]["ok"]


def test_nothing_starts_without_a_complete_authorisation(gs):
    p = dict(BENCH, flight_type="tether")
    s, r = authorise(gs, p, phrase="ok")
    assert s == 403
    assert gs("POST", "/api/run/start", dict(profile=p, token="nope"))[0] == 403
    assert gs("GET", "/api/run")[1]["state"] == "none"


def test_radio_props_on_needs_a_fresh_preflight_before_it_can_be_authorised(gs):
    p = dict(BENCH, flight_type="tether", stage="disarm")
    s, r = authorise(gs, p)
    assert s == 403 and "Preflight" in r["error"]
    assert gs("POST", "/api/preflight", {"sysid": 113})[1]["go"] is True
    s, r = authorise(gs, p)
    assert s == 200 and r["token"]


def test_a_full_demo_run_arms_only_on_the_operators_press_and_leaves_a_record(gs):
    s, a = authorise(gs, BENCH)
    assert s == 200
    tok = a["token"]
    assert gs("POST", "/api/run/start", dict(profile=BENCH, token=tok))[0] == 200
    r = wait(gs, "ready")
    assert not r["arm_sent"] and r["t"]["armed"] is None            # the agent is waiting, not arming
    assert gs("POST", "/api/run/start", dict(profile=BENCH, token=tok))[0] == 409   # one agent on the link
    assert gs("POST", "/api/run/arm", dict(token=tok))[0] == 403                    # ARM must be confirmed
    assert gs("POST", "/api/run/arm", dict(token="stolen", confirm="ARM"))[0] == 403
    assert gs("POST", "/api/run/arm", dict(token=tok, confirm="ARM"))[1]["ok"]
    wait(gs, "armed")
    assert gs("POST", "/api/run/arm", dict(token=tok, confirm="ARM"))[0] == 409     # not twice
    assert gs("POST", "/api/run/word", dict(word="DEMO_TAKEOVER"))[1]["ok"]
    done = wait(gs, ("done", "ended"))
    assert done["result"]["outcome"] == "takeover" and done["result"]["takeover_source"] == "pilot_switch"
    time.sleep(0.5)
    runs = gs("GET", "/api/runs")[1]
    assert runs[0]["operator"] == "J Leong" and runs[0]["outcome"] == "takeover" and runs[0]["mode"] == "demo"
    s, rec = gs("GET", f"/api/runs/{runs[0]['id']}")
    assert rec["record"]["phrase"] == core.PHRASE and rec["record"]["aircraft_typed"] == "ARTEX113"
    assert "J Leong" in rec["history"]


def test_the_token_stops_working_if_the_profile_is_edited_after_authorising(gs):
    s, a = authorise(gs, BENCH)
    edited = dict(BENCH, wait_s=30)
    assert gs("POST", "/api/run/start", dict(profile=edited, token=a["token"]))[0] == 403


def test_the_operator_can_abort_a_running_demo_and_demo_words_are_demo_only(gs):
    s, a = authorise(gs, dict(BENCH, stage="disarm", flight_type="bench"))
    p = dict(BENCH, stage="disarm")
    gs("POST", "/api/run/start", dict(profile=p, token=a["token"]))
    wait(gs, "ready")
    gs("POST", "/api/run/arm", dict(token=a["token"], confirm="ARM"))
    wait(gs, "armed")
    assert gs("POST", "/api/run/word", dict(word="KILL"))[0] == 400          # KILL is never sent from here
    assert gs("POST", "/api/run/word", dict(word="ABORT"))[1]["ok"]
    done = wait(gs, ("done", "ended"))
    assert done["result"]["outcome"] == "guard_acted"


def test_a_live_run_is_an_ssh_to_the_aircrafts_own_companion_and_never_arms_itself(tmp_path, monkeypatch):
    """LIVE mode builds the same agent command over ssh. Popen is captured: nothing is contacted."""
    monkeypatch.setattr(core, "RUN_DIR", tmp_path / "runs")
    seen = {}

    class FakePopen:
        def __init__(self, cmd, **kw):
            seen["cmd"] = cmd
            self.stdout = self.stderr = iter(())
            self.stdin = None

        def wait(self):
            return 255

        def poll(self):
            return 255

    monkeypatch.setattr(srv.subprocess, "Popen", FakePopen)
    p = core.normalise(dict(name="Hover radio", flight_type="tether", link="radio", stage="disarm", height_m=0.8))
    run = srv.Run(dict(id="x", profile=p, operator="J Leong"), core.agent_argv(p, "/tmp/t.csv"), demo=False)
    run.start()
    time.sleep(0.5)
    cmd = seen["cmd"]
    assert cmd[0] == "ssh" and "jetson-2" in cmd and "BatchMode=yes" in cmd
    remote = cmd[-1]
    assert remote.startswith("cd ~/FlightPlanner && python3 tools/drone_agent.py offboard-flight")
    assert "--expect-sysid 113" in remote and "--gate" in remote and "--allow-kill" not in remote
    assert "ARM" not in remote                       # arming is only ever the operator's word on stdin
    assert run.state == "ended" and "ssh could not reach" in run.result["error"]      # 255 is said plainly
