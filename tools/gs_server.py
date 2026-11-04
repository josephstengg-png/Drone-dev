#!/usr/bin/env python3
"""Ground-station GUI server: mock 5 made real (4 Nov 2026). Standard library only.

    python3 tools/gs_server.py                 # LIVE: runs the agent on the companion over ssh
    python3 tools/gs_server.py --demo          # DEMO: a simulated aircraft, nothing is contacted
    then open http://127.0.0.1:8787

It is a front end over the existing tools and decides nothing about flight: profiles become a
`drone_agent.py offboard-flight` command (tools/gs_core.py), the agent runs where it always has
(on the Jetson, over ssh) and stops at READY, and the aircraft arms only when the page sends `ARM`
after the operator's recorded authorisation and a deliberate press. Preflight is tools/preflight.py;
the out-of-band abort is tools/takeover.py. The handset stays the real override.

Local only, on purpose. It binds 127.0.0.1, rejects any other Host or Origin (DNS rebinding) and
every POST must carry X-GS-Client (a web page in your browser cannot send that cross-origin).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gs_core as core  # noqa: E402
from swarm_deploy import host_argv  # noqa: E402

ROOT = core.ROOT
GUI = ROOT / "gui" / "index.html"
GATE_S = 150          # drone_agent --gate-timeout-s: how long READY waits for the operator's ARM
SEND_WORDS = {"ARM", "ABORT", "LAND", "HOLD"}
DEMO_WORDS = {"DEMO_TAKEOVER", "DEMO_KILL"}


class Run:
    """One agent process: its events, its log, its state, and the pipe the operator's words go down."""

    def __init__(self, rec: dict, argv: list[str], demo: bool):
        self.rec, self.argv, self.demo = rec, argv, demo
        self.feed: list[dict] = []
        self.lock = threading.Lock()
        self.state = "starting"
        self.stage = None
        self.t = dict(started=time.time(), ready=None, armed=None, done=None)
        self.result: dict | None = None
        self.arm_sent = False
        self.proc: subprocess.Popen | None = None

    def start(self) -> None:
        if self.demo:
            cmd = [sys.executable, str(ROOT / "tools" / "fake_agent.py")] + self.argv[2:]
        else:
            ac = core.AIRCRAFT[self.rec["profile"]["sysid"]]
            cmd, _ = host_argv(dict(host=ac["host"], remote_dir="~/FlightPlanner"), self.argv)
            cmd[1:1] = ["-o", "ConnectTimeout=8"]
        self.rec["command"] = cmd
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, bufsize=1, cwd=str(ROOT))
        threading.Thread(target=self._read, args=(self.proc.stdout, "event"), daemon=True).start()
        threading.Thread(target=self._read, args=(self.proc.stderr, "log"), daemon=True).start()
        threading.Thread(target=self._wait, daemon=True).start()

    def _push(self, kind: str, **kw) -> None:
        with self.lock:
            self.feed.append(dict(n=len(self.feed), t=round(time.time(), 2), kind=kind, **kw))

    def _read(self, pipe, kind: str) -> None:
        for line in pipe:
            line = line.rstrip("\n")
            if not line:
                continue
            if kind == "log":
                self._push("log", text=line)
                continue
            try:
                ev = json.loads(line)
            except ValueError:
                self._push("log", text=line)
                continue
            self._on_event(ev)

    def _on_event(self, ev: dict) -> None:
        if "verb" in ev:
            self.result = ev
            self.t["done"] = time.time()
            self.state = "done"
            self._push("result", **{"data": ev})
            return
        e = ev.get("event")
        if e == "ready":
            self.state, self.t["ready"] = "ready", time.time()
        elif e == "armed":
            self.state, self.t["armed"] = "armed", time.time()
        elif e == "stage":
            self.stage = ev.get("stage")
        elif e in ("rc_takeover", "takeover_classified", "still_armed"):
            if self.state != "done":
                self.state = "pilot"
        self._push("event", data=ev)

    def _wait(self) -> None:
        rc = self.proc.wait()
        time.sleep(0.3)     # let the readers drain
        if self.result is None:
            tail = [f["text"] for f in self.feed if f["kind"] == "log"][-1:]
            self.result = dict(error=f"agent exited with code {rc}" + (f": {tail[0]}" if tail else "")
                               + (" (ssh could not reach the companion)" if rc == 255 else ""), passed=False)
            self.state = "ended"
        self.t["done"] = self.t["done"] or time.time()
        self.rec.update(state=self.state, result=self.result, finished=time.strftime("%Y-%m-%d %H:%M:%S"),
                        feed=self.feed)
        core.save_run(self.rec)

    def send(self, word: str) -> bool:
        if not self.proc or self.proc.poll() is not None:
            return False
        try:
            self.proc.stdin.write(word + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError, ValueError):
            return False
        self._push("sent", text=word)
        return True

    @property
    def active(self) -> bool:
        return self.state not in ("done", "ended")

    def snapshot(self, since: int) -> dict:
        with self.lock:
            feed = self.feed[since:]
        return dict(id=self.rec["id"], state=self.state, stage=self.stage, t=self.t, result=self.result,
                    profile=self.rec["profile"], operator=self.rec["operator"], feed=feed, demo=self.demo,
                    gate_s=GATE_S, arm_sent=self.arm_sent, timeline=core.timeline(self.rec["profile"]),
                    speed=float(os.environ.get("DEMO_SPEED", "4")) if self.demo else 1.0)


class App:
    def __init__(self, demo: bool):
        self.demo = demo
        self.tokens = core.Tokens()
        self.run: Run | None = None
        self.pre: dict[int, dict] = {}
        self.lock = threading.Lock()

    def ctx(self, sysid: int) -> dict:
        p = self.pre.get(sysid)
        return dict(preflight_go=None if not p else p["go"], preflight_age_s=None if not p else time.time() - p["at"])

    def preflight(self, sysid: int) -> dict:
        ac = core.AIRCRAFT[sysid]
        if self.demo:
            rep = dict(all_go=True, aircraft=[dict(sysid=sysid, host=ac["host"], verdict="GO", blockers=[],
                       warnings=["DEMO: simulated result, nothing was contacted"],
                       ground=dict(loss_pct=0.0, avg_ms=0.4),
                       probe=dict(backpack_ping=dict(loss_pct=0.0), wifi=dict(signal_dbm=-50), relay_pids=[1],
                                  vehicle=dict(reachable=True, armed=False, battery_pct=88)))])
        else:
            try:
                r = subprocess.run([sys.executable, str(ROOT / "tools" / "preflight.py"), "--fleet", ac["fleet"],
                                    "--json"], capture_output=True, text=True, timeout=170, cwd=str(ROOT))
                rep = json.loads(r.stdout[r.stdout.index("{"):])
            except (subprocess.TimeoutExpired, ValueError) as exc:
                rep = dict(all_go=False, aircraft=[dict(sysid=sysid, host=ac["host"], verdict="NO-GO",
                           blockers=[f"preflight did not complete: {type(exc).__name__}"], warnings=[])])
        go = bool(rep.get("all_go"))
        self.pre[sysid] = dict(go=go, at=time.time(), report=rep)
        return dict(go=go, at=time.time(), report=rep)

    def abort_out_of_band(self, sysid: int) -> dict:
        if self.demo:
            return dict(ok=True, text="DEMO: out-of-band abort flag simulated")
        r = subprocess.run([sys.executable, str(ROOT / "tools" / "takeover.py"), "--fleet",
                            core.AIRCRAFT[sysid]["fleet"], "--now"], capture_output=True, text=True,
                           timeout=30, cwd=str(ROOT))
        return dict(ok=r.returncode == 0, text=(r.stderr or r.stdout).strip()[-400:])


def make_handler(app: App):
    class H(BaseHTTPRequestHandler):
        server_version = "gs/1"

        def log_message(self, *a):          # quiet
            pass

        def _send(self, code: int, body, ctype="application/json") -> None:
            data = body if isinstance(body, bytes) else (json.dumps(body, default=str) if ctype == "application/json"
                                                          else body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype + ("; charset=utf-8" if ctype != "application/json" else ""))
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _guard(self, post: bool) -> bool:
            port = self.server.server_address[1]
            allowed_hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
            if self.headers.get("Host", "") not in allowed_hosts:
                self._send(403, dict(error="bad Host"))
                return False
            if post:
                origin = self.headers.get("Origin")
                if origin and origin.split("://", 1)[-1] not in allowed_hosts:
                    self._send(403, dict(error="bad Origin"))
                    return False
                if self.headers.get("X-GS-Client") != "ui":
                    self._send(403, dict(error="missing X-GS-Client"))
                    return False
            return True

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            try:
                return json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return {}

        # ---- GET ----
        def do_GET(self):
            if not self._guard(False):
                return
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path in ("/", "/index.html"):
                return self._send(200, GUI.read_bytes(), "text/html")
            if u.path == "/api/state":
                return self._send(200, dict(demo=app.demo, phrase=core.PHRASE, aircraft=sorted(core.AIRCRAFT),
                                            stages=[[s, core.STAGE_NOTE[s]] for s in core.STAGES],
                                            defaults=core.DEFAULTS, height_max=core.HEIGHT_MAX_M, gate_s=GATE_S,
                                            preflight={k: dict(go=v["go"], age_s=time.time() - v["at"],
                                                               report=v["report"]) for k, v in app.pre.items()},
                                            run_active=bool(app.run and app.run.active),
                                            ttl_s=core.TOKEN_TTL_S))
            if u.path == "/api/profiles":
                return self._send(200, core.list_profiles())
            if u.path == "/api/run":
                if not app.run:
                    return self._send(200, dict(state="none"))
                return self._send(200, app.run.snapshot(int(q.get("since", ["0"])[0])))
            if u.path == "/api/runs":
                return self._send(200, core.list_runs())
            if u.path.startswith("/api/runs/"):
                r = core.load_run(u.path.rsplit("/", 1)[-1])
                return self._send(200 if r else 404, dict(record=r, history=core.history_block(r)) if r
                                  else dict(error="no such run"))
            self._send(404, dict(error="not found"))

        # ---- POST ----
        def do_POST(self):
            if not self._guard(True):
                return
            u, b = urlparse(self.path), self._body()
            try:
                return self._post(u.path, b)
            except (KeyError, ValueError, TypeError) as exc:
                return self._send(400, dict(error=f"{type(exc).__name__}: {exc}"))

        def _post(self, path: str, b: dict):
            if path == "/api/profiles":
                return self._send(200, core.save_profile(b))
            if path == "/api/profiles/delete":
                return self._send(200, dict(ok=core.delete_profile(str(b.get("id", "")))))
            if path == "/api/profiles/preview":
                p = core.normalise(b)
                return self._send(200, dict(profile=p, errors=core.validate(p),
                                            timeline=core.timeline(p) if not core.validate(p) else None,
                                            assessment=core.assess(p, app.ctx(p["sysid"])),
                                            confirmations=core.confirmations(p) if p["sysid"] in core.AIRCRAFT else [],
                                            argv=core.agent_argv(p, "/tmp/gs_trace.csv")
                                            if not core.validate(p) else None))
            if path == "/api/preflight":
                return self._send(200, app.preflight(int(b["sysid"])))
            if path == "/api/run/authorise":
                p = core.normalise(b["profile"])
                ok, why = core.authorise(p, app.ctx(p["sysid"]), b)
                if not ok:
                    return self._send(403, dict(error=why))
                texts = {c["id"]: c["text"] for c in core.confirmations(p)}
                rec = dict(operator=b["operator"].strip(), accepted=sorted(b["accepted"]),
                           confirmations=[texts[i] for i in sorted(b["confirmed"]) if i in texts],
                           phrase=core.PHRASE, aircraft_typed=b["aircraft"].strip().upper(),
                           authorised=time.strftime("%Y-%m-%d %H:%M:%S"))
                return self._send(200, dict(token=app.tokens.issue(p, b["operator"], rec), ttl_s=core.TOKEN_TTL_S))
            if path == "/api/run/start":
                p = core.normalise(b["profile"])
                tok = app.tokens.get(b.get("token"), p)
                if not tok:
                    return self._send(403, dict(error="Authorisation missing, expired or for a different profile."))
                with app.lock:
                    if app.run and app.run.active:
                        return self._send(409, dict(error="A run is already active. Only one agent may hold the link."))
                    rid = core.new_run_id()
                    rec = dict(id=rid, started=time.strftime("%Y-%m-%d %H:%M:%S"), mode="demo" if app.demo else "live",
                               profile=p, **tok["record"])
                    app.run = Run(rec, core.agent_argv(p, f"/tmp/gs_{rid}.csv"), app.demo)
                    app.run.start()
                return self._send(200, dict(id=rid))
            run = app.run
            if path == "/api/run/arm":
                if not run or run.state != "ready" or run.arm_sent:
                    return self._send(409, dict(error="The agent is not waiting at READY."))
                tok = app.tokens.get(b.get("token"), run.rec["profile"])
                if not tok or b.get("confirm") != "ARM":
                    return self._send(403, dict(error="Authorisation expired or ARM not confirmed. Authorise again."))
                run.arm_sent = True
                app.tokens.revoke(b["token"])
                run.rec["arm_requested"] = time.strftime("%Y-%m-%d %H:%M:%S")
                return self._send(200, dict(ok=run.send("ARM")))
            if path == "/api/run/word":
                w = str(b.get("word", "")).upper()
                if not run or not run.active:
                    return self._send(409, dict(error="No active run."))
                if w not in SEND_WORDS - {"ARM"} and not (app.demo and w in DEMO_WORDS):
                    return self._send(400, dict(error=f"{w} is not sent from here."))
                return self._send(200, dict(ok=run.send(w)))
            if path == "/api/run/abort_flag":
                sysid = int(run.rec["profile"]["sysid"]) if run else int(b["sysid"])
                return self._send(200, app.abort_out_of_band(sysid))
            self._send(404, dict(error="not found"))

    return H


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="simulated aircraft; contacts nothing")
    ap.add_argument("--port", type=int, default=8787)
    a = ap.parse_args()
    app = App(a.demo)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(app))
    print(f"[gs] {'DEMO (simulated aircraft)' if a.demo else 'LIVE (real aircraft over ssh)'} -- "
          f"http://127.0.0.1:{a.port}", file=sys.stderr, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        if app.run and app.run.active:
            print("[gs] a run is still active; its agent keeps going on the companion.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
