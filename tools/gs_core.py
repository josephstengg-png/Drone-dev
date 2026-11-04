"""Ground-station core: mission profiles, the responsibility assessment, the agent command line
and run records. Pure functions and files, no hardware, so tests/test_gs_core.py pins all of it.
tools/gs_server.py serves it and gui/index.html shows it (mock 5 -> the real GUI, 4 Nov 2026).

WHAT "THE OPERATOR BEARS THE RESPONSIBILITY" MEANS HERE (operator's request, 4 Nov 2026)
  - A risk the software cannot remove but a human may knowingly accept is an ACCEPT item. The
    operator ticks each one, names themselves, types the aircraft and the phrase, and the run
    record keeps all of it. Nothing else stands between the profile and the aircraft.
  - A BLOCK item is not a risk, it is a missing fact the run cannot start without (invalid
    profile, no recent preflight GO on the radio link). Responsibility does not clear it.
  - Not in anyone's gift, so not here: the agent's own refusals (wrong sysid, already armed), the
    typed ARM at READY, the guard running WITHOUT --allow-kill, and the handset. The pilot's
    ch5 / kill are the authority and nothing in this file touches them.
"""

from __future__ import annotations

import hashlib
import json
import re
import secrets
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
import offboard_control as oc  # noqa: E402

PROFILE_DIR = ROOT / "profiles"
RUN_DIR = ROOT / "runs"

PHRASE = "I TAKE RESPONSIBILITY FOR THIS RUN"
TOKEN_TTL_S = 15 * 60
PREFLIGHT_MAX_AGE_S = 10 * 60

STAGES = oc.STAGES
FLIGHT_STAGES = oc.FLIGHT_STAGES
STAGE_NOTE = {
    "prearm": "Offboard selected and streaming, disarmed (T1)",
    "arm": "... and armed, props spooling at idle (T2)",
    "takeoff": "... and lifting to the take-off pause height (T3)",
    "hold": "... and holding at the hover height",
    "descent": "... and descending slowly",
    "landing": "... and landing",
    "disarm": "The whole flight: take off, hover, land, disarm (the full hover test)",
}
# drone_agent.py caps a real hover at 1.05 m unless --max-height-m raises it, to 2.0 m at most (4 Nov 2026,
# endurance hover). The profile may ask for up to 2.0 m; the command then carries --max-height-m.
HEIGHT_DEFAULT_CAP_M = 1.05
HEIGHT_MAX_M = 2.0
HOLD_MAX_S = 7200.0
AIRCRAFT = {113: dict(host="jetson-2", radio="udpout://127.0.0.1:14540", usb="serial:/dev/ttyACM0",
                      fleet="fleet-artex113.yaml", hover=0.42),
            115: dict(host="jetson-1", radio="udpout://127.0.0.1:14540", usb="serial:/dev/ttyACM0",
                      fleet="fleet-artex115.yaml", hover=0.42)}

DEFAULTS = dict(name="", description="", sysid=113, link="radio", flight_type="bench", stage="takeoff",
                height_m=0.8, hold_s=5.0, wait_s=15.0, takeoff_pause_m=0.5,
                test_descent_rate=0.08, final_descent_rate=0.05,
                lift_by_climb=True, ground_attitude=True, height_timeout_s=None,
                battery_hold_pct=None, battery_land_pct=None, setpoint_hz=None)
FLIGHT_TYPES = ("bench", "tether", "free")


# ---- profiles ---------------------------------------------------------------------------

def slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")[:60]


def normalise(raw: dict) -> dict:
    """Fill defaults, coerce types, drop unknown keys. Never raises on bad values (validate does)."""
    p = dict(DEFAULTS)
    for k in DEFAULTS:
        if k in raw and raw[k] is not None and raw[k] != "":
            p[k] = raw[k]
    for k in ("height_m", "hold_s", "wait_s", "takeoff_pause_m", "test_descent_rate", "final_descent_rate"):
        try:
            p[k] = float(p[k])
        except (TypeError, ValueError):
            p[k] = float("nan")
    try:
        p["sysid"] = int(p["sysid"])
    except (TypeError, ValueError):
        p["sysid"] = -1
    if p["height_timeout_s"] is not None:
        try:
            p["height_timeout_s"] = float(p["height_timeout_s"])
        except (TypeError, ValueError):
            p["height_timeout_s"] = float("nan")
    for k in ("battery_hold_pct", "battery_land_pct", "setpoint_hz"):
        if p[k] is not None:
            try:
                p[k] = float(p[k])
            except (TypeError, ValueError):
                p[k] = float("nan")
    for k in ("lift_by_climb", "ground_attitude"):
        p[k] = bool(p[k])
    p["name"] = str(p["name"]).strip()
    p["description"] = str(p["description"]).strip()
    return p


def validate(p: dict) -> list[str]:
    """Reasons this profile cannot be run as written. Empty = runnable."""
    e = []
    if len(slug(p["name"])) < 2:
        e.append("Give the profile a name (letters or digits).")
    if p["sysid"] not in AIRCRAFT:
        e.append(f"Aircraft {p['sysid']} is not known here (ARTEX{', ARTEX'.join(map(str, AIRCRAFT))}).")
    if p["link"] not in ("radio", "usb"):
        e.append("Link must be radio (ELRS) or usb.")
    if p["flight_type"] not in FLIGHT_TYPES:
        e.append("Flight type must be bench, tether or free.")
    if p["stage"] not in STAGES:
        e.append(f"Stage must be one of {', '.join(STAGES)}.")
    # the window only means something up to the stage that uses it (prearm, arm, take-off pause, hold); beyond that
    # the agent is given the number but never reads it, so a stale value must not stop a full-flight profile
    uses_window = p["stage"] in ("prearm", "arm", "takeoff", "hold")
    for k, lo, hi, label in (("height_m", 0.3, HEIGHT_MAX_M, "Hover height"),
                             ("hold_s", 0.0, HOLD_MAX_S, "Hold time"),
                             ("wait_s", 1.0, HOLD_MAX_S, "Takeover window"),
                             ("takeoff_pause_m", 0.2, HEIGHT_MAX_M, "Take-off pause height"),
                             ("test_descent_rate", 0.03, 0.5, "Test descent rate"),
                             ("final_descent_rate", 0.03, 0.5, "Final descent rate")):
        v = p[k]
        if k == "wait_s" and not uses_window:
            continue
        if not (v == v) or not lo <= v <= hi:
            e.append(f"{label} must be between {lo:g} and {hi:g}"
                     f"{' m' if k.endswith('_m') else ' s' if k.endswith('_s') else ' m/s'}.")
    if p["height_timeout_s"] is not None and not (0.3 <= p["height_timeout_s"] <= 10):
        e.append("Height timeout must be 0.3 to 10 s, or blank for automatic.")
    if p["setpoint_hz"] is not None and not (p["setpoint_hz"] == p["setpoint_hz"] and 1 <= p["setpoint_hz"] <= 50):
        e.append("Setpoint rate must be 1 to 50 Hz, or blank for the agent's default (3 over the radio, 20 over USB).")
    bh, bl = p["battery_hold_pct"], p["battery_land_pct"]
    for v, label in ((bh, "Guard battery hold"), (bl, "Guard battery land")):
        if v is not None and not (v == v and 0 <= v <= 60):
            e.append(f"{label} must be 0 to 60 %, or blank for the default.")
    if bh is not None and bl is not None and bh == bh and bl == bl and bh < bl:
        e.append("Guard battery hold must not be below the land threshold.")
    return e


def profile_hash(p: dict) -> str:
    return hashlib.sha256(json.dumps(p, sort_keys=True, default=str).encode()).hexdigest()[:16]


def list_profiles() -> list[dict]:
    PROFILE_DIR.mkdir(exist_ok=True)
    out = []
    for f in sorted(PROFILE_DIR.glob("*.json")):
        try:
            out.append(dict(normalise(json.loads(f.read_text())), id=f.stem))
        except (OSError, ValueError):
            continue
    return out


def save_profile(raw: dict) -> dict:
    p = normalise(raw)
    errs = validate(p)
    if errs:
        raise ValueError(" ".join(errs))
    PROFILE_DIR.mkdir(exist_ok=True)
    (PROFILE_DIR / f"{slug(p['name'])}.json").write_text(json.dumps(p, indent=1) + "\n")
    return dict(p, id=slug(p["name"]))


def delete_profile(pid: str) -> bool:
    f = PROFILE_DIR / f"{slug(pid)}.json"
    if f.exists():
        ARCHIVE = PROFILE_DIR / "deleted"
        ARCHIVE.mkdir(exist_ok=True)
        f.rename(ARCHIVE / f"{f.stem}-{int(time.time())}.json")      # archive, never delete
        return True
    return False


# ---- the planned flight, for the chart and the progress marker ---------------------------

def timeline(p: dict) -> dict:
    """Planned height against time from ARM, from the controller's own constants. An ESTIMATE:
    the real climb is gentler (accel_max 1 m/s^2) and take-off waits for lift-off."""
    lim = oc.Limits()
    prof = oc.stage_profile(p["stage"], height_m=p["height_m"], wait_s=p["wait_s"], hold_s=p["hold_s"],
                            takeoff_pause_m=p["takeoff_pause_m"], test_descent_rate_ms=p["test_descent_rate"],
                            final_descent_rate_ms=p["final_descent_rate"]) \
        if p["stage"] in FLIGHT_STAGES else None
    if prof is None:
        return dict(points=[[0, 0], [p["wait_s"], 0]], marks=[], total_s=p["wait_s"], flies=False)
    h = prof["target_height_m"]
    t = lim.spool_s
    pts = [[0.0, 0.0], [t, 0.0]]
    marks = [["spool", 0.0, t]]
    t_up = h / lim.climb_rate_ms
    pts.append([t + t_up, h])
    marks.append(["climb", t, t + t_up])
    t += t_up
    pts.append([t + prof["hold_s"], h])
    marks.append(["hold", t, t + prof["hold_s"]])
    t += prof["hold_s"]
    zone = min(lim.final_zone_m, h)
    if h > zone:
        t_d = (h - zone) / prof["descent_rate_ms"]
        pts.append([t + t_d, zone])
        marks.append(["descent", t, t + t_d])
        t += t_d
    t_f = zone / prof["final_descent_rate_ms"]
    pts.append([t + t_f, 0.0])
    marks.append(["landing", t, t + t_f])
    t += t_f
    return dict(points=[[round(a, 2), round(b, 3)] for a, b in pts],
                marks=[[n, round(a, 2), round(b, 2)] for n, a, b in marks], total_s=round(t, 1), flies=True,
                under_test=p["stage"])


# ---- the command the agent is started with ----------------------------------------------

def agent_argv(p: dict, trace: str) -> list[str]:
    """drone_agent.py offboard-flight, exactly as it would be typed. Deliberately NEVER --allow-kill:
    the guard may command LAND but must not cut the motors in the air (DShot scaling unconfirmed)."""
    ac = AIRCRAFT[p["sysid"]]
    flies = p["stage"] in FLIGHT_STAGES
    a = ["python3", "tools/drone_agent.py", "offboard-flight", "--link", ac[p["link"]],
         "--expect-sysid", str(p["sysid"]), "--real-aircraft", "--gate", "--stage", p["stage"],
         "--wait-s", f"{p['wait_s']:g}", "--gate-timeout-s", "150"]
    if p["setpoint_hz"] is not None:
        a += ["--setpoint-hz", f"{p['setpoint_hz']:g}"]
    if flies:
        a += ["--height", f"{p['height_m']:g}", "--hold-s", f"{p['hold_s']:g}",
              "--takeoff-pause-m", f"{p['takeoff_pause_m']:g}",
              "--test-descent-rate", f"{p['test_descent_rate']:g}",
              "--final-descent-rate", f"{p['final_descent_rate']:g}"]
        if p["height_m"] > HEIGHT_DEFAULT_CAP_M:
            a += ["--max-height-m", f"{HEIGHT_MAX_M:g}"]
        for flag, k in (("--battery-hold-pct", "battery_hold_pct"), ("--battery-land-pct", "battery_land_pct")):
            if p[k] is not None:
                a += [flag, f"{p[k]:g}"]
        if p["lift_by_climb"]:
            a.append("--lift-by-climb")
        if p["ground_attitude"]:
            a.append("--ground-attitude")
        if p["link"] == "radio":
            a += ["--allow-radio-flight", "--height-timeout-s", f"{p['height_timeout_s'] or 3:g}"]
        elif p["height_timeout_s"]:
            a += ["--height-timeout-s", f"{p['height_timeout_s']:g}"]
    a += ["--trace", trace]
    return a


# ---- the assessment: what is being accepted ----------------------------------------------

def confirmations(p: dict) -> list[dict]:
    n = f"ARTEX{p['sysid']}"
    if p["flight_type"] == "bench":
        c = [f"Propellers are OFF on {n}", "Hands and tools are clear of the rotors"]
    else:
        c = [f"Propellers are ON on {n} and checked tight"]
        if p["flight_type"] == "tether":
            c += ["The tether is tied to something FIXED and stops the aircraft below the ceiling",
                  "The area is clear of anyone not on the crew"]
        else:
            c += ["The area, including overhead, is clear of anyone not on the crew",
                  "The landing area and what is around it have been walked"]
        c += ["A named pilot holds the handset: ch5 (takeover) and ch8 (kill) within reach",
              "ch7 is at CENTRE (Altitude), ch8 kill is off, throttle is centred"]
        if p["link"] == "usb":
            c.append("The USB cable is plugged in with slack and strain relief")
    return [dict(id=f"c{i}", text=t) for i, t in enumerate(c)]


def assess(p: dict, ctx: dict) -> list[dict]:
    """ctx: preflight_go (bool|None), preflight_age_s (float|None). Items are level block|accept|note."""
    items = []

    def add(i, level, title, text):
        items.append(dict(id=i, level=level, title=title, text=text))

    flies = p["stage"] in FLIGHT_STAGES
    props_on = p["flight_type"] != "bench"
    radio = p["link"] == "radio"
    for msg in validate(p):
        add("invalid", "block", "Profile is not runnable", msg)
    if radio and props_on:
        age = ctx.get("preflight_age_s")
        if ctx.get("preflight_go") is not True or age is None or age > PREFLIGHT_MAX_AGE_S:
            add("preflight", "block", "Preflight GO needed",
                "Props on over the radio: run Preflight on the Connect page and get GO (under 10 % loss, "
                "under 150 ms) within the last 10 minutes. The disarm must be deliverable.")
    if props_on and flies:
        add("untested", "accept", "Props-on flight of this software is unproven",
            "No props-on run of the offboard agent has completed. Run 1 (29 Oct) lifted about 0.3 m and drifted "
            f"20 cm towards the USB cable within a second; run 2 was never flown. This profile hovers at "
            f"{p['height_m']:g} m.")
        add("drift", "accept", "No horizontal position hold",
            "Only height is controlled. Indoors the aircraft will drift sideways; the tether and the pilot "
            "limit it, the software does not.")
    if props_on and flies and (p["height_m"] > HEIGHT_DEFAULT_CAP_M or p["hold_s"] > 600):
        mins = p["hold_s"] / 60
        add("long_hover", "accept", "A long or high props-on hover",
            f"{p['height_m']:g} m for {mins:.0f} min of hold. The aircraft has no horizontal position hold, so drift "
            "accumulates for the whole time and nothing but the tether and the pilot's attention limits it. "
            "A fall from this height carries more energy than the 0.3 m run that has been flown. "
            "The pilot must stay at the handset for the full duration, with ch5 and kill in reach.")
    if props_on and flies and p["battery_land_pct"] is not None and p["battery_land_pct"] < 15:
        add("battery_deep", "accept", "The guard will not land you before PX4's own battery failsafes",
            f"The guard lands at {p['battery_land_pct']:g} % (default 15). PX4 on ARTEX113 (COM_LOW_BAT_ACT 3) warns at 20 %, "
            "returns at 15 % and lands at 10 %, and indoors a return needs a position it does not have, so the "
            "outcome is unobserved. The percentage is a voltage estimate (no current sensor, no capacity set): "
            "it reads low under load and can recover. Thrust needed to hover rises as the pack sags, and the "
            "controller aborts to LAND if it saturates for 2 s.")
    if p["flight_type"] == "free" and flies:
        add("untethered", "accept", "Free flight: nothing physically limits the aircraft",
            "A height-loop fault or a drift takes it wherever it goes. The only stops are the pilot's switches.")
    if radio and flies and props_on:
        add("radio_loop", "accept", "The height loop runs across the ELRS radio",
            "Height reaches the agent about every 2 s and its commands arrive about 0.35 s late. In the 29 Oct "
            "analysis no tuning survived a 0.05 error in the hover-thrust guess: the aircraft goes through the "
            "ceiling or onto the floor in the time between two readings. Only the pilot's switches act in "
            "real time. The agent aborts to LAND over the same radio.")
        add("gcs_failsafe", "accept", "A heartbeat gap over 5 s makes PX4 land against the pilot",
            "NAV_DLL_ACT is 3 on ARTEX113 (read 3 Nov). The 3 Nov lab-manager run saw 'Connection to ground "
            "station lost', a Land failsafe and a disarm on touchdown, not by the pilot's switch.")
    if p["link"] == "usb" and props_on and flies:
        add("usb_cable", "accept", "The USB cable can come out",
            "It jolted out at a floor impact on 29 Oct and the agent lost its link. The aircraft carries on in "
            "PX4's failsafe, not under the agent.")
    if not props_on:
        add("bench", "note", "Props off",
            "Nothing lifts, so the height loop is never exercised: a pass shows the plumbing and the takeover "
            "path, not that the hover would hold (3 Nov).")
    add("handset", "note", "The handset is the authority",
        "Take over with ch5 OFF (back to ch7 = Altitude). Stop with ch8 kill HELD for 5 s: a shorter kill "
        "leaves it armed and the motors restart. Never take over into Stabilized (ch7 -100): the stick is "
        "thrust there, and centred it is about 50 % against a 0.42 hover.")
    add("guard", "note", "Safety guard without --allow-kill",
        "On a fault the guard can command LAND but never cut the motors in the air. On a takeover it stands down.")
    return items


def authorise(p: dict, ctx: dict, body: dict) -> tuple[bool, str]:
    """Does this request carry a complete, honest statement of responsibility for THIS profile?"""
    items = assess(p, ctx)
    blocks = [i for i in items if i["level"] == "block"]
    if blocks:
        return False, "Blocked: " + "; ".join(i["title"] for i in blocks)
    need = {i["id"] for i in items if i["level"] == "accept"}
    if need - set(body.get("accepted") or []):
        return False, "Every accepted risk must be ticked: " + ", ".join(sorted(need - set(body.get("accepted") or [])))
    conf = {c["id"] for c in confirmations(p)}
    if conf - set(body.get("confirmed") or []):
        return False, "Every confirmation must be ticked."
    if len((body.get("operator") or "").strip()) < 2:
        return False, "Name the responsible operator."
    if (body.get("aircraft") or "").strip().upper() != f"ARTEX{p['sysid']}":
        return False, f"Type the aircraft name exactly: ARTEX{p['sysid']}."
    if (body.get("phrase") or "").strip().upper() != PHRASE:
        return False, f"Type exactly: {PHRASE}"
    return True, ""


class Tokens:
    """Authorisations: bound to one profile's exact contents, one use at a time, short lived."""

    def __init__(self, now=time.time):
        self._t: dict[str, dict] = {}
        self._now = now

    def issue(self, p: dict, operator: str, record: dict) -> str:
        tok = secrets.token_urlsafe(16)
        self._t[tok] = dict(hash=profile_hash(p), operator=operator.strip(), at=self._now(), record=record)
        return tok

    def get(self, tok: str | None, p: dict | None = None) -> dict | None:
        e = self._t.get(tok or "")
        if not e or self._now() - e["at"] > TOKEN_TTL_S:
            return None
        if p is not None and profile_hash(p) != e["hash"]:
            return None
        return e

    def revoke(self, tok: str) -> None:
        self._t.pop(tok, None)


# ---- run records ------------------------------------------------------------------------

def new_run_id() -> str:
    return time.strftime("%Y%m%d-%H%M%S")


def save_run(rec: dict) -> Path:
    RUN_DIR.mkdir(exist_ok=True)
    f = RUN_DIR / f"{rec['id']}.json"
    f.write_text(json.dumps(rec, indent=1, default=str) + "\n")
    return f


def list_runs() -> list[dict]:
    RUN_DIR.mkdir(exist_ok=True)
    out = []
    for f in sorted(RUN_DIR.glob("*.json"), reverse=True):
        try:
            r = json.loads(f.read_text())
        except (OSError, ValueError):
            continue
        res = r.get("result") or {}
        out.append(dict(id=r.get("id"), started=r.get("started"), mode=r.get("mode"),
                        profile=(r.get("profile") or {}).get("name"), operator=r.get("operator"),
                        sysid=(r.get("profile") or {}).get("sysid"), state=r.get("state"),
                        outcome=res.get("outcome"), passed=res.get("passed"), error=res.get("error")))
    return out


def load_run(rid: str) -> dict | None:
    f = RUN_DIR / f"{re.sub(r'[^0-9-]', '', rid)}.json"
    try:
        return json.loads(f.read_text())
    except (OSError, ValueError):
        return None


def history_block(rec: dict) -> str:
    """A paste-ready history.md entry. The GUI offers it; a person decides whether to add it."""
    p, res = rec["profile"], rec.get("result") or {}
    acc = rec.get("accepted") or []
    lines = [f"## {rec['started']} (GUI run {rec['id']}) -- {p['name']} on ARTEX{p['sysid']}, "
             f"{'DEMO' if rec.get('mode') == 'demo' else 'live'}", ""]
    lines.append(f"- Profile: {p['flight_type']} / {p['link']} / until stage {p['stage']}, "
                 f"height {p['height_m']:g} m, hold {p['hold_s']:g} s, window {p['wait_s']:g} s.")
    lines.append(f"- Responsible operator: {rec.get('operator')}. Accepted: "
                 f"{', '.join(acc) if acc else 'nothing beyond the confirmations'}.")
    lines.append(f"- Outcome: {res.get('outcome')}; passed: {res.get('passed')}; takeover: "
                 f"{res.get('takeover_source') or 'none'}"
                 f"{' after ' + str(res.get('takeover_after_s')) + ' s' if res.get('takeover_after_s') else ''}"
                 f"{'; error: ' + str(res.get('error')) if res.get('error') else ''}.")
    if res.get("setpoints"):
        lines.append(f"- Setpoints: {res['setpoints'].get('sent')} sent, {res.get('link_losses')} link losses, "
                     f"max height {res.get('max_height_m')} m.")
    lines.append(f"- Record: `runs/{rec['id']}.json`")
    return "\n".join(lines) + "\n"
