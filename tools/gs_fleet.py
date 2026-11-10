"""Ground-station core for MULTI-AIRCRAFT waypoint runs (10 Nov 2026). Pure functions and files, no hardware;
tests/test_gs_fleet.py pins them. tools/gs_server.py drives the agents; gui/index.html shows it.

The run is the swarm_deploy.py sequence, with the operator in the two places that matter:

  PREPARE   one resident `drone_agent.py prepare` per aircraft: identity, upload, read-back, preflight contract
  BARRIER   every aircraft READY, or NOBODY arms
  ARM ALL   gate 1, the operator's press: `ARM <token>` to all at one instant; every aircraft must report armed
  START ALL gate 2, the operator's press: `START <token>` to all; each starts its mission (AUTO.MISSION)
  WATCH     STATUS polls; the first fault on any aircraft puts every OTHER aircraft in HOLD (Loiter)

Operator's decisions (10 Nov 2026): waypoint missions first; two gates (arm all, then go all; a failure at either
gate means nobody proceeds and the armed ones are disarmed); after the start, a fault on one aircraft puts the
others in HOLD; one named pilot per aircraft.

What this file does NOT relax: the agent's own preflight contract (identity, mission on board, fence and RTL above
the mission, RC-loss action, 3-D GPS fix, a position estimate, battery). Indoors there is no GPS fix, so a real
aircraft stops at NOT READY and nobody arms; that is the guardrail working. Indoors, use mode "upload".
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path

import gs_core as core

FLEET_DIR = core.PROFILE_DIR / "fleet"
MODES = ("upload", "arm", "mission")
MODE_NOTE = {
    "upload": "Upload and verify each aircraft's mission, run the preflight contract, stop. Never arms.",
    "arm": "Upload, READY barrier, ARM ALL, hold, DISARM ALL. Never starts a mission.",
    "mission": "Upload, READY barrier, ARM ALL, then START ALL: every aircraft flies its mission.",
}
FLIGHT_TYPES = ("bench", "props_on")
# PX4 disarms an armed aircraft that has not taken off after COM_DISARM_PRFLT (10 s on the ARTEX airframes, seen in
# every armed bench run on 5 Nov 2026). START ALL must come inside that, or the aircraft disarm under the operator.
START_WINDOW_S = 8.0
READY_GATE_S = 150.0          # how long the barrier waits for ARM ALL before letting every agent go
ARM_LEAD_S = 2.0              # shared ARM instant, seconds after the press
START_LEAD_S = 1.5            # shared START instant
WATCH_POLL_S = 4.0
STATUS_MISSES_FAULT = 2       # consecutive STATUS timeouts that count as a lost link
PX4_MAIN_AUTO = 4
PX4_SUB = {2: "TAKEOFF", 3: "LOITER (HOLD)", 4: "MISSION", 5: "RTL", 6: "LAND", 7: "RTGS", 8: "FOLLOW", 9: "PRECLAND"}
PX4_MAIN = {1: "MANUAL", 2: "ALTITUDE", 3: "POSITION", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED"}

DEFAULTS = dict(name="", description="", mode="upload", flight_type="bench", arm_hold_s=5.0, failsafe_s=60.0,
                aircraft=[])
MEMBER_DEFAULTS = dict(sysid=None, plan="", link="radio")


# ---- profiles ---------------------------------------------------------------------------

def normalise(raw: dict) -> dict:
    p = dict(DEFAULTS)
    for k in DEFAULTS:
        if k in raw and raw[k] not in (None, ""):
            p[k] = raw[k]
    for k in ("arm_hold_s", "failsafe_s"):
        try:
            p[k] = float(p[k])
        except (TypeError, ValueError):
            p[k] = float("nan")
    members = []
    for m in p["aircraft"] if isinstance(p["aircraft"], list) else []:
        if not isinstance(m, dict):
            continue
        q = dict(MEMBER_DEFAULTS)
        q.update({k: m[k] for k in MEMBER_DEFAULTS if k in m and m[k] not in (None, "")})
        try:
            q["sysid"] = int(q["sysid"])
        except (TypeError, ValueError):
            q["sysid"] = -1
        q["plan"] = str(q["plan"]).strip()
        members.append(q)
    p["aircraft"] = sorted(members, key=lambda m: m["sysid"])
    return p


def plan_ok(path: str) -> bool:
    """A plan inside missions/, named drone-N.plan, that exists. Nothing outside the repo is ever pushed."""
    return bool(re.fullmatch(r"missions/[A-Za-z0-9._/-]+/drone-\d+\.plan", path)) and ".." not in path \
        and (core.ROOT / path).is_file()


def plan_summary(path: str) -> dict:
    """Items and the highest commanded altitude, as the agent will see them."""
    try:
        items = json.loads((core.ROOT / path).read_text())["mission"]["items"]
    except (OSError, ValueError, KeyError, TypeError):
        return dict(items=None, top_alt_m=None)
    top = 0.0
    for it in items:
        if int(it.get("command", 0)) in (16, 22, 17, 18, 19, 21):       # waypoint, takeoff, loiter*, land
            try:
                top = max(top, float((it.get("params") or [0] * 7)[6] or 0))
            except (TypeError, ValueError, IndexError):
                pass
    return dict(items=len(items), top_alt_m=round(top, 1))


def validate(p: dict) -> list[str]:
    e = []
    if not re.search(r"[A-Za-z0-9]", p["name"]):
        e.append("Give the fleet profile a name (letters or digits).")
    if p["mode"] not in MODES:
        e.append("Mode must be upload, arm or mission.")
    if p["flight_type"] not in FLIGHT_TYPES:
        e.append("Flight type must be bench (props off) or props_on.")
    if p["mode"] == "mission" and p["flight_type"] == "bench":
        e.append("A mission run flies: it cannot be a props-off bench run. Use mode arm to test the gates props off.")
    if not (p["arm_hold_s"] == p["arm_hold_s"] and 1 <= p["arm_hold_s"] <= START_WINDOW_S):
        e.append(f"Arm hold must be 1 to {START_WINDOW_S:g} s (PX4 disarms an aircraft on the ground after ~10 s).")
    if not (p["failsafe_s"] == p["failsafe_s"] and 20 <= p["failsafe_s"] <= 300):
        e.append("Agent failsafe must be 20 to 300 s (each agent returns its aircraft if the ground station goes silent).")
    ac = p["aircraft"]
    if len(ac) < 2:
        e.append("A fleet run needs at least two aircraft.")
    sysids = [m["sysid"] for m in ac]
    if len(set(sysids)) != len(sysids):
        e.append("Each aircraft may appear once.")
    for m in ac:
        if m["sysid"] not in core.AIRCRAFT:
            e.append(f"Aircraft {m['sysid']} is not known here (ARTEX{', ARTEX'.join(map(str, core.AIRCRAFT))}).")
        if m["link"] not in ("radio", "usb"):
            e.append(f"ARTEX{m['sysid']}: link must be radio or usb.")
        if not plan_ok(m["plan"]):
            e.append(f"ARTEX{m['sysid']}: choose a plan from missions/ (missions/<site>/mission_drones/drone-N.plan).")
    hosts = [core.AIRCRAFT[m["sysid"]]["host"] for m in ac if m["sysid"] in core.AIRCRAFT]
    if len(set(hosts)) != len(hosts):
        e.append("Two aircraft on one companion: each Jetson holds one aircraft's link.")
    plans = [m["plan"] for m in ac]
    if len(set(plans)) != len(plans):
        e.append("Each aircraft needs its own plan.")
    if p["mode"] != "upload" and len({str(Path(x).parent) for x in plans}) > 1:
        e.append("The plans come from different planning runs. The separation guardrail only checked drones planned "
                 "together: pick every plan from one mission_drones folder.")
    return e


def profile_hash(p: dict) -> str:
    return core.profile_hash(p)


def list_profiles() -> list[dict]:
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    out = []
    for f in sorted(FLEET_DIR.glob("*.json")):
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
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    (FLEET_DIR / f"{core.slug(p['name'])}.json").write_text(json.dumps(p, indent=1) + "\n")
    return dict(p, id=core.slug(p["name"]))


def delete_profile(pid: str) -> bool:
    f = FLEET_DIR / f"{core.slug(pid)}.json"
    if f.exists():
        arch = FLEET_DIR / "deleted"
        arch.mkdir(exist_ok=True)
        f.rename(arch / f"{f.stem}-{int(time.time())}.json")         # archive, never delete
        return True
    return False


def list_plans() -> list[dict]:
    """Every drone-N.plan under missions/, grouped by the planning run (its folder)."""
    out = []
    for f in sorted((core.ROOT / "missions").glob("**/drone-*.plan")):
        rel = str(f.relative_to(core.ROOT))
        if plan_ok(rel):
            out.append(dict(path=rel, folder=str(Path(rel).parent), **plan_summary(rel)))
    return out


# ---- the agent commands -------------------------------------------------------------------

def agent_argv(p: dict, m: dict, run_id: str) -> list[str]:
    """One resident agent for one aircraft. The plan path is the same relative path on the companion."""
    ac = core.AIRCRAFT[m["sysid"]]
    return ["python3", "tools/drone_agent.py", "prepare", "--link", ac[m["link"]], "--slot", str(m["sysid"]),
            "--plan", m["plan"], "--run-id", run_id, "--failsafe-s", f"{p['failsafe_s']:g}", "--resident",
            "--expect-sysid", str(m["sysid"])]


# ---- responsibility ----------------------------------------------------------------------

def confirmations(p: dict) -> list[dict]:
    names = " and ".join(f"ARTEX{m['sysid']}" for m in p["aircraft"])
    if p["flight_type"] == "bench":
        c = [f"Propellers are OFF on {names}", "Hands and tools are clear of every rotor"]
    else:
        c = [f"Propellers are ON on {names} and checked tight",
             "The operating area, including overhead, is clear of anyone not on the crew",
             "Every aircraft's take-off point and its mission area have been walked",
             "Each named pilot can see their own aircraft for the whole run",
             "Every handset: ch5 Offboard OFF, ch7 at CENTRE, ch8 kill off, kill within reach"]
    c.append("Each aircraft carries the parameter configuration this run needs (outdoors: the outdoor baseline, "
             "not the 3 m/s indoor caps)" if p["flight_type"] == "props_on" else
             "This is a bench run: the indoor configuration may make the preflight contract refuse, which is fine")
    return [dict(id=f"c{i}", text=t) for i, t in enumerate(c)]


def assess(p: dict, ctx: dict) -> list[dict]:
    """ctx: {sysid: {preflight_go, preflight_age_s}}. Items are level block|accept|note."""
    items = []

    def add(i, level, title, text):
        items.append(dict(id=i, level=level, title=title, text=text))

    for msg in validate(p):
        add("invalid", "block", "Fleet profile is not runnable", msg)
    props_on = p["flight_type"] == "props_on"
    if props_on:
        stale = []
        for m in p["aircraft"]:
            c = ctx.get(m["sysid"]) or {}
            age = c.get("preflight_age_s")
            if m["link"] == "radio" and (c.get("preflight_go") is not True or age is None
                                         or age > core.PREFLIGHT_MAX_AGE_S):
                stale.append(f"ARTEX{m['sysid']}")
        if stale:
            add("preflight", "block", "Preflight GO needed on every aircraft",
                f"Props on over the radio: {', '.join(stale)} need a Preflight GO from the last 10 minutes "
                "(under 10 % loss, under 150 ms). Every disarm and every HOLD must be deliverable.")
    if p["mode"] == "mission":
        add("multi_untested", "accept", "No multi-aircraft mission has been flown on the real aircraft",
            "The sequence has run in simulation and as props-off bench gates only. Both aircraft fly their missions at "
            "once, each under its own PX4, each with its own pilot.")
        add("hold_position", "accept", "HOLD needs a position",
            "When one aircraft faults, the others are put in HOLD (PX4 Loiter). Loiter needs a GPS position; without "
            "one PX4 refuses or falls back to its own failsafe. The pilots must be ready to take over.")
        add("start_spread", "accept", "The aircraft do not start at exactly the same instant",
            "Arming over the radio has spread up to ~2.6 s between aircraft (14 Oct). START is sent to all for one "
            "shared instant, but each arrives over its own radio.")
        add("separation", "accept", "Separation is only what the planner guaranteed",
            "There is no runtime collision avoidance. The planner stacks the aircraft in altitude layers and checked "
            "the plans together; a takeover or a HOLD can bring two aircraft closer than planned.")
        add("link_load", "accept", "The radio carries PX4's full default telemetry",
            "About 80-86 % of the ELRS telemetry capacity. Commands (HOLD, RTL) arrive in ~1 s, sometimes several. "
            "A lean outdoor downlink profile is not built yet.")
    if p["mode"] in ("arm", "mission") and props_on:
        add("motors", "accept", "Several aircraft armed at once with props on",
            "ARM ALL spins every set of props within a couple of seconds. The pilots' kill switches are the stop.")
    if p["mode"] == "upload":
        add("upload_only", "note", "Upload only",
            "Nothing arms. Writes each mission to its flight controller (a mission, not a parameter) and reports "
            "the preflight contract. Upload over the radio is about 1 s per item.")
    add("pilots", "note", "One pilot per aircraft",
        "Each aircraft's handset is its override: ch5 OFF to take over, ch8 kill held 5 s to stop. The buttons here "
        "go through the agents and the radio.")
    add("contract", "note", "The agent's preflight contract still applies",
        "Each agent refuses READY without identity, the mission on board, fence and RTL above the mission, a 3-D GPS "
        "fix and a position estimate. Indoors that means NOT READY, and nobody arms.")
    return items


def authorise(p: dict, ctx: dict, body: dict) -> tuple[bool, str]:
    items = assess(p, ctx)
    blocks = [i for i in items if i["level"] == "block"]
    if blocks:
        return False, "Blocked: " + "; ".join(i["title"] for i in blocks)
    need = {i["id"] for i in items if i["level"] == "accept"}
    missing = need - set(body.get("accepted") or [])
    if missing:
        return False, "Every accepted risk must be ticked: " + ", ".join(sorted(missing))
    if {c["id"] for c in confirmations(p)} - set(body.get("confirmed") or []):
        return False, "Every confirmation must be ticked."
    if len((body.get("operator") or "").strip()) < 2:
        return False, "Name the responsible operator."
    pilots = body.get("pilots") or {}
    names = []
    for m in p["aircraft"]:
        n = str(pilots.get(str(m["sysid"])) or pilots.get(m["sysid"]) or "").strip()
        if len(n) < 2:
            return False, f"Name the pilot holding ARTEX{m['sysid']}'s handset."
        names.append(n.lower())
    if len(set(names)) != len(names):
        return False, "One pilot per aircraft: the same person cannot hold two handsets."
    typed = {t.strip().upper() for t in re.split(r"[,\s+]+", body.get("aircraft") or "") if t.strip()}
    if typed != {f"ARTEX{m['sysid']}" for m in p["aircraft"]}:
        return False, "Type every aircraft name, e.g. " + ", ".join(f"ARTEX{m['sysid']}" for m in p["aircraft"]) + "."
    if (body.get("phrase") or "").strip().upper() != core.PHRASE:
        return False, f"Type exactly: {core.PHRASE}"
    return True, ""


# ---- reading the agents, and the "others hold" rule --------------------------------------

def describe_mode(main: int | None, sub: int | None) -> str:
    if main is None:
        return "?"
    if main == PX4_MAIN_AUTO:
        return "AUTO " + PX4_SUB.get(sub, str(sub))
    return PX4_MAIN.get(main, str(main))


def member_fault(st: dict | None, items: int | None, misses: int) -> str | None:
    """Is this aircraft, after START, no longer flying its mission as planned? None = fine (or finished normally).

    st: the last STATUS reply (armed, main_mode, sub_mode, waypoint). A mission ends with PX4's RTL or LAND and a
    disarm on touchdown; that is not a fault. Leaving AUTO while armed is (the pilot took over, or a PX4 failsafe).
    """
    if misses >= STATUS_MISSES_FAULT:
        return f"no STATUS for {misses} polls: link lost"
    if not st:
        return None
    if st.get("armed") is False:
        last = items - 1 if items else None
        if st.get("_was_rtl") or (last is not None and (st.get("waypoint") or 0) >= last):
            return None
        return f"disarmed mid-mission at waypoint {st.get('waypoint')}"
    if st.get("armed") and st.get("main_mode") is not None and st.get("main_mode") != PX4_MAIN_AUTO:
        return f"left AUTO while armed: now {describe_mode(st.get('main_mode'), st.get('sub_mode'))}"
    return None


def hold_targets(members: dict[int, dict], faulted: int) -> list[int]:
    """Which aircraft get HOLD when `faulted` faults: every other one still armed and flying its mission, not already
    held, not finished and not in a pilot's hands."""
    out = []
    for sysid, m in members.items():
        if sysid == faulted or m.get("held") or m.get("fault") or m.get("finished"):
            continue
        st = m.get("status") or {}
        if st.get("armed") and st.get("main_mode") == PX4_MAIN_AUTO and st.get("sub_mode") in (2, 4):
            out.append(sysid)
        elif m.get("started") and not st:
            out.append(sysid)         # started but no status yet: hold it rather than guess
    return sorted(out)
