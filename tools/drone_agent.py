#!/usr/bin/env python3
"""One vehicle's agent: prepare a mission, report readiness, start, watch, abort.

    python3 tools/drone_agent.py prepare --link udpin://:14540 --expect-sysid 1 \
        --plan missions/toapayoh/mission_drones/drone-1.plan --params sar_turn_params.params
    python3 tools/drone_agent.py start   --link udpin://:14540 --expect-sysid 1
    python3 tools/drone_agent.py status  --link udpin://:14540 --expect-sysid 1
    python3 tools/drone_agent.py abort   --link udpin://:14540 --expect-sysid 1

This is the piece that runs next to a single aircraft — on a Jetson in the field, on this
workstation against SITL. It owns exactly one vehicle and speaks only MAVLink, so the link
is the only thing that changes between a simulated vehicle, a board on USB (maintenance
only) and a flight controller at the far end of an ELRS radio.

No GUI. QGroundControl has no CLI, no IPC and no headless mode, so scripting it means
synthesising mouse clicks -- not something to put on the critical path of a live flight.
QGC also uploads to one vehicle at a time; agents upload to every vehicle at once.

Every verb prints ONE json object to stdout for the orchestrator. Human-readable progress
goes to stderr, so the two never mix.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

INT_PARAM_TYPES = {1, 2, 3, 4, 5, 6, 7, 8}
HEARTBEAT_TIMEOUT_S = 20.0
MISSION_UPLOAD_TIMEOUT_S = 120.0
PARAM_READ_ATTEMPTS = 3
NAV_COMMANDS_WITH_ALTITUDE = {16, 22, 21, 82}      # waypoint, takeoff, land, spline
MIN_BATTERY_FRACTION = 0.30
FENCE_MARGIN_M = 5.0


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def emit(obj: dict) -> None:
    """The single machine-readable line. Orchestrator reads stdout, nothing else."""
    print(json.dumps(obj), flush=True)


# ---------------------------------------------------------------- link + parameters

def normalise_link(url: str) -> str:
    """Accept MAVSDK-style URLs as well as pymavlink's own spelling.

    The link is the ONLY thing that differs between SITL, a USB board and a radio, so it
    is worth accepting whatever spelling the caller has to hand.
    """
    u = url.replace("://", ":")
    if u.startswith("udpin:") and u.count(":") == 2 and u.split(":")[1] == "":
        return "udpin:0.0.0.0:" + u.rsplit(":", 1)[1]        # udpin://:14540
    if u.startswith("serial:"):
        rest = u[len("serial:"):]
        return rest.lstrip("/") if rest.startswith("///") else rest
    return u


def start_heartbeat(conn) -> None:
    """Announce ourselves as a ground station, once a second, forever.

    PX4 refuses to arm with "No connection to the ground control station" if no GCS
    heartbeat has arrived -- the command comes back MAV_RESULT_TEMPORARILY_REJECTED with
    no explanation. MAVSDK does this for you; pymavlink does not. Measured 28 Sep 2026.

    Sends are serialised behind a lock because this thread and the main one share one
    link, and on a serial link two interleaved writes corrupt both frames.
    """
    lock = threading.Lock()
    raw_write = conn.write

    def locked_write(buf):
        with lock:
            return raw_write(buf)

    conn.write = locked_write

    def beat():
        while True:
            try:
                conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                return
            time.sleep(1.0)

    threading.Thread(target=beat, daemon=True).start()


def connect(link: str, expect_sysid: int | None) -> mavutil.mavlink_connection:
    conn = mavutil.mavlink_connection(normalise_link(link), source_system=250,
                                      source_component=190)
    start_heartbeat(conn)
    log(f"[link] {link} -- waiting for heartbeat")
    hb = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    if hb is None:
        raise TimeoutError(f"no heartbeat on {link} within {HEARTBEAT_TIMEOUT_S:.0f}s")
    log(f"[link] connected, vehicle is sysid {conn.target_system}")
    if expect_sysid is not None and conn.target_system != expect_sysid:
        raise ValueError(f"wrong vehicle: link {link} answers as sysid "
                         f"{conn.target_system}, expected {expect_sysid}")
    return conn


def decode_param(msg) -> float | int:
    """PX4 advertises PARAM_ENCODE_BYTEWISE: an int arrives as the bits of a float."""
    if msg.param_type in INT_PARAM_TYPES:
        return int(struct.unpack("<i", struct.pack("<f", msg.param_value))[0])
    return msg.param_value


def read_param(conn, name: str, timeout: float = 3.0):
    for _ in range(PARAM_READ_ATTEMPTS):
        conn.mav.param_request_read_send(conn.target_system, 1, name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and msg.param_id == name:
                return decode_param(msg)
    return None


def write_param(conn, name: str, value: float, is_int: bool) -> bool:
    payload = (struct.unpack("<f", struct.pack("<i", int(value)))[0] if is_int
               else float(value))
    ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int
             else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    conn.mav.param_set_send(conn.target_system, 1, name.encode(), payload, ptype)
    deadline = time.time() + 4
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            got = decode_param(msg)
            return abs(float(got) - float(value)) < 1e-4
    return False


def write_param_typed(conn, name: str, value: float) -> bool:
    """Write a parameter whose type we look up on the vehicle first.

    Per-drone overrides come from fleet.yaml as plain numbers, with no type information.
    Guessing int-vs-float wrong writes nonsense, because PX4 encodes integers bytewise.
    """
    conn.mav.param_request_read_send(conn.target_system, 1, name.encode(), -1)
    deadline = time.time() + 4
    ptype = None
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            ptype = msg.param_type
            break
    if ptype is None:
        return False
    return write_param(conn, name, value, ptype in INT_PARAM_TYPES)


def load_param_file(path: Path) -> list[tuple[str, float, bool]]:
    """QGC .params -> [(name, value, is_int)]. Columns: sysid compid name value type."""
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = line.split()
        if len(f) >= 5:
            out.append((f[2], float(f[3]), f[4] == "6"))
    return out


# ---------------------------------------------------------------- mission

def plan_to_items(plan_path: Path) -> tuple[list[dict], float]:
    """QGC .plan -> MISSION_ITEM_INT fields, plus the mission's highest commanded altitude.

    The .plan's items map 1:1 onto MAVLink mission items; no home item is prepended.
    `params` is [p1, p2, p3, p4, x, y, z] and any of them may be null.
    """
    plan = json.loads(plan_path.read_text())
    raw = plan["mission"]["items"]
    items, top_alt = [], 0.0
    for seq, it in enumerate(raw):
        p = [0.0 if v is None else float(v) for v in it["params"]]
        cmd = int(it["command"])
        if cmd in NAV_COMMANDS_WITH_ALTITUDE:
            top_alt = max(top_alt, p[6])
        items.append(dict(
            seq=seq, frame=int(it["frame"]), command=cmd,
            current=1 if seq == 0 else 0,
            autocontinue=1 if it.get("autoContinue", True) else 0,
            p1=p[0], p2=p[1], p3=p[2], p4=p[3],
            x=int(round(p[4] * 1e7)), y=int(round(p[5] * 1e7)), z=p[6],
        ))
    return items, top_alt


def upload_mission(conn, items: list[dict]) -> None:
    """Mission protocol: COUNT, serve each request, wait for ACK.

    PX4 keeps the mission AND the current item index in dataman across restarts, and
    uploading does not reset the index. A vehicle whose last run ended on the RTL item
    will otherwise resume *there* -- "return to launch" from 1 m, then disarm on the
    ground, having never taken off. Measured 24 Sep 2026. Hence the clear and the
    explicit set-current below.
    """
    MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    conn.mav.mission_clear_all_send(conn.target_system, 1, MT)
    conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)

    conn.mav.mission_count_send(conn.target_system, 1, len(items), MT)
    sent, deadline = set(), time.time() + MISSION_UPLOAD_TIMEOUT_S
    while time.time() < deadline:
        msg = conn.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                    "MISSION_ACK"], blocking=True, timeout=5)
        if msg is None:
            raise TimeoutError(f"vehicle stopped requesting items at {len(sent)}/{len(items)}")
        if msg.get_type() == "MISSION_ACK":
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"vehicle rejected the mission: MAV_MISSION result {msg.type}")
            break
        i = items[msg.seq]
        conn.mav.mission_item_int_send(
            conn.target_system, 1, i["seq"], i["frame"], i["command"],
            i["current"], i["autocontinue"], i["p1"], i["p2"], i["p3"], i["p4"],
            i["x"], i["y"], i["z"], MT)
        sent.add(msg.seq)
    else:
        raise TimeoutError("no MISSION_ACK before timeout")

    conn.mav.mission_set_current_send(conn.target_system, 1, 0)
    log(f"[plan] uploaded {len(items)} items, current index reset to 0")


def mission_count_onboard(conn) -> int | None:
    conn.mav.mission_request_list_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    msg = conn.recv_match(type="MISSION_COUNT", blocking=True, timeout=6)
    if msg is None:
        return None
    conn.mav.mission_ack_send(conn.target_system, 1,
                              mavutil.mavlink.MAV_MISSION_ACCEPTED,
                              mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    return msg.count


# ---------------------------------------------------------------- preflight contract

def gather_state(conn, seconds: float = 6.0) -> dict:
    state, deadline = {}, time.time() + seconds
    while time.time() < deadline:
        msg = conn.recv_match(blocking=True, timeout=2)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        state[msg.get_type()] = msg
    return state


def preflight_contract(conn, expect_sysid, top_alt, uploaded, onboard) -> list[dict]:
    """The checks that a .plan cannot express.

    Every one of these came from something that actually bit us: a 7 m vertical geofence
    with GF_ACTION=Land that put the aircraft back on the ground during climb-out, an
    RTL_RETURN_ALT of 5 m that would transit home below the rooftops, NAV_RCL_ACT=Land on
    a link where RC loss is also C2 loss.
    """
    checks = []

    def add(name, ok, detail):
        checks.append(dict(name=name, ok=bool(ok), detail=detail))

    add("identity", conn.target_system == expect_sysid,
        f"vehicle answers as sysid {conn.target_system}, slot expects {expect_sysid}")
    add("mission_uploaded", onboard == uploaded,
        f"{onboard} items on board, {uploaded} in the plan")

    gf_dist = read_param(conn, "GF_MAX_VER_DIST")
    gf_act = read_param(conn, "GF_ACTION")
    fence_ok = gf_dist is not None and (gf_dist == 0 or gf_dist > top_alt + FENCE_MARGIN_M)
    add("fence_ceiling", fence_ok,
        f"GF_MAX_VER_DIST={gf_dist} (0=off), mission tops out at {top_alt:.1f} m"
        + ("" if fence_ok else f"; GF_ACTION={gf_act}"))

    rtl_alt = read_param(conn, "RTL_RETURN_ALT")
    add("rtl_altitude", rtl_alt is not None and rtl_alt >= top_alt,
        f"RTL_RETURN_ALT={rtl_alt}, mission tops out at {top_alt:.1f} m")

    rcl = read_param(conn, "NAV_RCL_ACT")
    add("rc_loss_action", rcl is not None and rcl != 3,
        f"NAV_RCL_ACT={rcl} (3=Land in place, 2=Return)")

    state = gather_state(conn)
    gps = state.get("GPS_RAW_INT")
    fix = gps.fix_type if gps else 0
    add("gps_fix", fix >= 3, f"fix_type={fix}, satellites={gps.satellites_visible if gps else 0}")

    pos = state.get("GLOBAL_POSITION_INT")
    add("position_estimate", pos is not None,
        "global position published" if pos else "no GLOBAL_POSITION_INT")

    batt = state.get("BATTERY_STATUS")
    remaining = batt.battery_remaining if batt else -1
    add("battery", remaining < 0 or remaining >= MIN_BATTERY_FRACTION * 100,
        f"battery_remaining={remaining}% (-1 = not reported)")

    return checks


# ---------------------------------------------------------------- verbs

def run_token(run_id: str, plan_path: Path) -> str:
    """Binds a START to the exact plan that was prepared.

    Without it, `start` has no memory that `prepare` ever happened: it would arm a vehicle
    that was never verified, or one whose plan was replaced in between. The barrier checks
    that every drone *said* ready; the token checks that the thing about to arm is the
    thing that was checked.
    """
    digest = hashlib.sha256(plan_path.read_bytes()).hexdigest()[:12]
    return f"{run_id}:{digest}"


def resident_loop(conn, args, token: str) -> int:
    """Hold the link and take commands on stdin until told to stop.

    The link staying up is the point. PX4 will not arm for a ground station it has not
    heard from recently, so a one-shot `start` that reconnects has to re-establish that
    relationship from cold -- cheap on localhost UDP, expensive on a radio whose
    round-trip is tens of milliseconds. One connection from prepare through landing.
    """
    state = dict(failsafe_armed=False, last_cmd=time.time())

    def watchdog():
        while True:
            time.sleep(1.0)
            if (state["failsafe_armed"] and args.failsafe_s > 0
                    and time.time() - state["last_cmd"] > args.failsafe_s):
                log(f"[failsafe] no command for {args.failsafe_s:.0f}s -- returning to launch")
                r = do_abort(conn, "rtl")
                emit(dict(verb="failsafe", slot=args.slot, sysid=conn.target_system,
                          t_epoch=time.time(), **r))
                state["failsafe_armed"] = False

    threading.Thread(target=watchdog, daemon=True).start()
    log(f"[resident] holding link, token {token}, "
        f"failsafe {'off' if args.failsafe_s <= 0 else f'{args.failsafe_s:.0f}s'}")

    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        state["last_cmd"] = time.time()
        parts = line.split()
        verb = parts[0].upper()

        if verb == "START":
            given = parts[1] if len(parts) > 1 else ""
            if given != token:
                emit(dict(verb="start", slot=args.slot, sysid=conn.target_system,
                          started=False,
                          error=f"token mismatch: given {given!r}, prepared {token!r}"))
                continue
            at = float(parts[2]) if len(parts) > 2 else None
            r = do_start(conn, at)
            emit(dict(verb="start", slot=args.slot, sysid=conn.target_system, **r))
            state["last_cmd"] = time.time()
            state["failsafe_armed"] = bool(r.get("started"))

        elif verb == "STATUS":
            emit(dict(verb="status", slot=args.slot, sysid=conn.target_system,
                      **do_status(conn)))

        elif verb == "ABORT":
            state["failsafe_armed"] = False
            action = parts[1] if len(parts) > 1 else "rtl"
            emit(dict(verb="abort", slot=args.slot, sysid=conn.target_system,
                      **do_abort(conn, action)))

        elif verb == "QUIT":
            emit(dict(verb="quit", slot=args.slot, sysid=conn.target_system))
            return 0

        else:
            emit(dict(verb="error", slot=args.slot, error=f"unknown command {verb!r}"))

    log("[resident] stdin closed")
    return 0


def cmd_prepare(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    plan_path = Path(args.plan)
    items, top_alt = plan_to_items(plan_path)
    log(f"[plan] {plan_path.name}: {len(items)} items, top altitude {top_alt:.1f} m")

    if args.params:
        applied = failed = 0
        for name, value, is_int in load_param_file(Path(args.params)):
            if write_param(conn, name, value, is_int):
                applied += 1
            else:
                failed += 1
        log(f"[param] {applied} applied, {failed} failed")

    # Per-drone overrides. This is where RTL separation lives: each aircraft returns at
    # its own altitude, so two drones coming home never collapse onto one layer.
    for pair in args.set or []:
        name, _, raw = pair.partition("=")
        if write_param_typed(conn, name.strip(), float(raw)):
            log(f"[param] override {name.strip()} = {raw}")
        else:
            log(f"[param] override {name.strip()} FAILED")

    upload_mission(conn, items)
    onboard = mission_count_onboard(conn)
    checks = preflight_contract(conn, args.expect_sysid, top_alt, len(items), onboard)

    ready = all(c["ok"] for c in checks)
    for c in checks:
        log(f"[check] {'PASS' if c['ok'] else 'FAIL'}  {c['name']}: {c['detail']}")
    token = run_token(args.run_id, plan_path)
    emit(dict(verb="prepare", slot=args.slot, sysid=conn.target_system, ready=ready,
              plan=str(plan_path), items=len(items), top_alt_m=top_alt,
              token=token, checks=checks,
              reasons=[c["name"] for c in checks if not c["ok"]]))
    if args.resident:
        return resident_loop(conn, args, token)
    return 0 if ready else 1


def do_start(conn, at: float | None) -> dict:
    """Arm, then MISSION_START. Returns the report; the caller decides how to emit it."""
    if at:
        wait = at - time.time()
        if wait > 0:
            log(f"[start] holding {wait:.2f}s for the shared launch instant")
            time.sleep(wait)

    armed, deadline, last_try = False, time.time() + 20, 0.0
    while time.time() < deadline:
        if time.time() - last_try > 3.0:
            conn.mav.command_long_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 0, 0, 0, 0, 0, 0)
            last_try = time.time()
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
    if not armed:
        return dict(started=False, armed=False, error="vehicle did not arm")

    log("[start] armed")
    conn.mav.command_long_send(conn.target_system, 1,
                               mavutil.mavlink.MAV_CMD_MISSION_START,
                               0, 0, 0, 0, 0, 0, 0, 0)
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    ok = ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    log(f"[start] MISSION_START {'accepted' if ok else 'not accepted'}")
    return dict(started=ok, armed=True, t_epoch=time.time())


def do_status(conn) -> dict:
    state = gather_state(conn, seconds=3)
    hb = state.get("HEARTBEAT")
    pos = state.get("GLOBAL_POSITION_INT")
    cur = state.get("MISSION_CURRENT")
    batt = state.get("BATTERY_STATUS")
    return dict(
        armed=bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None,
        main_mode=(hb.custom_mode >> 16) & 0xFF if hb else None,
        sub_mode=(hb.custom_mode >> 24) & 0xFF if hb else None,
        waypoint=cur.seq if cur else None,
        rel_alt_m=round(pos.relative_alt / 1000.0, 2) if pos else None,
        lat=pos.lat / 1e7 if pos else None, lon=pos.lon / 1e7 if pos else None,
        battery_pct=batt.battery_remaining if batt else None)


def do_abort(conn, action: str) -> dict:
    cmd = (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH if action == "rtl"
           else mavutil.mavlink.MAV_CMD_NAV_LAND)
    conn.mav.command_long_send(conn.target_system, 1, cmd, 0, 0, 0, 0, 0, 0, 0, 0)
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    ok = ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    log(f"[abort] {action} {'accepted' if ok else 'NOT accepted'}")
    return dict(action=action, accepted=ok)


def cmd_start(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    time.sleep(2.0)   # one-shot mode: PX4 must see a few GCS heartbeats before arming
    r = do_start(conn, args.at)
    emit(dict(verb="start", slot=args.slot, sysid=conn.target_system, **r))
    return 0 if r.get("started") else 1


def cmd_status(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    emit(dict(verb="status", slot=args.slot, sysid=conn.target_system, **do_status(conn)))
    return 0


def cmd_abort(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    emit(dict(verb="abort", slot=args.slot, sysid=conn.target_system,
              **do_abort(conn, args.action)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("verb", choices=["prepare", "start", "status", "abort"])
    ap.add_argument("--link", required=True,
                    help="udpin://:14540, serial:///dev/ttyACM0, udpout://host:port")
    ap.add_argument("--expect-sysid", type=int, default=None,
                    help="refuse the vehicle if it answers as anything else")
    ap.add_argument("--slot", type=int, default=None, help="which drone-N this is")
    ap.add_argument("--plan")
    ap.add_argument("--params")
    ap.add_argument("--at", type=float, default=None,
                    help="epoch seconds: hold, then arm and start (shared clock launch)")
    ap.add_argument("--set", action="append", metavar="NAME=VALUE",
                    help="per-drone parameter override, repeatable")
    ap.add_argument("--resident", action="store_true",
                    help="after prepare, hold the link and take commands on stdin")
    ap.add_argument("--run-id", default="run", help="run id, part of the token")
    ap.add_argument("--failsafe-s", type=float, default=60.0,
                    help="once flying, RTL if the orchestrator goes silent this long "
                         "(0 disables)")
    ap.add_argument("--action", choices=["rtl", "land"], default="rtl")
    args = ap.parse_args()

    if args.verb == "prepare" and not args.plan:
        ap.error("prepare needs --plan")

    try:
        return {"prepare": cmd_prepare, "start": cmd_start,
                "status": cmd_status, "abort": cmd_abort}[args.verb](args)
    except Exception as exc:
        log(f"[error] {type(exc).__name__}: {exc}")
        emit(dict(verb=args.verb, slot=args.slot, ready=False, error=str(exc)))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
