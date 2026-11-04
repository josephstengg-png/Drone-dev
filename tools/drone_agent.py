#!/usr/bin/env python3
"""One vehicle's agent: prepare a mission, report readiness, start, watch, abort.

    python3 tools/drone_agent.py prepare --link udpin://:14540 --expect-sysid 1 \
        --plan missions/toapayoh/mission_drones/drone-1.plan --params sar_turn_params.params
    python3 tools/drone_agent.py upload  --link udpin://:14540 --expect-sysid 1 \
        --plan missions/toapayoh/mission_drones/drone-1.plan
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
import select
import struct
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_rules  # noqa: E402

INT_PARAM_TYPES = {1, 2, 3, 4, 5, 6, 7, 8}
HEARTBEAT_TIMEOUT_S = 20.0
MISSION_UPLOAD_TIMEOUT_S = 120.0
UPLOAD_S_PER_ITEM = 4.0      # ELRS measured ~1 s/item; leave 4x headroom for a bad link
# PX4 stores MISSION_ITEM_INT, so a plan's local-relative frame 3 reads back as 6 and 0 as 5.
FRAME_EQUIV = {3: 6, 0: 5}
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


def start_heartbeat(conn, interval_s: float = 1.0) -> None:
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

    conn.hb_failures = 0
    conn.hb_last_ok = None

    def beat():
        # 19 Oct 2026: this loop used to `return` on the first exception, so one failed send
        # stopped the heartbeat for good, silently. The 15 Oct incident log shows PX4 lost
        # the agent's heartbeat about 4 s before its setpoints stopped. Never give up; say so.
        while True:
            try:
                conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                conn.hb_last_ok = time.time()
            except Exception as exc:
                conn.hb_failures += 1
                if conn.hb_failures == 1 or conn.hb_failures % 10 == 0:
                    log(f"[link] heartbeat send failed ({conn.hb_failures}x): {exc} -- retrying")
            time.sleep(interval_s)

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


def upload_mission(conn, items: list[dict], timeout: float = MISSION_UPLOAD_TIMEOUT_S) -> None:
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

    def send_item(seq: int) -> None:
        i = items[seq]
        conn.mav.mission_item_int_send(
            conn.target_system, 1, i["seq"], i["frame"], i["command"],
            i["current"], i["autocontinue"], i["p1"], i["p2"], i["p3"], i["p4"],
            i["x"], i["y"], i["z"], MT)

    conn.mav.mission_count_send(conn.target_system, 1, len(items), MT)
    sent, deadline = set(), time.time() + timeout
    last_seq, quiet_resends = None, 0
    while time.time() < deadline:
        msg = conn.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                    "MISSION_ACK"], blocking=True, timeout=2.5)
        if msg is None:
            # a radio drops packets: nudge the vehicle by repeating our last message
            quiet_resends += 1
            if quiet_resends > 6:
                raise TimeoutError(
                    f"vehicle stopped requesting items at {len(sent)}/{len(items)}")
            log(f"[plan] silence at {len(sent)}/{len(items)}; resend {quiet_resends}/6")
            if last_seq is None:
                conn.mav.mission_count_send(conn.target_system, 1, len(items), MT)
            else:
                send_item(last_seq)
            continue
        quiet_resends = 0
        if msg.get_type() == "MISSION_ACK":
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"vehicle rejected the mission: MAV_MISSION result {msg.type}")
            break
        send_item(msg.seq)
        last_seq = msg.seq
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


def download_mission(conn, attempts: int = 4) -> list[dict] | None:
    """Read the whole mission back from the vehicle, item by item. None if it cannot."""
    MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = None
    for _ in range(attempts):
        conn.mav.mission_request_list_send(conn.target_system, 1, MT)
        msg = conn.recv_match(type="MISSION_COUNT", blocking=True, timeout=6)
        if msg is not None:
            count = msg.count
            break
    if count is None:
        return None

    def fetch(seq: int):
        for _ in range(attempts):
            conn.mav.mission_request_int_send(conn.target_system, 1, seq, MT)
            deadline = time.time() + 5
            while time.time() < deadline:
                m = conn.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=1)
                if m is not None and m.seq == seq:
                    return m
        return None

    items = []
    for seq in range(count):
        m = fetch(seq)
        if m is None:
            log(f"[readback] item {seq}/{count} never arrived")
            return None
        items.append(dict(seq=m.seq, frame=m.frame, command=m.command,
                          p1=m.param1, p2=m.param2, p3=m.param3, p4=m.param4,
                          x=m.x, y=m.y, z=m.z))
    conn.mav.mission_ack_send(conn.target_system, 1,
                              mavutil.mavlink.MAV_MISSION_ACCEPTED, MT)
    return items


def diff_items(sent: list[dict], got: list[dict]) -> list[str]:
    """Human-readable differences between what we sent and what the vehicle holds."""
    if len(sent) != len(got):
        return [f"{len(got)} items on board, {len(sent)} sent"]
    norm = lambda f: FRAME_EQUIV.get(f, f)
    out = []
    for a, b in zip(sent, got):
        bad = []
        if a["command"] != b["command"]:
            bad.append("command")
        if norm(a["frame"]) != norm(b["frame"]):
            bad.append("frame")
        if a["x"] != b["x"] or a["y"] != b["y"]:
            bad.append("position")
        bad += [k for k in ("p1", "p2", "p3", "p4", "z") if abs(a[k] - b[k]) > 1e-3]
        if bad:
            out.append(f"item {a['seq']}: {'/'.join(bad)} differ")
    return out


def vehicle_armed(conn) -> bool | None:
    """Armed state from the VEHICLE's own heartbeat (not ours, not another GCS's)."""
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg is None or msg.get_srcSystem() != conn.target_system:
            continue
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            continue
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


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
    state = dict(failsafe_armed=False, last_cmd=time.time(), armed_only=False)
    guard = Guard(conn, args)
    # pymavlink's connection is not safe to read from two threads. Every command handler
    # below runs under this lock, and the monitor only reads while holding it, so there is
    # exactly one reader at any instant.
    conn_lock = threading.Lock()

    def monitor():
        while True:
            time.sleep(0.2)
            with conn_lock:
                msg = conn.recv_match(blocking=False)
                while msg is not None:
                    guard.observe(msg)
                    msg = conn.recv_match(blocking=False)
                acted = guard.check()
            if acted:
                state["failsafe_armed"] = False
                if acted.get("carried_out") in (safety_rules.LAND, safety_rules.DISARM):
                    state["armed_only"] = False

    def disarm_if_left_armed():
        # We armed it for an arm-only test and the orchestrator is gone: do not leave
        # motors armed with nobody watching.
        if guard.pilot_has_it:
            log("[safety] leaving, and the PILOT HAS IT: not disarming. If the aircraft is "
                "still armed, use the arm switch (ch6) or the kill switch (ch8).")
            return
        if state["armed_only"]:
            log("[resident] leaving while armed by ARM -- disarming")
            with conn_lock:
                do_disarm(conn)
            state["armed_only"] = False

    def watchdog():
        while True:
            time.sleep(1.0)
            if guard.pilot_has_it:
                state["failsafe_armed"] = False
            if (state["failsafe_armed"] and args.failsafe_s > 0
                    and time.time() - state["last_cmd"] > args.failsafe_s):
                log(f"[failsafe] no command for {args.failsafe_s:.0f}s -- returning to launch")
                with conn_lock:
                    r = do_abort(conn, "rtl")
                emit(dict(verb="failsafe", slot=args.slot, sysid=conn.target_system,
                          t_epoch=time.time(), **r))
                state["failsafe_armed"] = False

    threading.Thread(target=watchdog, daemon=True).start()
    if guard.enabled:
        threading.Thread(target=monitor, daemon=True).start()
        log(f"[resident] safety guard on: HOLD at {guard.wd.limits.hold_rate}/min, "
            f"LAND at {guard.wd.limits.land_rate}/min, ABORT flag {guard.flag}")
    log(f"[resident] holding link, token {token}, "
        f"failsafe {'off' if args.failsafe_s <= 0 else f'{args.failsafe_s:.0f}s'}")

    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        state["last_cmd"] = time.time()
        parts = line.split()
        verb = parts[0].upper()

        # Emergency words outrank the token dance: no run id, no plan, no argument.
        if verb in ("ABORT", "LAND", "HOLD", "KILL") and len(parts) == 1:
            with conn_lock:
                if verb == "KILL":
                    guard.kill("operator said KILL on stdin")
                else:
                    guard.operator(verb)
                    acted = guard.check()
            state["failsafe_armed"] = False
            continue

        if verb == "START":
            given = parts[1] if len(parts) > 1 else ""
            if given != token:
                emit(dict(verb="start", slot=args.slot, sysid=conn.target_system,
                          started=False,
                          error=f"token mismatch: given {given!r}, prepared {token!r}"))
                continue
            at = float(parts[2]) if len(parts) > 2 else None
            with conn_lock:
                r = do_start(conn, at)
            emit(dict(verb="start", slot=args.slot, sysid=conn.target_system, **r))
            state["last_cmd"] = time.time()
            state["failsafe_armed"] = bool(r.get("started"))
            if r.get("started"):
                state["armed_only"] = False

        elif verb == "ARM":
            given = parts[1] if len(parts) > 1 else ""
            if given != token:
                emit(dict(verb="arm", slot=args.slot, sysid=conn.target_system,
                          armed=False,
                          error=f"token mismatch: given {given!r}, prepared {token!r}"))
                continue
            at = float(parts[2]) if len(parts) > 2 else None
            with conn_lock:
                r = do_arm(conn, at)
            state["armed_only"] = bool(r.get("armed"))
            emit(dict(verb="arm", slot=args.slot, sysid=conn.target_system, **r))
            state["last_cmd"] = time.time()

        elif verb == "DISARM":
            with conn_lock:
                r = do_disarm(conn)
            if r.get("disarmed"):
                state["armed_only"] = False
            emit(dict(verb="disarm", slot=args.slot, sysid=conn.target_system, **r))

        elif verb == "STATUS":
            with conn_lock:
                st = do_status(conn)
            emit(dict(verb="status", slot=args.slot, sysid=conn.target_system, **st))

        elif verb == "ABORT":
            state["failsafe_armed"] = False
            action = parts[1] if len(parts) > 1 else "rtl"
            with conn_lock:
                ab = do_abort(conn, action)
            emit(dict(verb="abort", slot=args.slot, sysid=conn.target_system, **ab))

        elif verb == "QUIT":
            disarm_if_left_armed()
            emit(dict(verb="quit", slot=args.slot, sysid=conn.target_system))
            return 0

        else:
            emit(dict(verb="error", slot=args.slot, error=f"unknown command {verb!r}"))

    log("[resident] stdin closed")
    disarm_if_left_armed()
    return 0


def cmd_upload(args) -> int:
    """Put a plan on the aircraft and prove it is there. Never arms, never starts.

    Replaces whatever mission the vehicle holds. Refuses an armed vehicle. Success means
    the autopilot read the same mission back, not merely that it said ACK.
    """
    conn = connect(args.link, args.expect_sysid)
    plan_path = Path(args.plan)
    items, top_alt = plan_to_items(plan_path)
    log(f"[plan] {plan_path.name}: {len(items)} items, top altitude {top_alt:.1f} m")

    armed = vehicle_armed(conn)
    if armed is not False:
        raise RuntimeError("refusing to upload: vehicle is ARMED" if armed else
                           "refusing to upload: no heartbeat from the vehicle to check "
                           "its arming state")

    t0 = time.time()
    upload_mission(conn, items, timeout=max(MISSION_UPLOAD_TIMEOUT_S,
                                            UPLOAD_S_PER_ITEM * len(items)))
    upload_s = time.time() - t0
    log(f"[upload] {len(items)} items in {upload_s:.1f} s ({upload_s / len(items):.2f} s/item)")

    got = download_mission(conn)
    problems = (["could not read the mission back from the vehicle"] if got is None
                else diff_items(items, got))
    for pr in problems:
        log(f"[readback] {pr}")
    verified = not problems
    log(f"[readback] {'VERIFIED' if verified else 'FAILED'}: "
        f"{len(got) if got else 0} of {len(items)} items on the vehicle")
    emit(dict(verb="upload", slot=args.slot, sysid=conn.target_system, verified=verified,
              uploaded=len(items), onboard=None if got is None else len(got),
              problems=problems, upload_s=round(upload_s, 1), top_alt_m=top_alt,
              plan=str(plan_path),
              plan_sha=hashlib.sha256(plan_path.read_bytes()).hexdigest()[:12]))
    return 0 if verified else 1


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


def hold_for(at: float | None, what: str) -> None:
    if at:
        wait = at - time.time()
        if wait > 0:
            log(f"[{what}] holding {wait:.2f}s for the shared instant")
            time.sleep(wait)


def do_arm(conn, at: float | None = None) -> dict:
    """Arm and wait to SEE the armed heartbeat. Never starts the mission."""
    hold_for(at, "arm")
    reason, deadline, last_try = None, time.time() + 20, 0.0
    while time.time() < deadline:
        if time.time() - last_try > 3.0:
            conn.mav.command_long_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 0, 0, 0, 0, 0, 0)
            last_try = time.time()
        ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
        if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            reason = f"arm command result {ack.result}"
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.get_srcSystem() == conn.target_system \
                and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("[arm] armed")
            return dict(armed=True, t_epoch=time.time())
    return dict(armed=False, error="vehicle did not arm" + (f" ({reason})" if reason else ""))


def height_feed_check(stamps: list[float], now: float, timeout_s: float) -> tuple[bool, float]:
    """Can the controller be fed? (ok, longest gap in s) from the arrival times of LOCAL_POSITION_NED, counting the
    gap still open at `now`. The controller aborts the moment a reading is older than its timeout, so a feed whose
    longest gap exceeds the timeout cannot fly. 4 Nov 2026: over ELRS the readings came every 2 to 3.3 s against a
    3 s timeout, and the agent armed, then aborted 0.6 s later with "height estimate lost"."""
    pts = sorted(stamps)
    if not pts:
        return False, float("inf")
    worst = max([b - a for a, b in zip(pts, pts[1:])] + [now - pts[-1]])
    return worst < timeout_s, worst


LOCAL_POSITION_NED_ID = 32
RADIO_HEIGHT_FEED_HZ = 10
RADIO_SETPOINT_HZ = 3.0
WIRED_SETPOINT_HZ = 20.0


def default_setpoint_hz(link: str, sim: bool, given: float | None) -> float:
    """The offboard setpoint rate. An explicit --setpoint-hz always wins. Otherwise 20 Hz on a wire or in the simulator, and
    3 Hz over a radio: 4 Nov 2026, ARTEX113 over ELRS, aircraft disarmed, a harmless command's round trip while streaming
    setpoints: none 0.9 s median (10/10 acked); 20 Hz 5.6 s (3/10, seven got no answer in 15 s); 5 Hz 4.6 s (9/10); 3 Hz 2.6 s
    (8/8); 2 Hz 1.2 s (8/8); 1 Hz 2.3 s (8/8). The agent's own 20 Hz stream buried its ARM command and the ground-station
    heartbeat, which is why radio arms took 6 to 20+ s or failed."""
    if given is not None:
        return given
    wired = link.startswith(("serial:", "/dev/"))
    return WIRED_SETPOINT_HZ if (sim or wired) else RADIO_SETPOINT_HZ


def set_height_feed(conn, hz: float) -> None:
    """Ask the flight controller for LOCAL_POSITION_NED at `hz` (0 = back to its default rate). A runtime
    MAV_CMD_SET_MESSAGE_INTERVAL, not a parameter: it is not saved and a reboot clears it.

    Measured 4 Nov 2026 over ELRS, aircraft disarmed, 25 s each: the default gave 0.48 Hz (longest gap 2.8 s); asking for
    5 Hz gave 1.64 Hz (0.9 s); asking for 10 Hz gave 3.0 Hz (1.0 s). Total downlink stayed at about 25 messages/s, so the
    rate budget was going to streams nobody here reads. The setpoint rate made no difference to it (0.44 / 0.44 / 0.36 Hz
    for 0 / 20 / 5 Hz). The request crosses the same slow uplink, so it can take seconds to land."""
    conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                               LOCAL_POSITION_NED_ID, int(1e6 / hz) if hz else 0, 0, 0, 0, 0, 0)


def height_feed_ready(stamps: list[float], now: float, timeout_s: float, window_s: float = 6.0,
                      min_hz: float = 1.0) -> tuple[bool, float, int]:
    """Is the feed fast enough RIGHT NOW? Judged on the last `window_s` only, so a slow start does not count against it.
    Needs at least `min_hz` on average as well as no gap over the timeout, so one lucky window cannot pass a 0.45 Hz feed.
    Returns (ok, longest gap in the window, readings in the window)."""
    recent = [t for t in stamps if now - window_s <= t <= now]
    ok, worst = height_feed_check(recent, now, timeout_s)
    return (ok and len(recent) >= window_s * min_hz), worst, len(recent)


ARM_RETRY_RADIO_S = 8.0
ARM_RETRY_WIRED_S = 3.0
STALE_ARM_WATCH_S = 20.0


def arm_retry_s(link: str, sim: bool) -> float:
    """Seconds between ARM commands. 4 Nov 2026: over ELRS a command can take 5 s or more (15 s lost at 20 Hz), so a 3 s retry
    queued several copies, and the copies that arrive after the first has armed the aircraft arrive after the PILOT has disarmed
    it: operator saw a re-arm after the disarm on 3 of 5 runs at 20 Hz, none at 3 Hz. Retry slowly over a radio."""
    return ARM_RETRY_WIRED_S if (sim or link.startswith(("serial:", "/dev/"))) else ARM_RETRY_RADIO_S


def stale_arm_guard(pump, state, texts, send_disarm, seconds: float = STALE_ARM_WATCH_S) -> dict:
    """After the aircraft has been disarmed, watch for `seconds`. If it arms again and PX4 says "Armed by external command" (a
    command from a MAVLink link, i.e. a stale ARM that was still queued), send DISARM. "Armed by RC switch" is the pilot and is
    never fought; if the message was lost on the radio there is no action either. Never forces (no kill).
    `texts` returns the live list of PX4 status messages."""
    import re
    start_n, end = len(texts()), time.time() + seconds
    seen, sent = False, 0
    while time.time() < end:
        pump(1.0)
        if not state["armed"]:
            continue
        who = re.findall(r"(?<!dis)armed by ([a-z ]+)", " ".join(texts()[start_n:]).lower())
        if who and who[-1].startswith("external command"):
            seen = True
            send_disarm()
            sent += 1
    return dict(stale_arm_seen=seen, disarms_sent=sent, watched_s=seconds)


def withdraw_arm(send_disarm, pump, state, seconds: float = 10.0) -> dict:
    """The arm loop gave up -- but its ARM commands may still be queued in a slow radio link (4 Nov 2026: ARM sent at
    +17 s, the agent gave up at +38 s, and the aircraft armed after the agent had exited, unsupervised). Send DISARM
    behind them, in order, and keep listening, so a late arm is seen and undone rather than left behind.

    Returns armed_late (an armed heartbeat showed up after giving up) and armed_now. If armed_now is still true the
    caller's normal armed-exit handling takes over; this never forces a disarm (no kill)."""
    end, late = time.time() + seconds, False
    while time.time() < end:
        send_disarm()
        pump(1.0)
        late = late or bool(state["armed"])
        if late and not state["armed"]:
            break
    return dict(armed_late=late, armed_now=bool(state["armed"]))


def do_disarm(conn) -> dict:
    """Disarm and wait to SEE the disarmed heartbeat."""
    deadline, last_try = time.time() + 15, 0.0
    while time.time() < deadline:
        if time.time() - last_try > 3.0:
            conn.mav.command_long_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            last_try = time.time()
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.get_srcSystem() == conn.target_system \
                and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("[disarm] disarmed")
            return dict(armed=False, disarmed=True)
    return dict(armed=True, disarmed=False, error="vehicle did not disarm")


def do_start(conn, at: float | None) -> dict:
    """Arm, then MISSION_START. Returns the report; the caller decides how to emit it."""
    arm = do_arm(conn, at)
    if not arm["armed"]:
        return dict(started=False, armed=False, error=arm["error"])

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


MAV_LANDED_ON_GROUND = 1
MAV_LANDED_AIRBORNE = (2, 3, 4)       # IN_AIR, TAKEOFF, LANDING
ESTIMATOR_ATTITUDE_FLAG = 1
FORCE_DISARM_MAGIC = 21196            # MAVLink's "yes, I mean it" for disarming in flight
EMERGENCY_WORDS = ("ABORT", "LAND", "HOLD", "DISARM", "KILL")

# Hover height on a real aircraft (4 Nov 2026). Default cap 1.05 m as always; --max-height-m raises it, to at
# most 2.0 m, for the endurance hover. The controller's ceiling stays 0.15 m above whatever cap is in force.
REAL_DEFAULT_CAP_M = 1.05
REAL_ABS_MAX_M = 2.0
GEOFENCE_MARGIN_M = 0.25


def real_height_limits(height: float, max_height: float | None, gf_action, gf_max_ver) -> tuple[float | None, str | None]:
    """(controller ceiling, None) for a real aircraft, or (None, why not).

    PX4's own vertical geofence (GF_MAX_VER_DIST, acting by GF_ACTION) would take the aircraft away from
    the agent before the agent's ceiling does, so a hover within GEOFENCE_MARGIN_M of it is refused.
    Conservative: PX4 may not evaluate the fence without a home position, and the agent cannot tell."""
    cap = REAL_DEFAULT_CAP_M if not max_height else max_height
    if cap > REAL_ABS_MAX_M:
        return None, f"--max-height-m {cap:g} is above the {REAL_ABS_MAX_M:g} m limit"
    if height > cap:
        return None, (f"--height {height:g} is above the {cap:g} m cap"
                      + ("" if max_height else " (--max-height-m raises it, to 2.0 m at most)"))
    if gf_action and gf_max_ver and gf_max_ver > 0 and height + GEOFENCE_MARGIN_M > gf_max_ver:
        return None, (f"PX4's vertical geofence GF_MAX_VER_DIST is {gf_max_ver:g} m (GF_ACTION {int(gf_action)}): "
                      f"a {height:g} m hover is within {GEOFENCE_MARGIN_M:g} m of it and PX4 would act first")
    return round(cap + 0.15, 2), None

# PX4's own words for the pilot's kill switch. These are the OPERATOR stopping the
# aircraft, not the aircraft failing, so they must neither count as accumulated error nor
# provoke the guard into commanding anything. Proven necessary on ARTEX113, 9 Oct 2026:
# a kill produced two CRITICAL messages, which alone tripped the ladder to HOLD and had
# the guard commanding AUTO.LOITER at an aircraft the pilot had just deliberately stopped.
# Note "Kill disengaged" does not contain "kill engaged", so it does not match.
PILOT_KILL_TEXTS = ("kill engaged", "kill switch engaged", "flight termination active",
                    "disarmed by kill-switch")

# PX4 main modes a human is flying by hand: MANUAL, ALTCTL, POSCTL, ACRO, STABILIZED,
# RATTITUDE. Not AUTO (4) and not OFFBOARD (6), which are ours.
PILOT_MODES = (1, 2, 3, 5, 7, 8)
AUTOMATIC_MODES = (4, 6)
OFFBOARD_MAIN_MODE = 6
INNOVATION_FIELDS = ("vel_ratio", "pos_horiz_ratio", "pos_vert_ratio",
                     "mag_ratio", "hagl_ratio")


def statustext_of(msg) -> str:
    text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
    return text.strip()


def limits_from_args(args) -> safety_rules.Limits:
    lim = safety_rules.Limits()
    for attr, name in (("hold_rate", "hold_rate"), ("land_rate", "land_rate"),
                       ("battery_hold_pct", "battery_hold_pct"), ("battery_land_pct", "battery_land_pct"),
                       ("tilt_land_deg", "tilt_land_deg"),
                       ("cut_after_s", "cut_after_s")):
        given = getattr(args, attr, None)
        if given is not None:
            setattr(lim, name, given)
    ceiling = getattr(args, "ceiling_m", None)
    if ceiling is not None:
        lim.ceiling_m = ceiling if ceiling > 0 else None
    lim.cut_enabled = bool(getattr(args, "allow_kill", False))   # stage 2 is a kill
    return lim


def parse_motor_range(text: str | None) -> tuple[float, float]:
    """'min,max' of SERVO_OUTPUT_RAW for this platform. DShot reports 0..1999 (the real
    ARTEX, UNCONFIRMED props-off); the Gazebo model reports rotor speed 150..2939 rad/s."""
    lo, hi = (float(x) for x in (text or "0,1999").split(","))
    return lo, hi


class Guard:
    """Watches one vehicle and carries out the abort ladder. See tools/safety_rules.py.

    Three ways a flight can be stopped, in rising order of how much has to still be
    working for them to help:

      1. the RC transmitter -- the authoritative one, and nothing here can match it
      2. this guard, on the companion: autonomous, needs only the companion-to-aircraft link
      3. the operator on the ground: a word on stdin, or the out-of-band ABORT flag file

    Non-blocking by design. carry_out() sends a command and returns; the caller's normal
    telemetry loop keeps running, and the landing is finished across later check() calls.
    A guard that blocked would stop feeding itself the telemetry it is judging.
    """

    def __init__(self, conn, args):
        self.conn = conn
        self.slot = getattr(args, "slot", None)
        self.enabled = not getattr(args, "no_safety", False)
        self.wd = safety_rules.Watchdog(limits_from_args(args), now=time.time())
        self.flag = Path(getattr(args, "abort_flag", "ABORT") or "ABORT")
        self.allow_kill = getattr(args, "allow_kill", False)
        self.armed: bool | None = None
        self.airborne: bool | None = None
        self._seen_landed_state = False
        self.stop_streaming = False          # the offboard streamer checks this
        self.acted: str | None = None
        self.landing_since: float | None = None
        self._last_disarm = 0.0
        self.killed = False
        self.pilot_has_it = False       # the RC switch has been thrown: hands off
        self.killed_by_pilot = False
        self.handback_requests = 0       # times the switch asked for OFFBOARD after a takeover
        self._prev_mode: int | None = None
        self.motor_lo, self.motor_hi = parse_motor_range(getattr(args, "motor_range", None))
        self.throttle_chan = getattr(args, "throttle_chan", 3) or 3
        self._range_at = 0.0                 # last valid rangefinder reading
        # Estimator z at the flying verb's height zero, for the fallback below the rangefinder's
        # 0.4 m: the EKF origin is set at boot and drifts, so raw -z is not height above the floor.
        self.z0: float | None = None
        # offboard-flight sets this: until its zero is taken, the estimator is not a height at all.
        # 3 Nov 2026: 45 min after boot raw -z read 4.92 m on the floor, and the ceiling rule
        # commanded LAND the moment the agent connected (props-off Run A, never armed).
        self.zero_required = False
        self.batt_v: float | None = None          # volts, from SYS_STATUS (None = unknown)
        self.batt_pct: float | None = None        # PX4's voltage-based estimate; no current sensor on ARTEX113
        # Takeover classification (29 Oct 2026): which way the aircraft left OFFBOARD.
        self.mode_chan = getattr(args, "mode_chan", 7) or 7
        self.offboard_chan = getattr(args, "offboard_chan", 5) or 5      # since 3 Nov 2026
        self.kill_chan = getattr(args, "kill_chan", 8) or 8
        self.kill_pwm_above = getattr(args, "kill_pwm_above", 1750) or 1750
        self.classify_s = 1.0                # wait this long for an RC reading after the exit
        self._mode_pwm: list[tuple[float, int]] = []
        self._offb_pwm: list[tuple[float, int]] = []
        self._exit_at: float | None = None
        self.takeover_source: str | None = None    # pilot_switch | kill_switch | link_loss | unknown
        self.link_losses = 0
        self.stick_override: float | None = None   # the agent's own climb intent, while it flies
        # A flag that predates the run is an abort nobody cleared. Refuse to fly rather
        # than guess which it was, and say which case this is -- the operator needs to
        # know whether the abort happened just now or last time.
        self._flag_at_start = self.flag.exists()

    # ---- telemetry in -------------------------------------------------------------

    def observe(self, msg) -> None:
        now, kind = time.time(), msg.get_type()
        if kind == "HEARTBEAT":
            if msg.get_srcSystem() == self.conn.target_system \
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.wd.heartbeat(now)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.wd.armed(self.armed, now)
                self._note_mode(vehicle_main_mode(msg))
        elif kind == "STATUSTEXT":
            text = statustext_of(msg)
            if any(k in text.lower() for k in PILOT_KILL_TEXTS):
                self._note_kill(text)
                return                  # the operator's own act: never scored as a fault
            self.wd.statustext(msg.severity, text, now)
        elif kind == "BATTERY_STATUS":
            self.wd.battery(getattr(msg, "battery_remaining", None), now)
        elif kind == "SYS_STATUS":
            # for the endurance log only; the rules above use BATTERY_STATUS
            mv, pct = getattr(msg, "voltage_battery", 0), getattr(msg, "battery_remaining", -1)
            self.batt_v = mv / 1000.0 if 0 < mv < 65535 else None
            self.batt_pct = pct if 0 <= pct <= 100 else None
        elif kind == "ATTITUDE":
            self.wd.attitude(math.degrees(msg.roll), math.degrees(msg.pitch), now)
        elif kind == "EXTENDED_SYS_STATE":
            if msg.landed_state == MAV_LANDED_ON_GROUND:
                self.airborne, self._seen_landed_state = False, True
            elif msg.landed_state in MAV_LANDED_AIRBORNE:
                self.airborne, self._seen_landed_state = True, True
            self.wd.airborne(self.airborne, now)
        elif kind == "DISTANCE_SENSOR":
            d = msg.current_distance
            if msg.min_distance < d < msg.max_distance:      # 65535 / 0 = no return
                self._range_at = now
                self.wd.vertical(d / 100.0, None, now)
        elif kind == "LOCAL_POSITION_NED":
            # height from the estimator only while the rangefinder is silent (below 0.4 m)
            fallback = -(msg.z - (self.z0 or 0.0)) if now - self._range_at > 1.0 else None
            if self.zero_required and self.z0 is None:
                fallback = None
            self.wd.vertical(fallback, -msg.vz, now)
        elif kind == "RC_CHANNELS":
            if self.stick_override is None:
                raw = getattr(msg, f"chan{self.throttle_chan}_raw", 0)
                self.wd.stick((raw - 1000) / 1000.0 if 900 < raw < 2100 else None, now)
            mode_pwm = getattr(msg, f"chan{self.mode_chan}_raw", 0)
            if 900 < mode_pwm < 2100:
                self._mode_pwm = self._mode_pwm[-49:] + [(now, mode_pwm)]
            offb_pwm = getattr(msg, f"chan{self.offboard_chan}_raw", 0)
            if 900 < offb_pwm < 2100:
                self._offb_pwm = self._offb_pwm[-49:] + [(now, offb_pwm)]
            kill_pwm = getattr(msg, f"chan{self.kill_chan}_raw", 0)
            if 900 < kill_pwm < 2100 and kill_pwm > self.kill_pwm_above:
                # STATUSTEXT can be lost on the radio; the switch position itself cannot lie
                self._note_kill(f"kill switch on (channel {self.kill_chan} = {kill_pwm})")
        elif kind == "SERVO_OUTPUT_RAW":
            span = self.motor_hi - self.motor_lo
            outs = [getattr(msg, f"servo{i}_raw") for i in range(1, 5)]
            if self.armed and span > 0:
                self.wd.motors([max(0.0, (v - self.motor_lo) / span) for v in outs], now)
        elif kind == "GLOBAL_POSITION_INT" and not self._seen_landed_state:
            self.airborne = (msg.relative_alt / 1000.0) > 1.0
        elif kind == "ESTIMATOR_STATUS":
            worst = max((getattr(msg, f, 0.0) or 0.0) for f in INNOVATION_FIELDS)
            self.wd.estimator(bool(msg.flags & ESTIMATOR_ATTITUDE_FLAG), worst, now)

    def _note_kill(self, text: str) -> None:
        """The pilot hit the kill switch. Same answer as any takeover: hands off."""
        if self.killed_by_pilot:
            return
        self.killed_by_pilot = True
        self.pilot_has_it = True        # reuse every gate the mode takeover already has
        if self.takeover_source in (None, "unknown"):
            self.takeover_source = "kill_switch"
        self.stop_streaming = True
        log(f"[safety] PILOT KILLED IT ({text}). Standing down -- no setpoints, no mode "
            "changes, no commands.")
        emit(dict(event="rc_takeover", slot=self.slot, via="kill_switch", text=text,
                  airborne=self.airborne, t_epoch=round(time.time(), 3)))

    def _note_mode(self, mode: int) -> None:
        """Spot the pilot taking the aircraft, and get out of the way for good.

        The signal is the TRANSITION out of one of our modes into a hand-flown one -- not
        the mode itself. An aircraft sitting in POSCTL before we ask for OFFBOARD has not
        been taken over; one that was in OFFBOARD a moment ago and is now in POSCTL has.
        """
        prev, self._prev_mode = self._prev_mode, mode

        # The pilot flipped from a hand-flown mode to OFFBOARD (channel 5 on since 3 Nov
        # 2026; channel 7 centre from 14 Oct). That is a request to hand the aircraft
        # BACK to the agent. The agent refuses, deliberately and out loud: it streams
        # level attitude at ZERO thrust and has no height hold, so resuming on an airborne
        # aircraft would command a fall. NOTE PX4 does not deny the mode. Measured on
        # ARTEX113, 14 Oct 2026, disarmed with nothing streaming: "navigation mode:
        # Offboard, in failsafe: no". Armed, the offboard-loss failsafe (COM_OF_LOSS_T,
        # action COM_OBL_RC_ACT, both aircraft = Position after 1 s) takes it out instead.
        if (self.pilot_has_it and not self.killed_by_pilot and prev in PILOT_MODES
                and mode == OFFBOARD_MAIN_MODE):
            self.handback_requests += 1
            log("[safety] switch requested OFFBOARD after a pilot takeover. The agent is NOT "
                "resuming: offboard flight is not implemented (the stream is zero thrust). "
                "PX4 will still select OFFBOARD; with no setpoints and the aircraft armed, "
                "its offboard-loss failsafe acts after COM_OF_LOSS_T (COM_OBL_RC_ACT).")
            emit(dict(event="rc_handback_requested", slot=self.slot, from_mode=prev,
                      to_mode=mode, resumed=False, airborne=self.airborne,
                      t_epoch=round(time.time(), 3)))
            return

        if self.pilot_has_it or prev is None:
            return
        if prev in AUTOMATIC_MODES and mode in PILOT_MODES:
            # Hands off at once, whatever the cause -- sending nothing is right whether the
            # pilot flipped the switch or the link dropped the setpoints. WHICH it was is
            # settled a moment later by _classify, from the mode switch's position.
            self.pilot_has_it = True
            self.stop_streaming = True      # stop arguing with the sticks
            self._exit_at = time.time()
            log(f"[safety] LEFT OUR MODE: main mode {prev} -> {mode}. "
                "Standing down -- no setpoints, no mode changes, no disarm while airborne.")
            emit(dict(event="rc_takeover", slot=self.slot, from_mode=prev, to_mode=mode,
                      airborne=self.airborne, t_epoch=round(time.time(), 3)))

    def _classify(self, now: float) -> None:
        """Pilot switch or lost link? Decided from ch5/ch7 readings that arrived after the exit."""
        if self._exit_at is None or self.takeover_source is not None:
            return
        before = [p for t, p in self._mode_pwm if t < self._exit_at]
        verdict = safety_rules.classify_mode_exit(
            [p for t, p in self._offb_pwm if t >= self._exit_at], before[-1] if before else None,
            [p for t, p in self._mode_pwm if t >= self._exit_at])
        if verdict != "pilot_switch" and now - self._exit_at < self.classify_s:
            return                      # a switch reading may still be on its way
        self.takeover_source = verdict
        if verdict == "link_loss":
            self.link_losses += 1
            log("[safety] *** OFFBOARD LOST ON THE LINK -- NOT A PILOT TAKEOVER *** the offboard "
                "switch (ch5) is still on and ch7 has not moved. PX4 failed over and the agent has "
                "stood down for good. PILOT: you have the aircraft -- fly it and land it.")
        else:
            log(f"[safety] PILOT HAS IT ({verdict.replace('_', ' ')}).")
        emit(dict(event="takeover_classified", slot=self.slot, source=verdict,
                  after_s=round(now - self._exit_at, 2), airborne=self.airborne,
                  t_epoch=round(now, 3)))

    def operator(self, word: str) -> bool:
        """A command from the ground. Returns False if it was not understood."""
        word = word.strip().upper()
        if word in ("ABORT", "LAND"):
            self.wd.manual(safety_rules.LAND, f"operator said {word}")
        elif word == "HOLD":
            self.wd.manual(safety_rules.HOLD, "operator said HOLD")
        elif word == "DISARM":
            self.wd.manual(safety_rules.DISARM, "operator said DISARM")
        elif word == "KILL":
            if not self.allow_kill:
                log("[safety] KILL refused: the agent was not started with --allow-kill")
                return False
            self.kill("operator said KILL")
        else:
            return False
        return True

    # ---- the verdict, and acting on it --------------------------------------------

    def check(self) -> dict | None:
        """Evaluate, act if the ladder has climbed, and nurse a landing to its end."""
        self._classify(time.time())
        if not self.enabled or self.pilot_has_it:
            # A human is flying it. Two aborts fighting each other is worse than either.
            return None
        if self.flag.exists():
            want = ""
            try:
                want = self.flag.read_text().strip().lower()
            except OSError:
                pass
            action = want if want in safety_rules.RUNG else safety_rules.LAND
            how = " left over from before this run" if self._flag_at_start else ""
            self.wd.manual(action, f"ABORT flag {self.flag}{how}"
                                   + (f" ({want})" if want else ""))

        v = self.wd.evaluate(time.time())
        if v.climbed:
            log(f"[safety] {v.action.upper()}: " + "; ".join(v.reasons[-2:])
                + f" (error rate {v.error_rate:.1f}/min)")
        left = self.wd.cut_in(time.time())
        if left is not None and self.airborne is not False and int(left) != getattr(self, "_cd", None):
            self._cd = int(left)      # once a second: the pilot's window to take over
            log(f"[safety] DESCENDING -- CUT IN {left:.0f}s unless the pilot takes over (ch7)")
            emit(dict(event="cut_pending", slot=self.slot, seconds=round(left, 1),
                      t_epoch=round(time.time(), 3)))
        self._finish_landing()
        if safety_rules.RUNG[v.action] >= safety_rules.RUNG[safety_rules.HOLD] \
                and v.action != self.acted:
            return self.carry_out(v.action, v.reasons)
        return None

    def carry_out(self, action: str, reasons: list[str]) -> dict:
        real = safety_rules.on_ground_action(action, self.airborne)
        where = {True: "airborne", False: "on the ground", None: "position unknown"}[self.airborne]
        log(f"[safety] acting: {action.upper()} -> {real.upper()} ({where})")
        self.acted = action
        if real == safety_rules.HOLD:
            # AUTO.LOITER: main mode 4, sub mode 3. Stops the mission, holds position.
            self.conn.mav.command_long_send(self.conn.target_system, 1,
                                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                            1, 4, 3, 0, 0, 0, 0)
        elif real == safety_rules.LAND:
            self.stop_streaming = True      # offboard setpoints would fight AUTO.LAND
            self.conn.mav.command_long_send(self.conn.target_system, 1,
                                            mavutil.mavlink.MAV_CMD_NAV_LAND,
                                            0, 0, 0, 0, 0, 0, 0, 0)
            self.landing_since = time.time()
        elif real == safety_rules.DISARM:
            self.stop_streaming = True
            self.landing_since = time.time()
            self._send_disarm()
        elif real == safety_rules.CUT:
            if self.allow_kill:
                return self.kill("; ".join(reasons[-2:]))
            # unreachable through the rules (cut_enabled follows --allow-kill), kept as a seatbelt
            log("[safety] CUT refused without --allow-kill: landing instead")
            self.conn.mav.command_long_send(self.conn.target_system, 1,
                                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            real = safety_rules.LAND
        ev = dict(event="safety", slot=self.slot, action=action, carried_out=real,
                  airborne=self.airborne, reasons=reasons,
                  error_rate=round(self.wd.error_rate(time.time()), 2),
                  t_epoch=round(time.time(), 3))
        emit(ev)
        return ev

    def _send_disarm(self) -> None:
        self.conn.mav.command_long_send(self.conn.target_system, 1,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        self._last_disarm = time.time()

    def _finish_landing(self) -> None:
        """Once it is down, cut the motors. Never while it is still flying."""
        if self.landing_since is None or self.armed is False:
            return
        if self.airborne is not False:
            return
        if time.time() - self._last_disarm > 3.0:
            log("[safety] on the ground -- disarming")
            self._send_disarm()

    def kill(self, reason: str) -> dict:
        """Force-disarm, in flight if need be. This drops the aircraft. Last resort."""
        log(f"[safety] KILL ({reason}) -- force disarm, the aircraft will fall")
        self.stop_streaming = self.killed = True
        self.conn.mav.command_long_send(self.conn.target_system, 1,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 0, FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0)
        ev = dict(event="safety", slot=self.slot, action="kill", carried_out="kill",
                  reasons=[reason], t_epoch=round(time.time(), 3))
        emit(ev)
        return ev


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


def stream_level_zero_thrust(conn) -> None:
    """One SET_ATTITUDE_TARGET: level, zero thrust, rates ignored. Offboard needs >2 Hz."""
    kw = dict(thrust_body=[0.0, 0.0, 0.0]) if _ATT_HAS_BODY_THRUST else {}
    conn.mav.set_attitude_target_send(int(time.monotonic() * 1000) & 0xFFFFFFFF,
                                      conn.target_system, 1, 0b00000111,
                                      [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, 0.0, **kw)


_ATT_HAS_BODY_THRUST = "thrust_body" in mavutil.mavlink.MAVLink_set_attitude_target_message.fieldnames


def vehicle_main_mode(hb) -> int:
    return (hb.custom_mode >> 16) & 0xFF


def disarm_unless_pilot(send_disarm, pump, state, guard, attempts: int = 6) -> str:
    """Send disarm commands until the aircraft is disarmed -- or the pilot has it.

    Returns "disarmed", "pilot" or "failed".

    THE RACE THIS CLOSES. On 15 Oct 2026 the guard's own error-rate abort ended an
    offboard-arm hold, the agent started this loop, and the pilot's kill arrived while it
    was running. The old loop kept sending disarms regardless, contradicting Option C
    (after a takeover or kill the agent sends NOTHING). The check is before every send and
    the wait ends early the moment the pilot takes over. One disarm may already be on the
    wire when the kill lands; that cannot be recalled, but no further one is sent.
    """
    for _ in range(attempts):
        if not state["armed"]:
            return "disarmed"        # already off (the pilot's arm switch, or PX4's own timer)
        if guard.pilot_has_it:
            return "pilot"
        send_disarm()
        pump(2.0, lambda: (not state["armed"]) or guard.pilot_has_it)
        if not state["armed"]:
            return "disarmed"
    return "pilot" if guard.pilot_has_it else "failed"


HEARTBEAT_FRESH_S = 3.0


def confirmed_disarmed(state, now: float) -> bool:
    """Disarmed only if the AIRCRAFT says so, recently. A silent link is not a disarm."""
    last = state.get("last_hb")
    return (state.get("armed") is False and last is not None
            and now - last <= HEARTBEAT_FRESH_S)


def hold_until_disarmed(send_disarm, pump, state, guard, *, window_s: float = 120.0,
                        retry_s: float = 2.0, warn_every_s: float = 5.0,
                        slot=None, clock=time.time) -> dict:
    """The aircraft is still armed and the agent is about to stop. Do not just leave.

    15 Oct 2026: the agent printed "disarming", none of its disarms reached PX4, and it exited
    with ARTEX113 armed; PX4 then failed over to LAND on the bench and the motors ran away.
    This keeps the link (and the heartbeat thread) alive, keeps sending disarms, and keeps
    telling the operator, until the aircraft CONFIRMS it is disarmed, the pilot takes over,
    or the window ends. Returns dict(outcome, waited_s, attempts).
    """
    t0, last_send, last_warn, attempts = clock(), None, None, 0
    while clock() - t0 < window_s:
        now = clock()
        if confirmed_disarmed(state, now):
            return dict(outcome="disarmed", waited_s=now - t0, attempts=attempts)
        if guard.pilot_has_it:
            return dict(outcome="pilot", waited_s=now - t0, attempts=attempts)
        if last_send is None or now - last_send >= retry_s:
            send_disarm(); attempts += 1; last_send = now
        if last_warn is None or now - last_warn >= warn_every_s:
            last_warn = now
            log("[safety] *** AIRCRAFT IS STILL ARMED *** the agent's disarm has not been "
                "confirmed. Use the ARM SWITCH (channel 6) or the KILL SWITCH (channel 8).")
            emit(dict(event="still_armed", slot=slot, waited_s=round(now - t0, 1),
                      disarm_attempts=attempts))
        pump(0.5)
    return dict(outcome="left_armed", waited_s=clock() - t0, attempts=attempts)


def await_pilot_disarm(pump, state, *, killed: bool, slot, window_s: float,
                       warn_every_s: float = 5.0, clock=time.time) -> dict:
    """After a pilot takeover: send NOTHING, say loudly that it is armed, wait for the pilot.

    Decided 14 Oct 2026 (option C). The agent used to try to disarm on the ground after a
    takeover. That fights a pilot who may be about to fly, and it was also unreliable --
    the disarm took 2 s in one run and failed for 12+ s in the next -- so it could leave the
    aircraft armed anyway, silently. Now the decision is the pilot's, and the agent's whole
    job is to make sure the pilot cannot miss that the aircraft is still armed. The ways to
    disarm are the arm switch (channel 6) and the kill switch (channel 8); neither needs us.

    `pump(seconds)` reads telemetry and updates state["armed"]. Nothing here sends a command.
    """
    t0 = clock()
    if not state["armed"]:
        return dict(disarmed=True, waited_s=0.0)
    if killed:
        log("[safety] kill switch engaged -- PX4 disarms by itself after COM_KILL_DISARM. "
            "The agent sends nothing.")
    last_warn = None
    while clock() - t0 < window_s:
        waited = clock() - t0
        if not state["armed"]:
            log(f"[safety] aircraft disarmed (by the pilot or PX4) after {waited:.1f} s")
            emit(dict(event="pilot_disarmed", slot=slot, waited_s=round(waited, 1)))
            return dict(disarmed=True, waited_s=waited)
        if not killed and (last_warn is None or clock() - last_warn >= warn_every_s):
            last_warn = clock()
            log("[safety] *** AIRCRAFT IS STILL ARMED *** The pilot has it and the agent "
                "will not disarm it. Disarm with the ARM SWITCH (channel 6) or the KILL "
                "SWITCH (channel 8).")
            emit(dict(event="still_armed", slot=slot, waited_s=round(waited, 1)))
        pump(0.5)
    waited = clock() - t0
    log(f"[safety] *** LEAVING THE AIRCRAFT ARMED *** nobody disarmed it in {window_s:.0f} s "
        "and the agent will not. It stays armed until the arm switch or kill switch is used.")
    emit(dict(event="left_armed", slot=slot, waited_s=round(waited, 1)))
    return dict(disarmed=False, waited_s=waited)


def cmd_offboard_arm(args) -> int:
    """Bench test, no GPS needed: stream level/zero-thrust, enter OFFBOARD, arm, hold, disarm.

    Props must be off. Always ends with a disarm attempt, whatever happened."""
    OFFBOARD = 6
    conn = connect(args.link, args.expect_sysid)
    result = dict(verb="offboard-arm", slot=args.slot, sysid=conn.target_system,
                  offboard=False, armed=False, held_s=0.0, disarmed=False)
    armed_now = vehicle_armed(conn)
    if armed_now is not False:
        result["error"] = "refusing: vehicle is ARMED or silent"
        emit(result)
        return 2
    wo_gps = read_param(conn, "COM_ARM_WO_GPS")
    result["com_arm_wo_gps"] = wo_gps
    log(f"[offboard] COM_ARM_WO_GPS={wo_gps}")

    state = dict(armed=False, mode=None)
    stdin = dict(pending=[], eof=False)
    guard = Guard(conn, args)

    def pump(seconds: float, until=None) -> bool:
        end = time.time() + seconds
        while time.time() < end:
            if not guard.stop_streaming:
                stream_level_zero_thrust(conn)
            msg = conn.recv_match(blocking=True, timeout=0.05)
            while msg is not None:
                kind = msg.get_type()
                if kind == "HEARTBEAT" and msg.get_srcSystem() == conn.target_system \
                        and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    state["mode"] = vehicle_main_mode(msg)
                    state["last_hb"] = time.time()
                elif kind == "STATUSTEXT":
                    text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
                    log(f"[vehicle] {text.strip()}")
                    result.setdefault("vehicle_text", []).append(text.strip())
                elif kind == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    result["arm_result"] = msg.result
                    log(f"[vehicle] arm/disarm ack result {msg.result}")
                guard.observe(msg)
                msg = conn.recv_match(blocking=False)
            # The operator can speak at any time, not only at the arming gate. Words the
            # guard does not own (the gate's own ARM) are queued for whoever is waiting,
            # never swallowed here.
            while select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    stdin["eof"] = True
                    break
                word = line.strip().upper()
                if not word:
                    continue
                if word in EMERGENCY_WORDS:
                    guard.operator(word)
                    result.setdefault("operator", []).append(word)
                else:
                    stdin["pending"].append(word)
            acted = guard.check()
            if acted:
                result.setdefault("safety", []).append(acted)
                result["error"] = "safety abort: " + "; ".join(acted["reasons"][-2:])
            if until and until():
                return True
            time.sleep(0.05)
        return until is None

    try:
        log("[offboard] streaming setpoints")
        pump(2.0)
        log("[offboard] requesting OFFBOARD")
        last = 0.0
        deadline = time.time() + 10
        while time.time() < deadline and state["mode"] != OFFBOARD and not guard.pilot_has_it:
            if time.time() - last > 2.0:
                conn.mav.command_long_send(conn.target_system, 1,
                                           mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                           1, OFFBOARD, 0, 0, 0, 0, 0)
                last = time.time()
            pump(0.5, lambda: state["mode"] == OFFBOARD)
        result["offboard"] = state["mode"] == OFFBOARD
        if not result["offboard"]:
            result["error"] = f"vehicle did not enter OFFBOARD (main mode {state['mode']})"
            return 2
        if args.gate:
            emit(dict(event="ready", slot=args.slot, sysid=conn.target_system))
            log("[offboard] READY -- waiting for workstation authorisation (ARM)")
            end, verdict = time.time() + args.gate_timeout_s, None
            while time.time() < end and verdict is None:
                pump(0.3)
                if stdin["pending"]:
                    verdict = stdin["pending"].pop(0)
                elif stdin["eof"]:
                    verdict = "EOF"
                elif guard.acted or guard.killed:
                    verdict = "ABORTED"
            if verdict != "ARM":
                result["error"] = f"not authorised to arm ({verdict or 'timeout'})"
                return 2
            if guard.acted or guard.killed:
                result["error"] = "refusing to arm: the safety guard has already aborted"
                return 2
            log("[offboard] authorised")
        log("[offboard] in OFFBOARD; arming")
        last = 0.0
        deadline = time.time() + 20
        while time.time() < deadline and not state["armed"]:
            if time.time() - last > 3.0:
                conn.mav.command_long_send(conn.target_system, 1,
                                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 1, 0, 0, 0, 0, 0, 0)
                last = time.time()
            pump(0.5, lambda: state["armed"])
        result["armed"] = state["armed"]
        if not state["armed"]:
            result["error"] = "vehicle did not arm"
            return 2
        t0 = time.time()
        result["armed_epoch"] = round(t0, 3)
        # Emitted the instant arming is observed, so the workstation can stamp it against
        # its own clock. Companion clocks are not trustworthy: a Jetson with no usable
        # default route never syncs NTP (jetson-1 sat at 1970-01-01 on 6 Oct 2026), which
        # makes any spread computed from armed_epoch meaningless.
        emit(dict(event="armed", slot=args.slot, sysid=conn.target_system,
                  armed_epoch=result["armed_epoch"]))
        log(f"[offboard] ARMED; holding {args.hold_s:.0f} s")
        while time.time() - t0 < args.hold_s:
            pump(0.5)
            if guard.acted or guard.killed:
                break
            if not state["armed"]:
                result["error"] = ("vehicle disarmed during hold -- by the pilot's arm switch, the kill "
                                   "switch or PX4's own timer; see vehicle_text")
                break
            if state["mode"] != OFFBOARD:
                result["error"] = ("pilot took over with the RC switch"
                                   if guard.pilot_has_it else
                                   f"left OFFBOARD during hold (main mode {state['mode']})")
                break
        result["held_s"] = round(time.time() - t0, 1)
        return 0 if "error" not in result else 2
    finally:
        if not guard.pilot_has_it:
            log("[offboard] disarming")
            send_disarm = lambda: conn.mav.command_long_send(  # noqa: E731
                conn.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0)
            disarm_unless_pilot(send_disarm, pump, state, guard)
            if not guard.pilot_has_it and not confirmed_disarmed(state, time.time()):
                h = hold_until_disarmed(send_disarm, pump, state, guard,
                                        window_s=args.armed_exit_window_s, slot=args.slot)
                result["disarm_attempts_after_hold"] = h["attempts"]
                if h["outcome"] == "left_armed":
                    result["left_armed"] = True
                    log(f"[safety] *** EXITING WITH THE AIRCRAFT STILL ARMED *** no disarm "
                        f"confirmed in {h['waited_s']:.0f} s. Disarm it NOW with the arm "
                        f"switch or the kill switch.")
                    emit(dict(event="left_armed", slot=args.slot,
                              waited_s=round(h["waited_s"], 1)))
        result["heartbeat_send_failures"] = getattr(conn, "hb_failures", None)
        result["rc_takeover"] = guard.pilot_has_it
        if guard.pilot_has_it:
            w = await_pilot_disarm(pump, state, killed=guard.killed_by_pilot,
                                   slot=args.slot, window_s=args.pilot_disarm_window_s)
            result["disarmed"] = w["disarmed"]
            result["left_armed"] = not w["disarmed"]
            result["pilot_disarm_wait_s"] = round(w["waited_s"], 1)
            emit(result)
            return 2 if "error" in result else 0
        result["disarmed"] = confirmed_disarmed(state, time.time())
        emit(result)
        if result.get("left_armed"):
            return 4


def cmd_offboard_flight(args) -> int:
    """The takeover ladder (29 Oct 2026, docs/takeover-ladder-plan.md): fly in OFFBOARD up to
    --stage, hold that stage open for the pilot to take over, and report how it went.

        prearm   OFFBOARD selected, streaming, disarmed       -> kill switch expected
        arm      + armed at zero thrust                       -> kill switch expected
        takeoff  + climb to a 0.3 m pause                      -> kill switch expected
        hold     + 1 m for the wait window                    -> ch7 to Altitude expected
        descent  + a slow descent                             -> ch7 to Altitude expected
        landing  + a slow final 0.3 m, touchdown by PX4 LAND  -> ch7 to Altitude expected
        disarm   + landed, before PX4's own auto-disarm       -> kill switch expected

    The height loop is offboard_control.HeightHold. Indoors PX4 takes only attitude + thrust,
    so the agent closes the height loop itself -- over ELRS that loop crosses the radio, with
    gaps of over a second (19 Oct). A gap trips PX4's offboard-loss failsafe into Altitude;
    the guard stands down for that exactly as for a pilot, and classifies which it was.

    Real aircraft only with --real-aircraft, --expect-sysid and, for any stage that arms,
    --gate (the operator types ARM after confirming props / tether / area). Never sends a
    disarm the aircraft would accept in the air, and nothing at all after a takeover.
    """
    import offboard_control as oc
    stage = args.stage
    upto = oc.STAGES.index(stage)
    conn = connect(args.link, args.expect_sysid)
    result = dict(verb="offboard-flight", stage=stage, slot=args.slot, sysid=conn.target_system,
                  outcome=None, takeover_source=None, takeover_stage=None)
    sim = read_param(conn, "SIM_GZ_EN") is not None
    result["simulator"] = sim
    args.setpoint_hz = default_setpoint_hz(args.link, sim, args.setpoint_hz)
    if not sim:
        need = [("--real-aircraft", args.real_aircraft), ("--expect-sysid", args.expect_sysid)]
        if upto >= oc.STAGES.index("arm"):
            need.append(("--gate", args.gate))
        missing = [flag for flag, ok in need if not ok]
        # 29 Oct 2026: the thrust-only height loop cannot be closed across the radio. Offline,
        # with ELRS's ~0.35 s delay at 5 Hz, no gain set survived a +/-0.05 hover-thrust error
        # (through the ceiling or onto the floor); in SITL, 5 Hz alone oscillated to 1.54 m
        # where 50 Hz held +/-1 cm. Flight stages need the companion WIRED to the FC.
        # 3 Nov 2026: --allow-radio-flight lifts this, by name, for props-off tests of the plumbing
        # over the radio (authorised by the lab manager through the operator). It proves nothing
        # about the loop: with props off nothing lifts, so the height feedback is never exercised.
        wired = args.link.startswith(("serial:", "/dev/"))
        if upto >= oc.STAGES.index("takeoff") and not wired and not args.allow_radio_flight:
            missing.append("a wired link (--link serial:/dev/...) for flight stages: the "
                           "height loop is unstable over ELRS (docs/history.md, 29 Oct 2026)")
        if missing or args.sim_outage:
            result["error"] = ("refusing a real aircraft: " +
                               (f"needs {', '.join(missing)}" if missing else
                                "--sim-outage is simulator-only"))
            emit(result)
            return 2
    if sim:
        ceiling = 1.5
        err = f"--height {args.height} too close to the {ceiling} m ceiling" if args.height > ceiling - 0.15 else None
    else:
        gf_act, gf_max = read_param(conn, "GF_ACTION"), read_param(conn, "GF_MAX_VER_DIST")
        ceiling, err = real_height_limits(args.height, args.max_height_m, gf_act, gf_max)
        result["geofence"] = dict(action=gf_act, max_ver_dist=gf_max)
    if err:
        result["error"] = err
        emit(result)
        return 2
    # the guard's own ceiling (stage 1 LAND) must sit above the controller's, or it fires in a normal hover
    if getattr(args, "ceiling_m", None) is None:
        args.ceiling_m = max(2.0, ceiling + 0.15)
    if vehicle_armed(conn) is not False:
        result["error"] = "refusing: vehicle is ARMED or silent"
        emit(result)
        return 2
    hover = read_param(conn, "MPC_THR_HOVER")
    result["hover_thrust"] = hover = float(hover) if hover else oc.Limits().hover_thrust
    result["com_disarm_land_s"] = read_param(conn, "COM_DISARM_LAND")
    if sim:     # the offboard port streams too little by default (20 Oct). Not asked of the
        for msg_id, us in ((32, 50000), (30, 50000), (65, 100000), (245, 200000),  # radio.
                           (132, 50000)):
            conn.mav.command_long_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                       0, msg_id, us, 0, 0, 0, 0, 0)

    import random
    outage = None
    if args.sim_outage:
        every, lo, hi = (float(x) for x in args.sim_outage.split(","))
        outage = oc.OutageModel(every, lo, hi, random.Random(args.seed))
    state = dict(armed=False, mode=None, z=None, vz=None, t_lp=None, tilt=None, yaw=None,
                 roll=None, pitch=None, last_hb=None)
    feed_asked = dict(on=False)       # set once we have asked for a faster height feed, so we put the default back
    armed_ever = dict(on=False)       # set when the aircraft armed under this agent, to start the stale-ARM watch
    rx = {}
    stdin = dict(pending=[], eof=False)
    guard = Guard(conn, args)
    guard.zero_required = True
    guard.classify_s = args.classify_s
    tx = dict(sent=0, dropped=0, last=0.0, on=True)
    cur = dict(cmd=oc.Command(oc.IDLE, 0.0, oc.level_quaternion(0.0)), stage=None, t_stage=None)
    trace = []
    heights = oc.HeightSource()

    def transmit(now):
        if not tx["on"] or guard.stop_streaming or now - tx["last"] < 1.0 / args.setpoint_hz:
            return
        tx["last"] = now
        if outage is not None and outage.dropped(now):
            tx["dropped"] += 1
            return
        c = cur["cmd"]
        kw = dict(thrust_body=[0.0, 0.0, 0.0]) if _ATT_HAS_BODY_THRUST else {}
        conn.mav.set_attitude_target_send(int(time.monotonic() * 1000) & 0xFFFFFFFF,
                                          conn.target_system, 1, 0b00000111, list(c.q),
                                          0.0, 0.0, 0.0, c.thrust, **kw)
        tx["sent"] += 1

    def estimate():
        return None if state["z"] is None or state.get("z0") is None else -(state["z"] - state["z0"])

    def height():
        return heights.height(estimate(), time.time())

    def enter(name):
        if name == cur["stage"]:
            return
        cur["stage"], cur["t_stage"] = name, time.time()
        log(f"[ladder] stage: {name}" + (" <- the stage under test" if name == stage else ""))
        emit(dict(event="stage", slot=args.slot, stage=name, under_test=name == stage,
                  height_m=None if height() is None else round(height(), 2),
                  t_epoch=round(time.time(), 3)))

    def pump(seconds: float, until=None) -> bool:
        end = time.time() + seconds
        while True:
            transmit(time.time())
            msg = conn.recv_match(blocking=True, timeout=0.02)
            while msg is not None:
                kind = msg.get_type()
                rx[kind] = rx.get(kind, 0) + 1
                if kind == "HEARTBEAT" and msg.get_srcSystem() == conn.target_system \
                        and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    state["mode"] = vehicle_main_mode(msg)
                    state["last_hb"] = time.time()
                elif kind == "LOCAL_POSITION_NED":
                    state.update(z=msg.z, vz=msg.vz, t_lp=time.time())
                elif kind == "DISTANCE_SENSOR" and \
                        msg.min_distance < msg.current_distance < msg.max_distance:
                    heights.range(msg.current_distance / 100.0, time.time())
                elif kind == "ATTITUDE":
                    state.update(yaw=msg.yaw, roll=msg.roll, pitch=msg.pitch)
                    state["tilt"] = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(msg.roll)
                                                                          * math.cos(msg.pitch)))))
                elif kind == "STATUSTEXT":
                    text = statustext_of(msg).strip()
                    log(f"[vehicle] {text}")
                    result.setdefault("vehicle_text", []).append(text)
                guard.observe(msg)
                if guard.pilot_has_it and result["takeover_stage"] is None:
                    result["takeover_stage"] = cur["stage"]
                    result["takeover_after_s"] = round(time.time() - (cur["t_stage"] or time.time()), 2)
                    h = height()
                    result["takeover_height_m"] = None if h is None else round(h, 2)
                    result["takeover_tilt_deg"] = None if state["tilt"] is None else round(state["tilt"], 1)
                msg = conn.recv_match(blocking=False)
            while select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    stdin["eof"] = True
                    break
                word = line.strip().upper()
                if word in EMERGENCY_WORDS:
                    guard.operator(word)
                    result.setdefault("operator", []).append(word)
                elif word:
                    stdin["pending"].append(word)
            acted = guard.check()
            if acted:
                result.setdefault("safety", []).append(acted)
            if until and until():
                return True
            if time.time() >= end:
                return until is None

    hands_off = lambda: guard.pilot_has_it or guard.stop_streaming  # noqa: E731
    t_start = time.time()
    try:
        pump(2.0)
        # Only flight stages need height. ARTEX113's ELRS link carries LOCAL_POSITION_NED at
        # 0.5 Hz (29 Oct), so a 2 s window can miss it, and pre-arm and arm never use it.
        if state["z"] is None and upto >= oc.STAGES.index("takeoff") and args.allow_radio_flight:
            pump(5.0, until=lambda: state["z"] is not None)
        if state["z"] is None and upto >= oc.STAGES.index("takeoff"):
            result["error"] = "no LOCAL_POSITION_NED (no height estimate)"
            return 2
        state["z0"] = guard.z0 = state["z"]
        rate_s = time.time() - t_start
        result["rx_hz"] = {k: round(v / rate_s, 1) for k, v in rx.items()
                           if k in ("LOCAL_POSITION_NED", "ATTITUDE", "RC_CHANNELS", "HEARTBEAT")}
        cur["cmd"] = oc.Command(oc.IDLE, 0.0, oc.level_quaternion(state["yaw"] or 0.0))

        enter("prearm")
        last, deadline = 0.0, time.time() + 10
        while time.time() < deadline and state["mode"] != OFFBOARD_MAIN_MODE and not hands_off():
            if time.time() - last > 2.0:
                conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                           0, 1, OFFBOARD_MAIN_MODE, 0, 0, 0, 0, 0)
                last = time.time()
            pump(0.5, lambda: state["mode"] == OFFBOARD_MAIN_MODE or hands_off())
        if hands_off():
            result["outcome"] = "takeover"
            return 0
        if state["mode"] != OFFBOARD_MAIN_MODE:
            result["error"] = f"vehicle did not enter OFFBOARD (main mode {state['mode']})"
            return 2
        if stage == "prearm":
            took = pump(args.wait_s, hands_off)
            result["outcome"] = "takeover" if took else "no_takeover"
            return 0

        if not sim and upto >= oc.STAGES.index("takeoff"):
            # Refuse BEFORE arming if the height feed cannot keep the controller alive (see height_feed_check).
            timeout = args.height_timeout_s or 1.0
            radio_link = not args.link.startswith(("serial:", "/dev/"))
            if radio_link:
                result["height_feed_requested_hz"] = RADIO_HEIGHT_FEED_HZ
                feed_asked["on"] = True
            log(f"[ladder] waiting up to 40 s for a height feed the controller can live on (a reading every "
                f"{timeout:g} s, 1 Hz or better)" + (f"; asked the aircraft for {RADIO_HEIGHT_FEED_HZ} Hz" if radio_link else ""))
            stamps, begun, last_ask, ok, worst, n = [], time.time(), 0.0, False, float("inf"), 0
            while time.time() - begun < 40.0 and not hands_off():
                if radio_link and time.time() - last_ask >= 3.0:      # a command can be lost or late on the radio
                    set_height_feed(conn, RADIO_HEIGHT_FEED_HZ)
                    last_ask = time.time()
                pump(0.25)
                if state["t_lp"] is not None and (not stamps or state["t_lp"] != stamps[-1]):
                    stamps.append(state["t_lp"])
                if time.time() - begun >= 6.0:
                    ok, worst, n = height_feed_ready(stamps, time.time(), timeout)
                    if ok:
                        break
            result["height_feed"] = dict(readings_last_6s=n, longest_gap_s=None if worst == float("inf") else round(worst, 1),
                                         timeout_s=timeout, waited_s=round(time.time() - begun, 1), ok=ok)
            if not ok:
                result["error"] = (f"refusing to arm: height does not arrive fast enough to fly. In the last 6 s there were "
                                   f"{n} readings, longest gap "
                                   f"{'unknown' if worst == float('inf') else f'{worst:.1f} s'}, against a {timeout:g} s "
                                   "timeout and a 1 Hz minimum, so the controller would abort at once. Over the radio this "
                                   "can happen; use the USB cable")
                return 2

        if args.gate:
            emit(dict(event="ready", slot=args.slot, sysid=conn.target_system, stage=stage))
            log(f"[ladder] READY for test '{stage}' -- type ARM once props/tether/area are confirmed")
            end, verdict = time.time() + args.gate_timeout_s, None
            while time.time() < end and verdict is None:
                pump(0.3)
                if stdin["pending"]:
                    verdict = stdin["pending"].pop(0)
                elif stdin["eof"]:
                    verdict = "EOF"
                elif hands_off():
                    verdict = "TAKEOVER"
            if verdict != "ARM":
                result["error"] = f"not authorised to arm ({verdict or 'timeout'})"
                return 2

        enter("arm")
        arm_gap = arm_retry_s(args.link, sim)
        arms_sent, last, deadline = 0, 0.0, time.time() + 20
        while time.time() < deadline and not state["armed"] and not hands_off():
            if time.time() - last > arm_gap:
                conn.mav.command_long_send(conn.target_system, 1,
                                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 1, 0, 0, 0, 0, 0, 0)
                last = time.time()
                arms_sent += 1
            pump(0.5, lambda: state["armed"] or hands_off())
        result["arm_commands_sent"], result["arm_retry_s"] = arms_sent, arm_gap
        if hands_off():
            result["outcome"] = "takeover"
            return 0
        if not state["armed"]:
            log("[ladder] gave up waiting for the arm; sending DISARM behind any ARM still queued in the link")
            w = withdraw_arm(lambda: conn.mav.command_long_send(conn.target_system, 1,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0), pump, state)
            result.update(arm_withdrawn=True, armed_late=w["armed_late"])
            result["error"] = "vehicle did not arm" + (" in time: it armed late, after the agent gave up"
                                                       if w["armed_late"] else "")
            return 2
        armed_ever["on"] = True
        emit(dict(event="armed", slot=args.slot, sysid=conn.target_system,
                  armed_epoch=round(time.time(), 3)))
        # Re-zero on the floor at arming: the estimate drifted 0.24 m between the first zero and
        # take-off in one SITL run, enough for the controller to think it was already airborne.
        state["z0"] = guard.z0 = state["z"]
        if stage == "arm":
            took = pump(args.wait_s, lambda: hands_off() or not state["armed"])
            result["outcome"] = "takeover" if guard.pilot_has_it else \
                ("disarmed" if not state["armed"] else "no_takeover")
            return 0

        prof = oc.stage_profile(stage, height_m=args.height, wait_s=args.wait_s,
                                hold_s=args.hold_s, takeoff_pause_m=args.takeoff_pause_m,
                                test_descent_rate_ms=args.test_descent_rate,
                                final_descent_rate_ms=args.final_descent_rate)
        lim = oc.Limits(target_height_m=prof["target_height_m"], ceiling_m=ceiling,
                        hover_thrust=hover, descent_rate_ms=prof["descent_rate_ms"],
                        final_descent_rate_ms=prof["final_descent_rate_ms"],
                        height_timeout_s=args.height_timeout_s or (0.5 if sim else 1.0),
                        lift_by_climb=args.lift_by_climb,
                        **({} if args.spool_fast_frac is None
                           else dict(spool_fast_frac=args.spool_fast_frac)))
        ctl = oc.HeightHold(lim, yaw_rad=state["yaw"] or 0.0, hold_s=prof["hold_s"])
        # Fix 3 (T3 run 1, 29 Oct): ARTEX113 sits 1.3 deg nose-down on the floor, and PX4's land
        # detector calls it airborne from the first spool, so its attitude integrators wound up
        # against the floor for 3 s (front-rear motors +0.15) and let go at lift-off. Commanding
        # the attitude it is sitting at leaves nothing to wind up; it eases to level over 1 s.
        floor_att = (state["roll"] or 0.0, state["pitch"] or 0.0)
        result["profile"] = prof
        ctl.start(time.time(), 0.0)
        t_prev = last_batt = time.time()
        while True:
            pump(0.05)
            now = time.time()
            dt, t_prev = now - t_prev, now
            if guard.pilot_has_it:
                result["outcome"] = "takeover"
                break
            if guard.stop_streaming:
                result["outcome"] = "guard_acted"
                break
            if not state["armed"]:
                result["outcome"] = "disarmed_in_flight"
                break
            if state["mode"] != OFFBOARD_MAIN_MODE:
                result["outcome"] = f"left_offboard_to_mode_{state['mode']}"
                break
            if args.lift_by_climb and not ctl.airborne:
                heights.floor(estimate(), now)      # on the floor, the estimate's drift is not height
            h = height()
            valid = state["t_lp"] is not None and now - state["t_lp"] < lim.height_timeout_s
            vz_up = None if state["vz"] is None else -state["vz"]
            cmd = ctl.step(now, dt, h, valid, vz_up, state["tilt"])
            if args.ground_attitude and cmd.phase != oc.ABORT:
                k = 1.0 if not ctl.airborne else max(0.0, 1.0 - (now - ctl.airborne_at) / 1.0)
                if k > 0.0:
                    cmd.q = oc.attitude_quaternion(k * floor_att[0], k * floor_att[1], ctl.yaw_rad)
            cur["cmd"] = cmd
            # The agent's own intent stands in for the throttle stick: the guard's "climbing
            # with the stick at centre" rule must not fire on a climb the agent asked for.
            guard.stick_override = max(0.0, min(1.0, 0.5 + (cmd.vz_sp or 0.0)))
            guard.wd.stick(guard.stick_override, now)
            name = oc.stage_of(cmd.phase, cmd.height_sp, lim.final_zone_m)
            # the take-off test's pause at --takeoff-pause-m holds take-off open; it is not "hold"
            enter("takeoff" if stage == "takeoff" and name == "hold" else name)
            trace.append((round(now - t_start, 2), cmd.phase, None if h is None else round(h, 3),
                          None if cmd.height_sp is None else round(cmd.height_sp, 3),
                          round(cmd.thrust, 3), state["mode"], guard.batt_v, guard.batt_pct))
            if now - last_batt >= 30.0 and (guard.batt_v is not None or guard.batt_pct is not None):
                last_batt = now      # ~100 bytes every 30 s: far too little to ever fill the stdout pipe
                emit(dict(event="battery", slot=args.slot, volts=guard.batt_v, pct=guard.batt_pct,
                          stage=cur["stage"], t_s=round(now - t_start, 1), t_epoch=round(time.time(), 3)))
            if cmd.request == "land":
                log(f"[ladder] controller abort: {cmd.reason} -> PX4 LAND")
                result.update(outcome="controller_abort", abort_reason=cmd.reason)
                tx["on"] = False
                conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_NAV_LAND,
                                           0, 0, 0, 0, 0, 0, 0, 0)
                break
            if cmd.request == "disarm":
                # touchdown is PX4 LAND's: its land detector confirms contact and disarms
                # (COM_DISARM_LAND). Disarming at our own 'landed' was refused in SITL, 20 Oct.
                log("[ladder] near the ground; touchdown handed to PX4 LAND")
                tx["on"] = False
                conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_NAV_LAND,
                                           0, 0, 0, 0, 0, 0, 0, 0)
                pump(20.0, lambda: guard.airborne is False or guard.pilot_has_it or not state["armed"])
                if state["armed"] and not guard.pilot_has_it:
                    enter("disarm")
                pump(20.0, lambda: guard.pilot_has_it or not state["armed"])
                result["outcome"] = "takeover" if guard.pilot_has_it else \
                    ("landed_px4_disarmed" if not state["armed"] else "landed_still_armed")
                break
        return 0
    finally:
        tx["on"] = False
        # The trace first, before anything below touches the link: in T3 run 1 (29 Oct) the USB
        # dropped at the floor impact, the pump() here raised again and the trace was lost.
        if args.trace:
            Path(args.trace).write_text("t_s,phase,height_m,height_sp_m,thrust,mode,batt_v,batt_pct\n" + "".join(
                ",".join("" if v is None else str(v) for v in r) + "\n" for r in trace))
        result["takeover_source"] = guard.takeover_source if guard.pilot_has_it else None
        result["link_losses"] = guard.link_losses
        result["setpoints"] = dict(sent=tx["sent"], dropped_sim=tx["dropped"],
                                   hz=args.setpoint_hz)
        result["max_height_m"] = max((r[2] for r in trace if r[2] is not None), default=None)
        if guard.pilot_has_it:
            pump(guard.classify_s + 0.5, lambda: guard.takeover_source is not None)
            result["takeover_source"] = guard.takeover_source
            w = await_pilot_disarm(pump, state, killed=guard.killed_by_pilot,
                                   slot=args.slot, window_s=args.pilot_disarm_window_s)
            result["disarmed"] = w["disarmed"]
        elif state["armed"]:
            # PX4 LAND (controller abort or the guard's) finishes on its own; wait for it,
            # and only then disarm what is confirmed on the ground.
            pump(30.0, lambda: not state["armed"] or guard.pilot_has_it or guard.airborne is False)
            send_disarm = lambda: conn.mav.command_long_send(  # noqa: E731
                conn.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0)
            disarm_unless_pilot(send_disarm, pump, state, guard)
            if state["armed"] and not guard.pilot_has_it:
                h = hold_until_disarmed(send_disarm, pump, state, guard,
                                        window_s=args.armed_exit_window_s, slot=args.slot)
                result["left_armed"] = h["outcome"] == "left_armed"
            result["disarmed"] = confirmed_disarmed(state, time.time())
        else:
            result["disarmed"] = confirmed_disarmed(state, time.time())
        if armed_ever["on"] and result.get("disarmed") is True:
            # a copy of the ARM can still be on its way over the radio: do not leave an unsupervised re-arm behind
            log(f"[safety] disarmed; watching {STALE_ARM_WATCH_S:.0f} s for a stale ARM arriving late")
            try:
                result["stale_arm"] = stale_arm_guard(
                    pump, state, lambda: result.setdefault("vehicle_text", []),
                    lambda: conn.mav.command_long_send(conn.target_system, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                                       0, 0, 0, 0, 0, 0, 0, 0))
            except Exception as exc:       # a dead link here must never cost the result (29 Oct: trace lost that way)
                result["stale_arm"] = dict(error=f"{type(exc).__name__}: {exc}")
        result["passed"] = (result["outcome"] == "takeover" and result["takeover_stage"] == stage
                            and result["takeover_source"] in ("pilot_switch", "kill_switch")
                            and not result.get("safety"))
        emit(result)
        if feed_asked["on"]:
            try:
                set_height_feed(conn, 0)      # the default rate again; best effort, a reboot clears it anyway
            except Exception:
                pass


# COM_FLTMODE<n> values, from PX4 1.16's own parameter metadata
# (build/px4_sitl_default/parameters.json), not from memory.
FLTMODE_NAMES = {-1: "Unassigned", 0: "Manual", 1: "Altitude", 2: "Position", 3: "Mission",
                 4: "Hold", 5: "Return", 6: "Acro", 7: "Offboard", 8: "Stabilized",
                 9: "Position Slow", 10: "Takeoff", 11: "Land", 12: "Follow Me",
                 13: "Precision Land"}
# The slots that hand the aircraft to a human. Hold/Mission/Return are still autopilot.
PILOT_FLTMODES = (0, 1, 2, 6, 8, 9)
RC_PARAMS = ("COM_RC_IN_MODE", "COM_RC_OVERRIDE", "COM_RC_STICK_OV", "RC_MAP_FLTMODE",
             "RC_MAP_KILL_SW", "COM_RCL_EXCEPT", "COM_FLTMODE1", "COM_FLTMODE2",
             "COM_FLTMODE3", "COM_FLTMODE4", "COM_FLTMODE5", "COM_FLTMODE6", "RC_MAP_OFFB_SW")


def rc_takeover_findings(p: dict, rc_seen: bool) -> list[str]:
    """What STOPS the RC switch from taking this aircraft. Empty = nothing does.

    Only blocking faults belong here. Policy choices live in rc_takeover_notes() -- see
    the note there about COM_RC_OVERRIDE, which is deliberately left at PX4's default on
    this project. Pure, so tests/test_rc_takeover.py can pin it without an aircraft.
    """
    out = []
    # 0 RC only, 2 RC and joystick, 3 whichever is seen first (so RC works if RC is first --
    # and the QGC mirror is observe-only, so no joystick can get in ahead of it).
    # 1 is joystick only and 4 disables stick input entirely.
    in_mode = p.get("COM_RC_IN_MODE")
    if in_mode not in (0, 2, 3):
        out.append(f"COM_RC_IN_MODE={in_mode}: this aircraft does not accept RC stick input "
                   f"(need 0 'RC only', 2 'RC and joystick', or 3 'keep first'; "
                   f"1 is joystick only and 4 is disabled)")

    if not p.get("RC_MAP_FLTMODE"):
        out.append("RC_MAP_FLTMODE=0: no channel carries the flight-mode switch, so no "
                   "switch position can request a mode")
    slots = {n: p.get(f"COM_FLTMODE{n}") for n in range(1, 7)}
    if not any(v is not None and int(v) in PILOT_FLTMODES for v in slots.values()):
        out.append("no COM_FLTMODE1..6 slot holds a hand-flown mode (Manual/Altitude/"
                   "Position/Acro/Stabilized), so no switch position hands over control")
    if not rc_seen:
        out.append("no RC_CHANNELS heard: the transmitter is off, unbound, or its link to "
                   "this aircraft is down. Nothing above can work until it is heard")
    return out


def rc_takeover_notes(p: dict) -> list[str]:
    """True but not blocking: what the current configuration does and does not cover.

    COM_RC_OVERRIDE is the big one. Bit 1 enables stick override during OFFBOARD, and this
    project deliberately leaves it CLEAR (6 Oct 2026) so a knocked stick cannot drop an
    aircraft out of offboard mid-run. The consequence is that the mode switch is the only
    way to take over, which is the intent -- a deliberate act, not an accidental one.
    """
    out = []
    ov = p.get("COM_RC_OVERRIDE")
    if ov is None:
        out.append("COM_RC_OVERRIDE could not be read")
    else:
        ov = int(ov)
        if not ov & 2:
            out.append(f"COM_RC_OVERRIDE={ov}: stick movement will NOT break OFFBOARD "
                       f"(deliberate here -- the mode switch is the takeover path). "
                       f"Set 3 only if you want the sticks to override too")
        if not ov & 1:
            out.append(f"COM_RC_OVERRIDE={ov}: stick movement will not break AUTO either")
    if not p.get("RC_MAP_KILL_SW"):
        out.append("RC_MAP_KILL_SW=0: no kill switch mapped -- the only override that does "
                   "not need the autopilot to cooperate")
    # A switch position mapped to Offboard is a position the pilot can land on by accident
    # or by design. The agent's stream is level attitude at zero thrust.
    off = [n for n in range(1, 7)
           if p.get(f"COM_FLTMODE{n}") is not None and int(p[f"COM_FLTMODE{n}"]) == 7]
    if off:
        out.append(f"COM_FLTMODE{'/'.join(str(n) for n in off)} = Offboard: that switch "
                   f"position asks PX4 for OFFBOARD. The agent streams level attitude at "
                   f"ZERO thrust and cannot hold height, so on an airborne aircraft it is "
                   f"a fall, not a hand-back. Bench use only; map it to a hand-flown mode "
                   f"for flight")
    # 3 Nov 2026 layout: OFFBOARD on its own switch; dropping it lands in the mode switch's mode.
    offb = p.get("RC_MAP_OFFB_SW")
    if offb:
        out.append(f"RC_MAP_OFFB_SW={int(offb)}: channel {int(offb)} on asks for OFFBOARD; off "
                   f"hands the aircraft to whatever channel {p.get('RC_MAP_FLTMODE')} selects. "
                   f"Leave that on Altitude while the agent flies, so a takeover holds height")
    direct = [n for n in range(1, 7)
              if p.get(f"COM_FLTMODE{n}") is not None and int(p[f"COM_FLTMODE{n}"]) in (0, 6, 8)]
    if direct:
        out.append(f"COM_FLTMODE{'/'.join(str(n) for n in direct)} = Manual/Acro/Stabilized: the "
                   f"throttle stick is thrust there, not climb rate. Taken over in flight with "
                   f"the stick centred that is about 50% thrust, above hover (0.42 on ARTEX113), "
                   f"so it climbs. Take over into Altitude; arm there with the stick at the bottom")
    return out


def cmd_rc_check(args) -> int:
    """Read-only: can the RC switch take this aircraft away from us?"""
    conn = connect(args.link, args.expect_sysid)
    params = {}
    for name in RC_PARAMS:
        params[name] = read_param(conn, name)
    rc_seen, deadline = False, time.time() + 6.0
    rssi = None
    while time.time() < deadline and not rc_seen:
        m = conn.recv_match(type="RC_CHANNELS", blocking=True, timeout=1)
        if m is not None:
            rc_seen, rssi = True, getattr(m, "rssi", None)
    slots = {n: params.get(f"COM_FLTMODE{n}") for n in range(1, 7)}
    findings = rc_takeover_findings(params, rc_seen)
    notes = rc_takeover_notes(params)
    for f in findings:
        log(f"[rc] {f}")
    for n in notes:
        log(f"[rc] note: {n}")
    if not findings:
        log("[rc] the RC switch can take this aircraft")
    emit(dict(verb="rc-check", slot=args.slot, sysid=conn.target_system,
              rc_seen=rc_seen, rssi=rssi, params=params,
              mode_slots={n: FLTMODE_NAMES.get(int(v), v) for n, v in slots.items()
                          if v is not None},
              findings=findings, notes=notes, ready=not findings))
    return 0 if not findings else 1


def cmd_abort(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    emit(dict(verb="abort", slot=args.slot, sysid=conn.target_system,
              **do_abort(conn, args.action)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("verb", choices=["prepare", "upload", "start", "status", "abort",
                                        "offboard-arm", "offboard-flight", "rc-check"])
    ap.add_argument("--hold-s", type=float, default=10.0, help="offboard-arm: seconds to stay armed")
    ap.add_argument("--gate", action="store_true",
                    help="offboard-arm: enter OFFBOARD, print a ready event, arm only on 'ARM' from stdin")
    ap.add_argument("--gate-timeout-s", type=float, default=90.0)
    ap.add_argument("--armed-exit-window-s", type=float, default=120.0,
                    help="offboard-arm: if the disarm is not confirmed, keep the link up, keep "
                         "retrying and keep warning for this long before exiting (code 4)")
    ap.add_argument("--pilot-disarm-window-s", type=float, default=60.0,
                    help="offboard-arm: after a pilot takeover, how long to keep warning that "
                         "the aircraft is still armed before leaving (the agent never disarms)")
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
    ap.add_argument("--no-safety", action="store_true",
                    help="run without the abort ladder (bench diagnosis only)")
    ap.add_argument("--abort-flag", default="ABORT",
                    help="path polled for an out-of-band abort; its contents may name the "
                         "rung (hold/land/disarm). tools/takeover.py writes this over ssh")
    ap.add_argument("--allow-kill", action="store_true",
                    help="permit KILL: a force-disarm that drops the aircraft out of the air")
    ap.add_argument("--hold-rate", type=float, default=None,
                    help="weighted vehicle errors per minute that stop the mission")
    ap.add_argument("--land-rate", type=float, default=None,
                    help="weighted vehicle errors per minute that force a landing")
    ap.add_argument("--battery-land-pct", type=float, default=None)
    ap.add_argument("--tilt-land-deg", type=float, default=None)
    ap.add_argument("--ceiling-m", type=float, default=None,
                    help="rangefinder height that triggers a descent (default 2.0; 0 = off)")
    ap.add_argument("--cut-after-s", type=float, default=None,
                    help="stage 2: seconds of failed descent before an in-air cut "
                         "(default 3; needs --allow-kill)")
    ap.add_argument("--motor-range", default="0,1999",
                    help="SERVO_OUTPUT_RAW min,max for motor saturation (DShot 0,1999)")
    ap.add_argument("--throttle-chan", type=int, default=3)
    # -- offboard-flight: the takeover ladder (29 Oct 2026, docs/takeover-ladder-plan.md)
    ap.add_argument("--stage", choices=["prearm", "arm", "takeoff", "hold", "descent",
                                        "landing", "disarm"],
                    help="offboard-flight: fly up to this stage and hold it open for the pilot")
    ap.add_argument("--wait-s", type=float, default=15.0,
                    help="offboard-flight: how long the stage under test waits for a takeover")
    ap.add_argument("--height", type=float, default=1.0, help="offboard-flight: hover height (m)")
    ap.add_argument("--takeoff-pause-m", type=float, default=0.3)
    ap.add_argument("--lift-by-climb", action="store_true",
                    help="offboard-flight: lift-off also from a sustained climb rate; the height "
                         "estimate is re-zeroed while on the floor (fix 1, 29 Oct)")
    ap.add_argument("--spool-fast-frac", type=float, default=None,
                    help="offboard-flight: quick spool to this fraction of hover (fix 2; default 0.8)")
    ap.add_argument("--ground-attitude", action="store_true",
                    help="offboard-flight: command the floor attitude until lift-off, then ease to "
                         "level over 1 s (fix 3, 29 Oct)")
    ap.add_argument("--test-descent-rate", type=float, default=0.08,
                    help="offboard-flight --stage descent: m/s, slow so the stage lasts")
    ap.add_argument("--final-descent-rate", type=float, default=0.05,
                    help="offboard-flight --stage landing: m/s over the last 0.3 m")
    ap.add_argument("--setpoint-hz", type=float, default=None,
                    help="offboard-flight: setpoint rate (default 20 on USB and in SITL, 3 over a radio: 20 Hz buries "
                         "commands on the ELRS uplink, 4 Nov 2026). 5 Hz made the height loop oscillate in SITL "
                         "(29 Oct); 50 Hz held +/-1 cm")
    ap.add_argument("--height-timeout-s", type=float, default=None,
                    help="offboard-flight: abort if no height estimate this long "
                         "(default 0.5 s sim, 1.0 s real)")
    ap.add_argument("--real-aircraft", action="store_true",
                    help="offboard-flight: required, with --expect-sysid and --gate, to fly a real one")
    ap.add_argument("--max-height-m", type=float, default=None,
                    help="offboard-flight, real aircraft: raise the hover-height cap above 1.05 m, to 2.0 m at most "
                         "(the controller ceiling follows, +0.15 m). Endurance hover, 4 Nov 2026")
    ap.add_argument("--battery-hold-pct", type=float, default=None,
                    help="guard: battery %% that makes it HOLD (default 25)")
    ap.add_argument("--allow-radio-flight", action="store_true",
                    help="offboard-flight, real aircraft: allow flight stages over ELRS. PROPS OFF "
                         "ONLY -- the thrust-only height loop is unstable across the radio (29 Oct); "
                         "pass --height-timeout-s 3 too, height arrives at 0.5 Hz")
    ap.add_argument("--sim-outage", default=None, metavar="EVERY,MIN,MAX",
                    help="offboard-flight, SIMULATOR ONLY: drop setpoints in bursts like ELRS "
                         "(mean seconds between outages, outage length range)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--trace", default=None, help="offboard-flight: write a CSV of the flight")
    ap.add_argument("--mode-chan", type=int, default=7, help="RC channel of the mode switch")
    ap.add_argument("--offboard-chan", type=int, default=5,
                    help="RC channel of PX4's offboard switch (RC_MAP_OFFB_SW; +100 = OFFBOARD)")
    ap.add_argument("--kill-chan", type=int, default=8, help="RC channel of the kill switch")
    ap.add_argument("--kill-pwm-above", type=int, default=1750,
                    help="kill switch counts as ON above this PWM (check with rc-check first)")
    ap.add_argument("--classify-s", type=float, default=1.5,
                    help="after leaving OFFBOARD, wait this long for a mode-switch reading "
                         "before calling it a lost link")
    args = ap.parse_args()

    if args.verb in ("prepare", "upload") and not args.plan:
        ap.error(f"{args.verb} needs --plan")
    if args.verb == "offboard-flight" and not args.stage:
        ap.error("offboard-flight needs --stage")

    try:
        return {"prepare": cmd_prepare, "upload": cmd_upload, "start": cmd_start,
                "status": cmd_status, "abort": cmd_abort,
                "offboard-arm": cmd_offboard_arm,
                "offboard-flight": cmd_offboard_flight,
                "rc-check": cmd_rc_check}[args.verb](args)
    except Exception as exc:
        log(f"[error] {type(exc).__name__}: {exc}")
        emit(dict(verb=args.verb, slot=args.slot, ready=False, error=str(exc)))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
