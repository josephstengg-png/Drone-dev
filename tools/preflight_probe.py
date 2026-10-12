#!/usr/bin/env python3
"""Everything a companion can tell us about itself and its aircraft, as one JSON object.

    python3 tools/preflight_probe.py <sysid> [--link udpout://127.0.0.1:14540]

Runs ON the companion. `tools/preflight.py` deploys and invokes it over ssh; nothing else
should need to call it directly. One JSON object on stdout, nothing else -- so a GUI or
an orchestrator can consume it without parsing prose.

WHY A PROBE AND NOT A PILE OF SSH CALLS. The same chain was being checked by hand every
session -- wifi, backpack, relay count, abort flag, heartbeat, GPS -- at one ssh round
trip per question, and inconsistently, because it lived in somebody's habit rather than
in code. The things it checks are exactly the ones that have actually bitten: a second
relay, a stale abort flag, the wrong backpack, a receiver that is not transmitting.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

BACKPACK = "10.0.0.1"
# The EKF's own gates. GPS below these is rejected for horizontal position, which is what
# produces "XY position sensor error" -- measured on ARTEX113, 11 Oct 2026.
GATES = dict(nsats=6, eph_m=3.0, epv_m=5.0, sacc_ms=0.5, pdop=2.5)


def sh(cmd: str, timeout: float = 12.0) -> str:
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired:
        return ""


def wifi() -> dict:
    iface = ""
    for n in os.listdir("/sys/class/net"):
        if n.startswith("wl"):
            iface = n
            break
    if not iface:
        return {"iface": None, "associated": False}
    link = sh(f"iw dev {iface} link")
    m = re.search(r"Connected to ([0-9a-f:]{17})", link, re.I)
    sig = re.search(r"signal:\s*(-?\d+)", link)
    return {"iface": iface, "associated": bool(m),
            "bssid": m.group(1).lower() if m else None,
            "signal_dbm": int(sig.group(1)) if sig else None}


def ping(host: str, n: int = 5) -> dict:
    out = sh(f"ping -c {n} -W 1 {host}", timeout=n * 2 + 8)
    loss = re.search(r"([\d.]+)% packet loss", out)
    rtt = re.search(r"rtt [^=]*= [\d.]+/([\d.]+)/", out)
    return {"loss_pct": float(loss.group(1)) if loss else 100.0,
            "avg_ms": float(rtt.group(1)) if rtt else None}


def backpack_counters() -> dict:
    try:
        with urllib.request.urlopen(f"http://{BACKPACK}/mavlink", timeout=6) as r:
            return json.load(r).get("counters", {})
    except Exception as exc:
        return {"error": str(exc)}


def relays() -> list:
    found = []
    for pid in sh("pgrep -x python3").split():
        try:
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="replace")
        except OSError:
            continue
        if "elrs_link" in cmd:
            found.append(int(pid))
    return found


def vehicle(link: str, sysid: int) -> dict:
    """Heartbeat, arm state, GPS quality and home, judged against the EKF's gates."""
    import drone_agent as agent

    out = {"reachable": False}
    try:
        conn = agent.connect(link, sysid)
    except Exception as exc:
        out["error"] = str(exc)
        return out
    out["reachable"] = True
    out["sysid"] = conn.target_system

    conn.mav.command_long_send(conn.target_system, 1, 512, 0, 242, 0, 0, 0, 0, 0, 0)
    seen = {}
    end = time.time() + 14
    while time.time() < end:
        m = conn.recv_match(type=["HEARTBEAT", "GPS_RAW_INT", "HOME_POSITION",
                                  "BATTERY_STATUS"], blocking=True, timeout=2)
        if m is None:
            continue
        if m.get_type() == "HEARTBEAT" and m.get_srcSystem() != conn.target_system:
            continue
        seen[m.get_type()] = m

    hb = seen.get("HEARTBEAT")
    if hb:
        out["armed"] = bool(hb.base_mode & 128)
        out["main_mode"] = (hb.custom_mode >> 16) & 0xFF
    b = seen.get("BATTERY_STATUS")
    if b is not None and b.battery_remaining >= 0:
        out["battery_pct"] = b.battery_remaining
    out["home_set"] = "HOME_POSITION" in seen

    g = seen.get("GPS_RAW_INT")
    if g:
        inf = float("inf")
        h = g.h_acc / 1000 if g.h_acc < 4e9 else inf
        v = g.v_acc / 1000 if g.v_acc < 4e9 else inf
        s = g.vel_acc / 1000 if g.vel_acc < 4e9 else inf
        hdop, vdop = g.eph / 100, g.epv / 100
        pdop = (hdop ** 2 + vdop ** 2) ** 0.5
        fails, soft = [], []
        if g.satellites_visible < GATES["nsats"]:
            fails.append(f"sats {g.satellites_visible}<{GATES['nsats']}")
        if h > GATES["eph_m"]:
            fails.append(f"eph {h:.2f}m>{GATES['eph_m']}m")
        if v > GATES["epv_m"]:
            fails.append(f"epv {v:.2f}m>{GATES['epv_m']}m")
        if s > GATES["sacc_ms"]:
            fails.append(f"sacc {s:.2f}>{GATES['sacc_ms']}")
        # PDOP here is DERIVED from HDOP and VDOP, because GPS_RAW_INT does not carry
        # PDOP. PX4's own EKF2_REQ_PDOP compares the value the GPS driver reports, which
        # may differ -- so this is advisory, not the authoritative gate.
        if pdop > GATES["pdop"]:
            soft.append(f"derived pdop {pdop:.2f}>{GATES['pdop']}")
        out["gps"] = {"fix_type": g.fix_type, "sats": g.satellites_visible,
                      "h_acc_m": None if h == inf else round(h, 2),
                      "v_acc_m": None if v == inf else round(v, 2),
                      "pdop": round(pdop, 2), "gate_failures": fails,
                      "advisory": soft, "position_usable": not fails}
    else:
        out["gps"] = {"fix_type": 0, "sats": 0, "gate_failures": ["no GPS_RAW_INT"],
                      "position_usable": False}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sysid", type=int)
    ap.add_argument("--link", default="udpout://127.0.0.1:14540")
    ap.add_argument("--abort-flag", default="ABORT")
    ap.add_argument("--no-vehicle", action="store_true",
                    help="companion checks only; do not open a MAVLink session")
    args = ap.parse_args()

    report = {
        "host": sh("hostname").strip(),
        "clock_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "clock_sane": time.time() > 1.7e9,       # anything in 1970 is not a clock
        "wifi": wifi(),
        "backpack_ping": ping(BACKPACK),
        "backpack_counters": backpack_counters(),
        "relay_pids": relays(),
        "abort_flag": Path(args.abort_flag).read_text().strip()
                      if Path(args.abort_flag).exists() else None,
        "qgc_running": bool(sh("pgrep -x QGroundControl").strip()),
    }
    report["vehicle"] = ({"skipped": True} if args.no_vehicle
                         else vehicle(args.link, args.sysid))
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
