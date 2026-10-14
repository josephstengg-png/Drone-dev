#!/usr/bin/env python3
"""Prove the whole chain, from this keyboard to the aircraft, and say GO or NO-GO.

    python3 tools/preflight.py --fleet fleet-artex-pair.yaml
    python3 tools/preflight.py --fleet fleet-artex113.yaml --json
    python3 tools/preflight.py --fleet fleet.yaml --no-vehicle   # ground side only

Read-only. It arms nothing, writes no parameters and changes no state.

WHY THIS EXISTS. The same sequence was being checked by hand before every run -- cable,
Jetson, backpack, which backpack, relay count, abort flag, heartbeat, link quality, GPS --
and it lived in one person's habit rather than in code, so it was run inconsistently and
remembered imperfectly. Every item below is here because it has actually caused a failure:

  carrier           12 Oct: the switch was unpowered; both Jetsons looked "dead"
  wrong BSSID       28 Sep: both companions attached to ONE transmitter
  relay count       7 Oct: two relays split the downlink; QGC showed no vehicle
  stale ABORT flag  by design the agent refuses to arm; easy to leave set
  packets_down 0    7 Oct: the aircraft's receiver was not transmitting at all
  GPS gates         11 Oct: "XY position sensor error" was a weak fix, not config
  QGC running       7 Oct: a second commander on a link that cannot carry it

Exit code 0 only if every aircraft is GO.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
from swarm_deploy import host_argv  # noqa: E402

REGISTRY = ROOT / "artex-registry.yaml"
COMPANIONS = ROOT / "companions.yaml"
PROBE = Path(__file__).resolve().parent / "preflight_probe.py"

# A lost ARM is a non-event; a lost DISARM is not. These are the same limits the pair
# arming test refuses to start above.
MAX_LOSS_PCT = 10.0
MAX_RTT_MS = 150.0


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def expected_bssid(sysid: int) -> str | None:
    """Which transmitter this aircraft's companion must be on, per the registry."""
    if not REGISTRY.exists():
        return None
    reg = yaml.safe_load(REGISTRY.read_text()) or {}
    for name, pack in (reg.get("backpacks") or {}).items():
        if name.upper().endswith(str(sysid)) and pack.get("bssid"):
            return str(pack["bssid"]).lower()
    return None


def resolve(host: str) -> str:
    """ssh aliases are not DNS names. Ping needs the address, so look it up.

    `ping jetson-1` fails with 100% loss even when ssh to jetson-1 works perfectly --
    the name lives in ssh config, not in DNS or /etc/hosts. companions.yaml is the
    source of truth for the mapping.
    """
    if COMPANIONS.exists():
        spec = yaml.safe_load(COMPANIONS.read_text()) or {}
        c = (spec.get("companions") or {}).get(host)
        if c and c.get("ip"):
            return str(c["ip"])
    return host


def ground_ping(host: str) -> dict:
    """The workstation's own reach to the companion. Catches a dead cable or switch."""
    try:
        r = subprocess.run(["ping", "-c", "4", "-W", "1", resolve(host)],
                           capture_output=True, text=True, timeout=20)
    except subprocess.TimeoutExpired:
        return {"loss_pct": 100.0, "avg_ms": None}
    loss = re.search(r"([\d.]+)% packet loss", r.stdout)
    rtt = re.search(r"rtt [^=]*= [\d.]+/([\d.]+)/", r.stdout)
    return {"loss_pct": float(loss.group(1)) if loss else 100.0,
            "avg_ms": float(rtt.group(1)) if rtt else None}


def probe(drone: dict, no_vehicle: bool, timeout: float) -> dict:
    host = drone.get("host", "local")
    remote = drone.get("remote_dir", "~/FlightPlanner")
    if host not in ("local", "localhost"):
        subprocess.run(["scp", "-q", "-o", "ConnectTimeout=10", str(PROBE),
                        f"{host}:{remote}/tools/preflight_probe.py"],
                       capture_output=True, timeout=60)
    argv = ["python3", "tools/preflight_probe.py", str(drone["sysid"]),
            "--link", drone["link"]]
    if no_vehicle:
        argv.append("--no-vehicle")
    cmd, cwd = host_argv(drone, argv)
    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"error": "probe timed out"}
    for line in reversed(r.stdout.strip().splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return {"error": (r.stderr.strip().splitlines() or ["no output from probe"])[-1]}


def assess(drone: dict, ground: dict, p: dict) -> tuple[list, list]:
    """Returns (blockers, warnings). A blocker means NO-GO."""
    bad, warn = [], []
    sysid = drone["sysid"]

    if ground["loss_pct"] >= 100:
        bad.append(f"ground LAN to {drone.get('host')}: unreachable")
    elif ground["loss_pct"] > 0:
        warn.append(f"ground LAN to {drone.get('host')}: {ground['loss_pct']:.0f}% loss")
    if p.get("error"):
        bad.append(f"companion probe failed: {p['error']}")
        return bad, warn

    w = p.get("wifi", {})
    if not w.get("associated"):
        bad.append("companion is not associated to any backpack")
    else:
        want = expected_bssid(sysid)
        if want and w.get("bssid") != want:
            bad.append(f"WRONG BACKPACK: on {w.get('bssid')}, expected {want} for "
                       f"ARTEX{sysid}")

    bp = p.get("backpack_ping", {})
    if bp.get("loss_pct", 100) > MAX_LOSS_PCT:
        bad.append(f"backpack link {bp.get('loss_pct')}% loss > {MAX_LOSS_PCT}%")
    elif (bp.get("avg_ms") or 0) > MAX_RTT_MS:
        bad.append(f"backpack RTT {bp.get('avg_ms')}ms > {MAX_RTT_MS}ms")

    c = p.get("backpack_counters", {})
    if c.get("error"):
        bad.append(f"backpack unreachable over http: {c['error']}")
    elif c.get("packets_down", 0) == 0:
        bad.append("backpack has received NOTHING from the aircraft "
                   "(receiver off, unbound, or in wifi mode)")

    n = len(p.get("relay_pids", []))
    if n == 0:
        bad.append("no relay running on the companion")
    elif n > 1:
        bad.append(f"{n} relays running -- they split the downlink; expected exactly 1")

    if p.get("abort_flag") is not None:
        bad.append(f"stale ABORT flag set ({p['abort_flag']!r}) -- the agent will refuse "
                   f"to arm; clear with tools/takeover.py --clear")

    if p.get("qgc_running") and not (p.get("relay_floor_s") or 0) > 0:
        # With agent priority on the relay (--mirror-floor-s) QGC is held off while an
        # agent talks, so it no longer has to be closed before a run. Without it, it is a
        # second commander on a link that cannot carry two (12 Oct 2026).
        warn.append("QGroundControl is running and the relay has NO agent priority -- a "
                    "second commander on a link that cannot carry two; close it, or run "
                    "the relay with --mirror-floor-s 5")
    if not p.get("clock_sane"):
        warn.append(f"companion clock is wrong ({p.get('clock_utc')}) -- timestamps and "
                    f"any cross-aircraft timing will be meaningless")

    v = p.get("vehicle", {})
    if v.get("skipped"):
        warn.append("vehicle checks skipped (--no-vehicle)")
        return bad, warn
    if not v.get("reachable"):
        bad.append(f"no MAVLink heartbeat: {v.get('error', 'unknown')}")
        return bad, warn
    if v.get("sysid") != sysid:
        bad.append(f"WRONG AIRCRAFT: answered as sysid {v.get('sysid')}, expected {sysid}")
    if v.get("armed"):
        bad.append("aircraft is ALREADY ARMED")
    if v.get("battery_pct") is not None and v["battery_pct"] < 30:
        bad.append(f"battery {v['battery_pct']}% below 30%")

    g = v.get("gps", {})
    if not g.get("position_usable"):
        warn.append("GPS not usable for position: " + ", ".join(g.get("gate_failures", []))
                    + " -- offboard/altitude arming is fine, takeoff and position modes "
                      "are not")
    if not v.get("home_set"):
        warn.append("home position not set")
    return bad, warn


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fleet", required=True)
    ap.add_argument("--json", action="store_true", help="machine-readable report on stdout")
    ap.add_argument("--no-vehicle", action="store_true")
    ap.add_argument("--timeout-s", type=float, default=120.0)
    args = ap.parse_args()

    drones = yaml.safe_load(Path(args.fleet).read_text())["drones"]
    with ThreadPoolExecutor(max(1, len(drones))) as ex:
        grounds = list(ex.map(lambda d: ground_ping(d["host"])
                              if d.get("host", "local") not in ("local", "localhost")
                              else {"loss_pct": 0.0, "avg_ms": 0.0}, drones))
        probes = list(ex.map(lambda d: probe(d, args.no_vehicle, args.timeout_s), drones))

    report, all_go = [], True
    for d, ground, p in zip(drones, grounds, probes):
        bad, warn = assess(d, ground, p)
        go = not bad
        all_go &= go
        report.append({"slot": d.get("slot"), "sysid": d["sysid"], "host": d.get("host"),
                       "verdict": "GO" if go else "NO-GO", "blockers": bad,
                       "warnings": warn, "ground": ground, "probe": p})

        v, g = p.get("vehicle", {}) or {}, (p.get("vehicle", {}) or {}).get("gps", {}) or {}
        log(f"\n=== ARTEX{d['sysid']} on {d.get('host')} -- "
            f"{'GO' if go else 'NO-GO'} ===")
        w = p.get("wifi", {})
        log(f"  ground LAN   {ground['loss_pct']:.0f}% loss, {ground['avg_ms']}ms")
        log(f"  backpack     {w.get('bssid')} {w.get('signal_dbm')}dBm, "
            f"{p.get('backpack_ping', {}).get('loss_pct')}% loss")
        log(f"  RF           down={p.get('backpack_counters', {}).get('packets_down')} "
            f"drops={p.get('backpack_counters', {}).get('drops_down')}")
        log(f"  relays       {len(p.get('relay_pids', []))}   "
            f"abort flag: {p.get('abort_flag') or 'none'}   "
            f"QGC: {'running' if p.get('qgc_running') else 'no'}")
        if v.get("reachable"):
            log(f"  vehicle      sysid {v.get('sysid')} armed={v.get('armed')} "
                f"mode={v.get('main_mode')} battery={v.get('battery_pct')}%")
            log(f"  GPS          fix{g.get('fix_type')} sats={g.get('sats')} "
                f"hacc={g.get('h_acc_m')}m pdop={g.get('pdop')} "
                f"usable={g.get('position_usable')}")
        for b in bad:
            log(f"  BLOCKER      {b}")
        for x in warn:
            log(f"  warning      {x}")

    log("\n" + ("[preflight] ALL AIRCRAFT GO" if all_go
                else "[preflight] NOT GO -- resolve every blocker above"))
    if args.json:
        print(json.dumps({"all_go": all_go, "aircraft": report}, indent=2))
    return 0 if all_go else 1


if __name__ == "__main__":
    raise SystemExit(main())
