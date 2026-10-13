#!/usr/bin/env python3
"""Run the simultaneous arm repeatedly and measure WHY it fails, not just that it did.

    python3 tools/arm_stability.py --fleet fleet-artex-pair.yaml --runs 6
    python3 tools/arm_stability.py --fleet fleet-artex-pair.yaml --runs 6 --hold-s 6

PROPS MUST BE OFF. Every run arms every aircraft in the fleet.

WHY THIS EXISTS. Simultaneous arming has failed twice, in two different ways, always on
the same aircraft, and never when that aircraft is armed on its own:

    12 Oct 2026  both armed, then 113 lost its offboard setpoint stream mid-hold
    13 Oct 2026  113 would not enter OFFBOARD at all; 115 was fine

Both were written up from a single run each, which is not evidence of anything. The
leading hypothesis is RF contention between two 2.4 GHz ELRS links driven hard at the same
moment -- but that was never tested, because the only numbers to hand were the backpack's
LIFETIME counters, and reading a lifetime total as a rate produced two wrong diagnoses in
one day (see history.md, 12 Oct). So this samples each backpack immediately before and
after every run and reports the DELTA, which is the measurement that can actually settle
it.

What it records per run: whether each aircraft reached READY, whether it armed, the arm
spread, the failure text, and per-aircraft uplink/downlink/drop deltas across the run.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from swarm_deploy import host_argv  # noqa: E402

PAIR = Path(__file__).resolve().parent / "pair_arm_test.py"


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def counters(drone: dict) -> dict:
    """The backpack's own counters, read from the companion that owns it."""
    argv, cwd = host_argv(drone, ["curl", "-s", "--max-time", "6",
                                  "http://10.0.0.1/mavlink"])
    try:
        r = subprocess.run(argv, cwd=cwd, capture_output=True, text=True, timeout=40)
        return json.loads(r.stdout).get("counters", {})
    except Exception:
        return {}


def sample_all(drones: list) -> dict:
    with ThreadPoolExecutor(max(1, len(drones))) as ex:
        return {d["sysid"]: c for d, c in zip(drones, ex.map(counters, drones))}


def delta(before: dict, after: dict, sysid: int) -> dict:
    b, a = before.get(sysid, {}), after.get(sysid, {})
    out = {}
    for k in ("packets_up", "packets_down", "drops_down"):
        if k in a and k in b:
            out[k] = a[k] - b[k]
    if out.get("packets_down") and "drops_down" in out:
        total = out["packets_down"] + out["drops_down"]
        out["loss_pct"] = round(100.0 * out["drops_down"] / total, 2) if total else 0.0
    return out


def parse_run(text: str, drones: list) -> dict:
    """Pull the per-aircraft outcome out of pair_arm_test's output."""
    res = {"verdict": "unknown", "spread_ms": None, "aircraft": {}}
    if "BOTH ARMED TOGETHER AND DISARMED" in text:
        res["verdict"] = "pass"
    elif "NOT A CLEAN PASS" in text:
        res["verdict"] = "fail"
    m = re.search(r"arm instants differ by (\d+) ms", text)
    if m:
        res["spread_ms"] = int(m.group(1))
    for d in drones:
        s = d["sysid"]
        a = {"ready": f"sysid {s}: READY" in text, "armed": False, "error": None}
        m = re.search(rf"\[result\] slot \d+ sysid {s}: armed=(\w+)[^\n]*", text)
        if m:
            a["armed"] = m.group(1) == "True"
            err = re.search(r"error=(.+?)(?: vehicle=|$)", m.group(0))
            if err:
                a["error"] = err.group(1).strip()
        res["aircraft"][s] = a
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fleet", required=True)
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--hold-s", type=float, default=6.0,
                    help="short by default: PX4 auto-disarms at COM_DISARM_PRFLT anyway")
    ap.add_argument("--settle-s", type=float, default=20.0,
                    help="pause between runs so the vehicles return to a resting state")
    ap.add_argument("--json", help="write the full record here")
    args = ap.parse_args()

    drones = yaml.safe_load(Path(args.fleet).read_text())["drones"]
    runs = []

    for i in range(1, args.runs + 1):
        log(f"\n================ run {i}/{args.runs} ================")
        before = sample_all(drones)
        t0 = time.time()
        p = subprocess.run([sys.executable, str(PAIR), "--fleet", args.fleet,
                            "--hold-s", str(args.hold_s)],
                           capture_output=True, text=True, timeout=600)
        took = time.time() - t0
        after = sample_all(drones)

        rec = parse_run(p.stdout + p.stderr, drones)
        rec["run"] = i
        rec["seconds"] = round(took, 1)
        rec["link"] = {d["sysid"]: delta(before, after, d["sysid"]) for d in drones}
        runs.append(rec)

        log(f"  verdict: {rec['verdict']}"
            + (f"   spread {rec['spread_ms']} ms" if rec["spread_ms"] else ""))
        for s, a in rec["aircraft"].items():
            log(f"    ARTEX{s}: ready={a['ready']} armed={a['armed']}"
                + (f"  error={a['error']}" if a["error"] else ""))
            d = rec["link"].get(s, {})
            if d:
                log(f"            link during run: up={d.get('packets_up')} "
                    f"down={d.get('packets_down')} drops={d.get('drops_down')} "
                    f"({d.get('loss_pct')}% loss)")
        if i < args.runs:
            time.sleep(args.settle_s)

    # ---- the summary that the single-run write-ups could not give -----------------
    passes = sum(1 for r in runs if r["verdict"] == "pass")
    log(f"\n================ {passes}/{len(runs)} clean passes ================")
    for d in drones:
        s = d["sysid"]
        ready = sum(1 for r in runs if r["aircraft"][s]["ready"])
        armed = sum(1 for r in runs if r["aircraft"][s]["armed"])
        ups = [r["link"].get(s, {}).get("packets_up") for r in runs]
        ups = [u for u in ups if u is not None]
        log(f"  ARTEX{s}: reached READY {ready}/{len(runs)}, armed {armed}/{len(runs)}"
            + (f", uplink per run median {statistics.median(ups):.0f} packets" if ups else ""))
        for r in runs:
            if r["aircraft"][s]["error"]:
                log(f"    run {r['run']}: {r['aircraft'][s]['error']}")
    spreads = [r["spread_ms"] for r in runs if r["spread_ms"]]
    if spreads:
        log(f"  arm spread: median {statistics.median(spreads):.0f} ms, "
            f"worst {max(spreads)} ms, over {len(spreads)} run(s) that armed together")

    if args.json:
        Path(args.json).write_text(json.dumps(runs, indent=2))
        log(f"  full record: {args.json}")
    return 0 if passes == len(runs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
