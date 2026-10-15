#!/usr/bin/env python3
"""Apply the agreed indoor safety baseline to a DISARMED aircraft, with a backup and a restore.

    python3 tools/param_baseline.py --sysid 113 --dry            # show the plan, change nothing
    python3 tools/param_baseline.py --sysid 113                  # backup, apply, verify
    python3 tools/param_baseline.py --sysid 113 --restore FILE   # put the backup back, verify

Runs ON the companion, like the other aircraft tools.

WHY THIS EXISTS. 113 and 115 were configured differently (link-loss action off, a fence
action that needs a position, and a battery emergency threshold ABOVE its low threshold), and
nobody could say which one was right. This sets both to one baseline chosen on 15 Oct 2026:
land in place on every loss, a 1 m vertical fence for the indoor demo, and sane battery
thresholds.

THE BACKUP IS THE POINT. The original values are read and written to a file BEFORE anything
is changed, so a crash leaves a record. Every write is read back and compared. Anything that
does not verify exits non-zero and prints the restore command.

NOTE FOR OUTDOOR WORK. GF_MAX_VER_DIST of 1 m fails the outdoor mission contract
(`fence_ceiling` in drone_agent.preflight_contract), by design. Restore from the backup, or
apply a different baseline, before an outdoor mission.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drone_agent as agent  # noqa: E402

# One baseline for both aircraft. The reasons are in docs/history.md, 15 Oct 2026.
INDOOR_BASELINE = {
    "GF_ACTION": 5,            # Land. Hold (2) needs a position, which indoors does not exist
    "GF_MAX_VER_DIST": 1.0,    # metres above home; the TFmini only helps below 1 m anyway
    "NAV_DLL_ACT": 3,          # Land on loss of the ground link (113 had 0 = disabled)
    "BAT_LOW_THR": 0.20,
    "BAT_CRIT_THR": 0.15,
    "BAT_EMERGEN_THR": 0.10,   # must be BELOW crit, which is below low
}
TOL = 1e-4


def same(a, b) -> bool:
    if a is None or b is None:
        return False
    return abs(float(a) - float(b)) < TOL


def make_plan(current: dict, desired: dict) -> list[dict]:
    """What would change. Pure, so the interesting decisions are testable."""
    plan = []
    for name, want in desired.items():
        have = current.get(name)
        plan.append(dict(name=name, have=have, want=want,
                         action="skip-unreadable" if have is None
                         else ("keep" if same(have, want) else "change")))
    return plan


def battery_order_ok(values: dict) -> bool:
    low, crit, emer = (values.get(k) for k in ("BAT_LOW_THR", "BAT_CRIT_THR", "BAT_EMERGEN_THR"))
    return None not in (low, crit, emer) and low > crit > emer


def write_and_verify(conn, name, value, attempts=3) -> bool:
    for _ in range(attempts):
        try:
            agent.write_param_typed(conn, name, value)
            if same(agent.read_param(conn, name), value):
                return True
        except Exception as exc:
            print(f"[baseline]   {name}: write error {exc}", flush=True)
        time.sleep(1.0)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udpout://127.0.0.1:14540")
    ap.add_argument("--sysid", type=int, required=True)
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--restore", metavar="FILE")
    ap.add_argument("--backup-dir", default=str(Path.home() / "FlightPlanner" / "backups"))
    args = ap.parse_args()

    conn = agent.connect(args.link, args.sysid)
    st = agent.do_status(conn)
    if st.get("armed") is not False:
        print("[baseline] REFUSED: aircraft is armed or its state is unknown", flush=True)
        return 2

    if args.restore:
        saved = json.loads(Path(args.restore).read_text())
        if saved.get("sysid") != args.sysid:
            print(f"[baseline] REFUSED: backup is for sysid {saved.get('sysid')}", flush=True)
            return 2
        bad = 0
        for name, val in saved["original"].items():
            if val is None:
                continue
            ok = write_and_verify(conn, name, val)
            bad += 0 if ok else 1
            print(f"[baseline] restore {name} -> {val}: {'verified' if ok else 'NOT VERIFIED'}",
                  flush=True)
        return 3 if bad else 0

    names = list(INDOOR_BASELINE)
    current = {n: agent.read_param(conn, n) for n in names}
    plan = make_plan(current, INDOOR_BASELINE)
    print(f"[baseline] ARTEX{args.sysid}: plan", flush=True)
    for p in plan:
        print(f"    {p['name']:18} {str(p['have']):>22} -> {str(p['want']):<8} {p['action']}",
              flush=True)
    if any(p["action"] == "skip-unreadable" for p in plan):
        print("[baseline] REFUSED: a parameter could not be read; will not guess", flush=True)
        return 2
    if args.dry:
        print("[baseline] dry run: nothing changed", flush=True)
        return 0

    Path(args.backup_dir).mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    backup = Path(args.backup_dir) / f"baseline-sysid{args.sysid}-{stamp}.json"
    backup.write_text(json.dumps(dict(sysid=args.sysid, taken=stamp, original=current),
                                 indent=2) + "\n")
    print(f"[baseline] original values saved: {backup}", flush=True)

    failed = []
    for p in plan:
        if p["action"] != "change":
            continue
        ok = write_and_verify(conn, p["name"], p["want"])
        print(f"[baseline] {p['name']:18} {p['have']} -> {p['want']}: "
              f"{'verified' if ok else 'NOT VERIFIED'}", flush=True)
        if not ok:
            failed.append(p["name"])

    final = {n: agent.read_param(conn, n) for n in names}
    print(f"[baseline] battery thresholds ordered low > crit > emergency: "
          f"{battery_order_ok(final)}", flush=True)
    if failed or not battery_order_ok(final):
        print(f"[baseline] !!! NOT ALL VERIFIED {failed}. Restore with: "
              f"python3 tools/param_baseline.py --sysid {args.sysid} --restore {backup}",
              flush=True)
        return 3
    print(f"[baseline] DONE and verified. Restore with: python3 tools/param_baseline.py "
          f"--sysid {args.sysid} --restore {backup}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
