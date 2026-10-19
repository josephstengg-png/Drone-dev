#!/usr/bin/env python3
"""Save EVERY parameter of an aircraft to a QGC-format .params file. Read-only.

    python3 tools/param_dump.py --sysid 115 --out ARTEX115-2026-10-19.params

Runs ON the companion. Asks for the full list, then re-requests any index that did not arrive
until the set is complete (the ELRS link drops packets, and QGC's own download gives up on it).
Refuses to write a partial file unless --allow-partial.

The output loads back into QGroundControl (Parameters -> Tools -> Load from file), over USB.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drone_agent as agent  # noqa: E402
from pymavlink import mavutil  # noqa: E402

INT_TYPES = {mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
             mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
             mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT32}


def collect(conn, got: dict, seconds: float) -> int | None:
    total, end = None, time.time() + seconds
    while time.time() < end:
        m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m is None:
            continue
        total = m.param_count
        got[m.param_index] = (m.param_id, agent.decode_param(m), m.param_type)
        end = max(end, time.time() + 2.0)        # keep listening while data flows
        if total and len(got) >= total:
            break
    return total


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udpout://127.0.0.1:14540")
    ap.add_argument("--sysid", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--allow-partial", action="store_true")
    args = ap.parse_args()

    conn = agent.connect(args.link, args.sysid)
    got: dict = {}
    conn.mav.param_request_list_send(conn.target_system, 1)
    total = collect(conn, got, 20.0)
    for rnd in range(1, 9):
        if total and len(got) >= total:
            break
        missing = [i for i in range(total or 0) if i not in got]
        print(f"[dump] round {rnd}: {len(got)}/{total}, re-requesting {len(missing)}", flush=True)
        for i in missing:
            conn.mav.param_request_read_send(conn.target_system, 1, b"", i)
            if i % 20 == 19:
                collect(conn, got, 0.5)
        total = collect(conn, got, 5.0) or total
    complete = bool(total) and len(got) >= total
    print(f"[dump] {len(got)}/{total} parameters {'COMPLETE' if complete else 'INCOMPLETE'}",
          flush=True)
    if not complete and not args.allow_partial:
        print("[dump] REFUSED to write a partial file", flush=True)
        return 1
    lines = [f"# Onboard parameters for sysid {args.sysid}, {len(got)} of {total}",
             "# Vehicle-Id Component-Id Name Value Type"]
    for _, (name, value, ptype) in sorted(got.items(), key=lambda kv: kv[1][0]):
        v = str(int(value)) if ptype in INT_TYPES else repr(float(value))
        lines.append(f"{args.sysid}\t1\t{name}\t{v}\t{ptype}")
    Path(args.out).write_text("\n".join(lines) + "\n")
    print(f"[dump] written {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
