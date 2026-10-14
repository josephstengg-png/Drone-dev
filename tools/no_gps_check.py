#!/usr/bin/env python3
"""Reproduce the no-GPS estimator state on a disarmed aircraft, read it, and RESTORE.

    python3 tools/no_gps_check.py --sysid 115

Runs ON the companion. Temporarily sets EKF2_GPS_CTRL=0 (GNSS fusion off) so the EKF is left
with IMU, barometer, magnetometer and the rangefinder -- exactly the indoor configuration --
without needing the aircraft to lose its satellites. Refuses unless disarmed.

THE RESTORE IS THE POINT. The original value is read first and written to
/tmp/gps_ctrl_orig_<sysid>.txt BEFORE anything is changed, so a crash leaves a record. The
restore runs in a `finally`, retries four times, and reads the value back. If it cannot
verify the restore it says so loudly and exits non-zero.
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drone_agent as agent  # noqa: E402
import mav_shell as sh  # noqa: E402

FIELD = re.compile(r"^\s{2,}([a-z_][a-z0-9_]*):\s*(.+?)\s*$")
BITS = [("ATTITUDE", 1), ("VELOCITY_HORIZ", 2), ("VELOCITY_VERT", 4), ("POS_HORIZ_REL", 8),
        ("POS_HORIZ_ABS", 16), ("POS_VERT_ABS", 32), ("POS_VERT_AGL", 64),
        ("CONST_POS_MODE", 128), ("PRED_POS_HORIZ_REL", 256), ("PRED_POS_HORIZ_ABS", 512)]


def merged(conn, cmd: str, runs: int) -> dict:
    out = {}
    for _ in range(runs):
        for line in sh.run(conn, cmd, quiet_s=3.0, total_s=25.0).splitlines():
            m = FIELD.match(line)
            if m:
                out[m.group(1)] = m.group(2)
    return out


def estimator_flags(conn, timeout=8.0):
    m = conn.recv_match(type="ESTIMATOR_STATUS", blocking=True, timeout=timeout)
    return m.flags if m else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udpout://127.0.0.1:14540")
    ap.add_argument("--sysid", type=int, required=True)
    ap.add_argument("--settle-s", type=float, default=45.0)
    args = ap.parse_args()

    conn = agent.connect(args.link, args.sysid)
    st = agent.do_status(conn)
    if st.get("armed") is not False:
        print("[no-gps] REFUSED: aircraft is armed or its state is unknown", flush=True)
        return 2

    orig = agent.read_param(conn, "EKF2_GPS_CTRL")
    if orig is None:
        print("[no-gps] REFUSED: could not read EKF2_GPS_CTRL", flush=True)
        return 2
    Path(f"/tmp/gps_ctrl_orig_{args.sysid}.txt").write_text(f"{int(orig)}\n")
    print(f"[no-gps] original EKF2_GPS_CTRL = {orig}  (recorded)", flush=True)
    before = estimator_flags(conn)
    print(f"[no-gps] estimator flags BEFORE: 0x{before:04x}" if before is not None
          else "[no-gps] no flags before", flush=True)

    restored = False
    try:
        ok = agent.write_param_typed(conn, "EKF2_GPS_CTRL", 0)
        now = agent.read_param(conn, "EKF2_GPS_CTRL")
        print(f"[no-gps] wrote 0 -> read back {now}  (accepted={ok})", flush=True)
        if now != 0:
            print("[no-gps] write did not take; nothing to measure", flush=True)
            return 1

        t0, flags = time.time(), None
        while time.time() - t0 < args.settle_s:
            f = estimator_flags(conn, timeout=3.0)
            if f is not None:
                flags = f
                if not (f & (2 | 16)):          # horizontal velocity and abs position gone
                    break
            time.sleep(1.0)
        print(f"[no-gps] estimator flags AFTER {time.time()-t0:.0f}s: "
              f"0x{flags:04x}" if flags is not None else "[no-gps] no flags after", flush=True)
        if flags is not None:
            for name, bit in BITS:
                print(f"    {name:20} {'YES' if flags & bit else 'no'}", flush=True)

        print("[no-gps] --- ekf2 status ---", flush=True)
        for line in sh.run(conn, "ekf2 status", quiet_s=4.0, total_s=25.0).splitlines():
            if line.strip() and "nsh>" not in line:
                print("    " + line.strip(), flush=True)

        flags_m = merged(conn, "listener estimator_status_flags -n 1", 6)
        print("[no-gps] --- estimator_status_flags (6 runs merged) ---", flush=True)
        for k in sorted(k for k in flags_m if k.startswith("cs_")):
            print(f"    {k:26} {flags_m[k]}", flush=True)
        lp = merged(conn, "listener vehicle_local_position -n 1", 8)
        print("[no-gps] --- vehicle_local_position (8 runs merged), checklist fields ---",
              flush=True)
        for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "xy_global", "z_global",
                  "dead_reckoning", "dist_bottom_valid"):
            print(f"    {k:20} {lp.get(k, '(not received)')}", flush=True)
        return 0
    finally:
        for attempt in range(1, 5):
            try:
                agent.write_param_typed(conn, "EKF2_GPS_CTRL", int(orig))
                back = agent.read_param(conn, "EKF2_GPS_CTRL")
            except Exception as exc:                       # keep trying; this must land
                back = None
                print(f"[no-gps] restore attempt {attempt} error: {exc}", flush=True)
            if back == orig:
                restored = True
                print(f"[no-gps] RESTORED: EKF2_GPS_CTRL = {back} (verified, attempt {attempt})",
                      flush=True)
                break
            time.sleep(2.0)
        if not restored:
            print(f"[no-gps] !!! RESTORE NOT VERIFIED !!! EKF2_GPS_CTRL should be {int(orig)}. "
                  f"Original is in /tmp/gps_ctrl_orig_{args.sysid}.txt", flush=True)
            sys.exit(3)


if __name__ == "__main__":
    raise SystemExit(main())
