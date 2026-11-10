#!/usr/bin/env python3
"""Download flight-controller logs (.ulg) over USB with MAVLink's LOG_* protocol. Read-only.

    python3 tools/pull_fc_logs.py --list
    python3 tools/pull_fc_logs.py --out ~/fclogs 43 44 45     # by id; no ids = the newest

Run it on the companion the board is plugged into (jetson-2 has ARTEX113 on /dev/ttyACM0).

WHY. 29 Oct 2026: T3 run 1's agent trace was lost when USB dropped at the floor impact, and the
flight controller's own log was the only record of the flight. Earlier pulls "over USB" were
never written down. This is that method: LOG_REQUEST_LIST, then LOG_REQUEST_DATA for the whole
file, re-requesting any 90-byte chunks that did not arrive.

Over ELRS this is far too slow; use USB. If LOG_ENTRY never arrives, the board may still be
booting or its SD card busy -- try again in a minute.
"""
import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

CHUNK = 90


def list_logs(conn, timeout_s=15.0) -> dict:
    conn.mav.log_request_list_send(conn.target_system, 1, 0, 0xFFFF)
    entries, end = {}, time.time() + timeout_s
    while time.time() < end:
        m = conn.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)
        if m and m.size:
            entries[m.id] = m
            if len(entries) >= m.num_logs:
                break
    return entries


def download(conn, entry, out: Path) -> int:
    """Write one log to out; returns the number of chunks still missing (0 = complete)."""
    buf, have = bytearray(entry.size), set()

    def request(ofs, n):
        conn.mav.log_request_data_send(conn.target_system, 1, entry.id, ofs, n)

    request(0, entry.size)
    idle = time.time()
    while len(have) * CHUNK < entry.size:
        m = conn.recv_match(type="LOG_DATA", blocking=True, timeout=0.5)
        if m and m.id == entry.id:
            buf[m.ofs:m.ofs + m.count] = bytes(m.data[:m.count])
            have.add(m.ofs)
            idle = time.time()
        elif time.time() - idle > 1.0:
            missing = [o for o in range(0, entry.size, CHUNK) if o not in have]
            if not missing:
                break
            request(missing[0], min(entry.size - missing[0], CHUNK * 500))
            idle = time.time()
    conn.mav.log_request_end_send(conn.target_system, 1)
    out.write_bytes(buf)
    return len([o for o in range(0, entry.size, CHUNK) if o not in have])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", nargs="*", type=int)
    ap.add_argument("--device", default="/dev/ttyACM0")
    ap.add_argument("--out", default=".")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    conn = mavutil.mavlink_connection(args.device, baud=115200, source_system=250, source_component=190)
    conn.wait_heartbeat(timeout=10)
    entries = list_logs(conn)
    if not entries:
        print("no LOG_ENTRY received (board booting, or SD card busy?)", file=sys.stderr)
        return 2
    if args.list:
        for i in sorted(entries):
            e = entries[i]
            print(f"{i:4d} {e.size:9d} bytes  "
                  f"{time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(e.time_utc))} UTC")
        return 0
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    bad = 0
    for i in args.ids or [max(entries)]:
        if i not in entries:
            print(f"id {i}: not on the card", file=sys.stderr)
            bad += 1
            continue
        path = out_dir / f"log{i}_{entries[i].time_utc}.ulg"
        missing = download(conn, entries[i], path)
        print(f"id {i}: {entries[i].size} bytes -> {path}, missing chunks {missing}")
        bad += missing > 0
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
