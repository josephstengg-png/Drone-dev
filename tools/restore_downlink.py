#!/usr/bin/env python3
"""Put every MAVLink stream the 'indoors' downlink profile touched back to PX4's default rate. Runtime commands only.

    python3 tools/restore_downlink.py [--link udpout://127.0.0.1:14540] [--sysid 113]

Run it on the companion if an offboard-flight run with --downlink indoors died before it could restore the streams (a reboot of the flight
controller does the same). Refuses if anything else is holding the link or the aircraft is armed.
"""
import argparse
import sys
import time

import drone_agent as a


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udpout://127.0.0.1:14540")
    ap.add_argument("--sysid", type=int, default=113)
    args = ap.parse_args()
    conn = a.connect(args.link, args.sysid)
    armed = a.vehicle_armed(conn)
    if armed is not False:
        print("refusing: the aircraft is armed or silent", file=sys.stderr)
        return 2
    for _ in range(3):
        a.send_downlink(conn, a.downlink_restore_commands(), 0.15)
        time.sleep(1.0)
    n, t0 = 0, time.time()
    while time.time() - t0 < 15:
        if conn.recv_match(blocking=True, timeout=0.2) is not None:
            n += 1
    print(f"restored {len(a.downlink_restore_commands())} streams to PX4's defaults; downlink now {n / 15:.1f} msgs/s (about 27 when all are on)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
