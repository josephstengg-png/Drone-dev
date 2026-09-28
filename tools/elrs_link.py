#!/usr/bin/env python3
"""Straddle the ELRS backpack's asymmetric UDP ports so the agent sees a normal link.

    python3 tools/elrs_link.py --agent-port 14540
    python3 tools/elrs_link.py --stats            # one-shot, prints module counters

WHY THIS EXISTS. The RadioMaster Pocket's TX backpack (firmware 1.5.5) bridges MAVLink
over UDP, but not symmetrically -- measured 28 Sep 2026 from jetson-1:

    GET http://10.0.0.1/mavlink
    {"enabled":true,"ports":{"listen":14555,"send":14550},"ip":{"gcs":"IP UNSET"},...}

It accepts GCS traffic on 14555 and emits vehicle traffic to 14550, and it learns which
host to emit to from the first packet that arrives on 14555 -- until then `gcs` reads
"IP UNSET" and nothing comes back however long you listen. pymavlink has no connection
string for "bind one port, send to another": `udpin` binds and can only reply to a peer
it has already heard from (so it can never speak first, which a mission upload must),
and `udpout` sends from an ephemeral port the module will never target. This relay owns
one socket bound to 14550 that sends to 10.0.0.1:14555, and presents the agent an
ordinary endpoint on localhost, so drone_agent.py stays unaware of any of it.

Uplink is proven: 100 MISSION_ITEM_INT/s (4.9 kB/s) accepted with zero loss, 5.8% loss at
200/s. Downlink is NOT yet proven -- it needs a powered aircraft with a bound receiver,
and the RF hop is far narrower than the WiFi hop measured here.
"""

from __future__ import annotations

import argparse
import json
import select
import socket
import sys
import time
import urllib.request

MODULE_IP = "10.0.0.1"
MODULE_LISTEN = 14555      # module's ear  -- we send here
MODULE_SEND = 14550        # module's mouth -- we bind here


def counters(ip: str = MODULE_IP) -> dict:
    try:
        with urllib.request.urlopen(f"http://{ip}/mavlink", timeout=5) as r:
            return json.load(r)
    except Exception as exc:
        return {"error": str(exc)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--module-ip", default=MODULE_IP)
    ap.add_argument("--module-listen", type=int, default=MODULE_LISTEN)
    ap.add_argument("--module-send", type=int, default=MODULE_SEND)
    ap.add_argument("--agent-port", type=int, default=14540,
                    help="localhost port the agent connects to with udpout://")
    ap.add_argument("--stats", action="store_true", help="print module counters and exit")
    ap.add_argument("--report-s", type=float, default=10.0)
    args = ap.parse_args()

    if args.stats:
        print(json.dumps(counters(args.module_ip), indent=2))
        return 0

    radio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    radio.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    radio.bind(("0.0.0.0", args.module_send))

    agent = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    agent.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    agent.bind(("127.0.0.1", args.agent_port))

    module = (args.module_ip, args.module_listen)
    agent_addr: tuple[str, int] | None = None
    up = down = dropped = 0
    last = time.time()

    print(f"[elrs] agent udp/{args.agent_port} <-> module {module[0]}:{module[1]} "
          f"(downlink on {args.module_send})", flush=True)

    while True:
        ready, _, _ = select.select([radio, agent], [], [], 1.0)
        for s in ready:
            try:
                data, addr = s.recvfrom(4096)
            except OSError:
                continue
            if s is agent:
                agent_addr = addr
                radio.sendto(data, module)
                up += 1
            else:
                down += 1
                if agent_addr is None:
                    dropped += 1       # vehicle spoke before the agent attached
                else:
                    agent.sendto(data, agent_addr)

        now = time.time()
        if now - last >= args.report_s:
            c = counters(args.module_ip).get("counters", {})
            print(f"[elrs] up={up} down={down} no-agent-drop={dropped} module={c}", flush=True)
            last = now


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[elrs] stopped", flush=True)
