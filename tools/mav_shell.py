#!/usr/bin/env python3
"""Run PX4 nsh commands over MAVLink -- the console QGroundControl offers, headless.

    python3 tools/mav_shell.py --link udpout://127.0.0.1:14540 --sysid 113 \
        "listener vehicle_local_position -n 1"
    python3 tools/mav_shell.py --sysid 113 --file commands.txt
    python3 tools/mav_shell.py --sysid 113 -i                 # interactive

WHY. Several things worth knowing are uORB topics that PX4 never publishes over MAVLink --
`estimator_status_flags` is the important one, because it is the only place that says
which aiding sources the EKF is actually fusing. The only way to read them is the nsh
shell, and the only remote route to that shell is SERIAL_CONTROL with device
SERIAL_CONTROL_DEV_SHELL. QGroundControl has a console widget that does this, but QGC
cannot complete a session over an ELRS link (measured 7 Oct 2026), so on this project
there was no way in at all.

READ-ONLY BY HABIT, NOT BY ENFORCEMENT. nsh will happily run commands that reconfigure or
stop modules. This sends whatever you type. Prefer `listener`, `param show`, and the
various `status` verbs; be deliberate about anything else, especially while armed.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drone_agent as agent  # noqa: E402

SHELL_DEV = 10                     # SERIAL_CONTROL_DEV_SHELL
FLAG_REPLY = 1                     # SERIAL_CONTROL_FLAG_REPLY
FLAG_RESPOND = 2                   # SERIAL_CONTROL_FLAG_RESPOND
FLAG_EXCLUSIVE = 4
FLAG_MULTI = 16
CHUNK = 70                         # the SERIAL_CONTROL payload is a fixed 70 bytes
POLL_S = 0.25                      # how often to ask PX4 for more buffered output


def send(conn, text: bytes) -> None:
    for i in range(0, max(len(text), 1), CHUNK):
        part = text[i:i + CHUNK]
        conn.mav.serial_control_send(
            SHELL_DEV, FLAG_RESPOND | FLAG_EXCLUSIVE | FLAG_MULTI, 0, 0,
            len(part), part.ljust(CHUNK, b"\x00"))


def poll(conn) -> None:
    """Ask PX4 for more buffered output.

    The shell does not push its whole output unprompted: it fills a small buffer and
    waits to be asked again. A client that sends the command and then only listens gets
    the first chunk and nothing else, which truncates any long listing -- exactly what
    `listener vehicle_local_position` did here, cutting off before the validity flags.
    QGroundControl polls continuously; so must we.
    """
    conn.mav.serial_control_send(SHELL_DEV, FLAG_RESPOND | FLAG_EXCLUSIVE | FLAG_MULTI,
                                 0, 0, 0, b"\x00" * CHUNK)


def drain(conn, quiet_s: float, total_s: float) -> str:
    """Collect output until the shell has been quiet for `quiet_s`, polling as we go.

    There is no end-of-output marker, so quiet time is the only usable signal.
    """
    out, last, last_poll = bytearray(), time.time(), 0.0
    end = time.time() + total_s
    while time.time() < end:
        if time.time() - last_poll > POLL_S:
            poll(conn)
            last_poll = time.time()
        m = conn.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.3)
        if m is not None and m.count:
            out += bytes(m.data[:m.count])
            last = time.time()
        elif time.time() - last > quiet_s and out:
            break
    return out.decode(errors="replace")


def run(conn, cmd: str, quiet_s: float, total_s: float) -> str:
    # Nudge the shell so PX4 opens the session, and swallow the banner/prompt it sends.
    send(conn, b"\n")
    drain(conn, 0.6, 3.0)
    send(conn, cmd.encode() + b"\n")
    return drain(conn, quiet_s, total_s)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="*", help="nsh command; repeatable as separate args")
    ap.add_argument("--link", default="udpout://127.0.0.1:14540")
    ap.add_argument("--sysid", type=int, default=None)
    ap.add_argument("--file", help="file of commands, one per line")
    ap.add_argument("-i", "--interactive", action="store_true")
    ap.add_argument("--quiet-s", type=float, default=2.5,
                    help="silence that counts as end-of-output")
    ap.add_argument("--total-s", type=float, default=30.0, help="cap per command")
    ap.add_argument("--poll-s", type=float, default=POLL_S,
                    help="seconds between polls for more output; a lossy radio may do "
                         "better with a gentler rate")
    args = ap.parse_args()
    globals()["POLL_S"] = args.poll_s

    cmds = list(args.command)
    if args.file:
        cmds += [c for c in Path(args.file).read_text().splitlines()
                 if c.strip() and not c.strip().startswith("#")]

    conn = agent.connect(args.link, args.sysid)

    if args.interactive:
        print("[shell] interactive; Ctrl-D to leave", file=sys.stderr)
        while True:
            try:
                line = input("pxh> ")
            except (EOFError, KeyboardInterrupt):
                print(file=sys.stderr)
                return 0
            if line.strip():
                print(run(conn, line, args.quiet_s, args.total_s))
        return 0

    if not cmds:
        print("[shell] give a command, --file, or -i", file=sys.stderr)
        return 2

    for c in cmds:
        print(f"\n=== pxh> {c} ===", flush=True)
        print(run(conn, c, args.quiet_s, args.total_s), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
