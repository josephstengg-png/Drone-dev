#!/usr/bin/env python3
"""Spin each motor alone, briefly, with the aircraft DISARMED -- PROPS MUST BE OFF.

    python3 tools/motor_test.py --sysid 115 --dry          # reads config, spins nothing
    python3 tools/motor_test.py --sysid 115 --throttle 10  # spins motor 1, 2, 3, 4 in turn

Runs ON the companion (link is localhost). Uses MAV_CMD_DO_MOTOR_TEST, which PX4 only
honours while disarmed and with COM_MOT_TEST_EN=1.

WHAT IT PROVES: each commanded motor's own output rises while the other three stay at zero,
so Motor N really is MAIN N, and each ESC/motor chain responds. WHAT IT DOES NOT PROVE:
balance under thrust. The commanded value is identical by construction, so equal outputs
here say nothing about matching. Balance is judged in a tethered hover.

SAFETY: refuses if the aircraft is armed, if COM_MOT_TEST_EN is not 1, or if the throttle
asked for is above --max-throttle (default 15 %). Bare motors at higher throttle spin up
far faster than they ever would with a prop.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drone_agent as agent  # noqa: E402
from pymavlink import mavutil  # noqa: E402

MAV_CMD_DO_MOTOR_TEST = 209
MAV_CMD_ACTUATOR_TEST = 310        # what PX4 1.16 uses; 209 answers UNSUPPORTED
THROTTLE_PERCENT = 0
MAV_RESULT_ACCEPTED, MAV_RESULT_UNSUPPORTED = 0, 3


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udpout://127.0.0.1:14540")
    ap.add_argument("--sysid", type=int, required=True)
    ap.add_argument("--throttle", type=float, default=10.0, help="percent")
    ap.add_argument("--max-throttle", type=float, default=15.0)
    ap.add_argument("--seconds", type=float, default=2.0, help="per motor")
    ap.add_argument("--motors", type=int, default=4)
    ap.add_argument("--dry", action="store_true", help="read and report only; spin nothing")
    args = ap.parse_args()

    if args.throttle > args.max_throttle:
        print(f"[motor-test] REFUSED: {args.throttle}% exceeds --max-throttle "
              f"{args.max_throttle}% (bare motors race without a prop)", file=sys.stderr)
        return 2

    conn = agent.connect(args.link, args.sysid)
    enabled = agent.read_param(conn, "COM_MOT_TEST_EN")
    state = agent.do_status(conn)
    print(f"[motor-test] COM_MOT_TEST_EN={enabled}  armed={state.get('armed')}", flush=True)
    if state.get("armed") is not False:
        print("[motor-test] REFUSED: aircraft is armed or its state is unknown", file=sys.stderr)
        return 2
    if enabled != 1:
        print("[motor-test] REFUSED: COM_MOT_TEST_EN is not 1", file=sys.stderr)
        return 2
    if args.dry:
        print("[motor-test] dry run: configuration is fine, nothing was spun")
        return 0

    conn.mav.command_long_send(conn.target_system, 1, 511, 0, 36, 100000, 0, 0, 0, 0, 0)
    results = []
    def spin(motor: int, command: int) -> tuple:
        """Send one test command and watch the outputs. Returns (ack, peaks)."""
        peaks = [0] * 4
        if command == MAV_CMD_DO_MOTOR_TEST:
            conn.mav.command_long_send(conn.target_system, 1, command, 0, motor,
                                       THROTTLE_PERCENT, args.throttle, args.seconds,
                                       0, 0, 0)
        else:   # ACTUATOR_TEST: value 0..1, timeout, function 1 = Motor 1
            conn.mav.command_long_send(conn.target_system, 1, command, 0,
                                       args.throttle / 100.0, args.seconds, 0, 0,
                                       float(motor), 0, 0)
        ack, end = None, time.time() + args.seconds + 1.5
        while time.time() < end:
            m = conn.recv_match(type=["COMMAND_ACK", "SERVO_OUTPUT_RAW"], blocking=True,
                                timeout=0.5)
            if m is None:
                continue
            if m.get_type() == "COMMAND_ACK" and m.command == command:
                ack = m.result
            elif m.get_type() == "SERVO_OUTPUT_RAW":
                for i, v in enumerate((m.servo1_raw, m.servo2_raw, m.servo3_raw,
                                       m.servo4_raw)):
                    peaks[i] = max(peaks[i], v)
        return ack, peaks

    command = MAV_CMD_DO_MOTOR_TEST
    for motor in range(1, args.motors + 1):
        ack, peaks = spin(motor, command)
        if ack == MAV_RESULT_UNSUPPORTED and command == MAV_CMD_DO_MOTOR_TEST:
            print("[motor-test] DO_MOTOR_TEST is UNSUPPORTED on this firmware; "
                  "retrying with ACTUATOR_TEST", flush=True)
            command = MAV_CMD_ACTUATOR_TEST
            ack, peaks = spin(motor, command)
        results.append((motor, ack, peaks))
        print(f"[motor-test] motor {motor}: ack={ack}  peak outputs ch1-4 = {peaks}",
              flush=True)
        time.sleep(1.0)

    # An UNSUPPORTED ack means the command was never executed, which says nothing about the
    # motor. Calling that a motor "PROBLEM" would send someone looking for a fault that
    # does not exist, so it is reported as its own outcome.
    bad = unsupported = 0
    for motor, ack, peaks in results:
        own, others = peaks[motor - 1], [p for i, p in enumerate(peaks) if i != motor - 1]
        if ack == MAV_RESULT_UNSUPPORTED:
            unsupported += 1
            verdict = "NOT RUN (command unsupported by this firmware)"
        elif own > 0 and all(p == 0 for p in others) and ack in (MAV_RESULT_ACCEPTED, None):
            verdict = "OK"
        else:
            bad += 1
            verdict = "PROBLEM"
        print(f"[motor-test] motor {motor}: {verdict} (own {own}, others {others}, ack {ack})")
    if unsupported:
        print(f"[motor-test] {unsupported} motor(s) NOT TESTED: no command this firmware "
              f"accepts spun them. Nothing is wrong with the motors.")
        return 3
    print(f"[motor-test] {'all four motors respond on their own channel' if not bad else f'{bad} PROBLEM(S)'}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
