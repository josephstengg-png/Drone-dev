"""Read-only probe of how fast the height feed (LOCAL_POSITION_NED) arrives over the radio, 4 Nov 2026.

    cd ~/FlightPlanner/tools && python3 height_feed_probe.py     # on the companion, aircraft DISARMED

Measures 25 s at the default rate, then with the rate asked at 5 Hz and at 10 Hz, and puts the default back. It sends only
MAV_CMD_SET_MESSAGE_INTERVAL (a runtime stream rate, not a parameter) and stops at once if it sees the aircraft armed.
Result on ARTEX113: 0.48 Hz (longest gap 2.8 s) -> 1.64 Hz (0.9 s) -> 3.0 Hz (1.0 s), total traffic unchanged.
"""
import sys, time
import drone_agent as a
from pymavlink import mavutil
c = a.connect("udpout://127.0.0.1:14540", 113)
SET = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
IDS = {"LOCAL_POSITION_NED": 32, "DISTANCE_SENSOR": 132}

def interval(msg_id, hz):   # hz 0 = back to the default rate
    c.mav.command_long_send(c.target_system, 1, SET, 0, msg_id, int(1e6 / hz) if hz else 0, 0, 0, 0, 0, 0)

def measure(label, secs=25):
    t0, n, st, armed_seen = time.time(), {}, [], False
    while time.time() - t0 < secs:
        m = c.recv_match(blocking=True, timeout=0.1)
        if m is None or m.get_srcSystem() != c.target_system:
            continue
        k = m.get_type(); n[k] = n.get(k, 0) + 1
        if k == "LOCAL_POSITION_NED": st.append(time.time())
        if k == "HEARTBEAT" and m.get_srcComponent() == 1 and (m.base_mode & 128): armed_seen = True
    gaps = [b - a_ for a_, b in zip(st, st[1:])]
    tot = sum(n.values())
    print(f"{label:34s} height {len(st)/secs:.2f} Hz  mean gap {sum(gaps)/len(gaps) if gaps else float('nan'):.1f}s max gap {max(gaps) if gaps else float('nan'):.1f}s | "
          f"all messages {tot/secs:.0f}/s | HEARTBEAT {n.get('HEARTBEAT', 0)/secs:.1f}/s RC {n.get('RC_CHANNELS', 0)/secs:.1f}/s ATT {n.get('ATTITUDE', 0)/secs:.1f}/s{'  ARMED!' if armed_seen else ''}", flush=True)
    if armed_seen: sys.exit("armed: stopping")

try:
    measure("baseline (default rates)")
    interval(32, 5)
    time.sleep(1.0)
    measure("LOCAL_POSITION_NED asked at 5 Hz")
    interval(32, 10)
    time.sleep(1.0)
    measure("LOCAL_POSITION_NED asked at 10 Hz")
finally:
    interval(32, 0)           # back to the default rate
    print("restored the default rate for LOCAL_POSITION_NED", flush=True)
