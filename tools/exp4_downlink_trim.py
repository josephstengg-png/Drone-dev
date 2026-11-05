"""EXPERIMENT 4 (5 Nov 2026): does trimming the aircraft's DOWNLINK speed up / de-loss commands on the ELRS uplink?
ELRS F500, telemetry ratio 1:2 (9921 bps). Aircraft DISARMED. Only harmless things are sent: MAV_CMD_REQUEST_MESSAGE (HOME_POSITION),
zero-thrust setpoints, and runtime MAV_CMD_SET_MESSAGE_INTERVAL (not parameters; -1 = off during a 'trim' block, 0 = PX4 default).
Conditions: downlink normal/trim x setpoint rate 0/3/20 Hz, randomised per round. Always restores every interval to default at the end.
Stops if the aircraft arms."""
import json, random, sys, time
import drone_agent as a

ROUNDS, N, GAP, TIMEOUT, DRAIN, SETTLE = 3, 6, 3.0, 15.0, 8.0, 12.0
CONDS = [(d, hz) for d in ("normal", "trim") for hz in (0, 3, 20)]
KEEP = {0, 77, 253}          # HEARTBEAT, COMMAND_ACK, STATUSTEXT: never touched
c = a.connect("udpout://127.0.0.1:14540", 113)
out = open(sys.argv[1], "w")
rng = random.Random(5112026)
ids = set()

def sp():
    c.mav.set_attitude_target_send(int(time.monotonic() * 1000) & 0xFFFFFFFF, c.target_system, 1, 0b00000111,
                                   [1, 0, 0, 0], 0, 0, 0, 0.0)

def pump(secs, hz, st):
    end = time.time() + secs
    while time.time() < end:
        if hz and time.time() - st["tx"] >= 1.0 / hz:
            st["tx"] = time.time(); sp()
        m = c.recv_match(blocking=True, timeout=0.02)
        if m is not None and m.get_srcSystem() == c.target_system:
            k = m.get_type(); st["n"][k] = st["n"].get(k, 0) + 1
            if m.get_msgId() not in KEEP and k != "BAD_DATA":
                ids.add(m.get_msgId())
            if k == "HEARTBEAT" and m.get_srcComponent() == 1 and (m.base_mode & 128):
                sys.exit("ARMED: stopping")
            if k == "COMMAND_ACK" and m.command == 512 and st["wait"] is not None and st["rtt"] is None:
                st["rtt"] = time.time() - st["wait"]

def fresh():
    return dict(tx=0.0, n={}, wait=None, rtt=None)

def set_all(interval_us, hz, st):
    """interval_us -1 = off, 0 = default, for every periodic id seen; twice, pumping between so replies are read."""
    for _ in range(2):
        for mid in sorted(ids):
            c.mav.command_long_send(c.target_system, 1, 511, 0, mid, interval_us, 0, 0, 0, 0, 0)
            pump(0.12, hz, st)

try:
    base = fresh(); pump(25, 0, base)                      # learn which streams exist
    out.write(json.dumps(dict(event="streams", ids=sorted(ids), n=base["n"])) + "\n"); out.flush()
    print("streams:", sorted(ids), flush=True)
    for rnd in range(ROUNDS):
        order = CONDS[:]; rng.shuffle(order)
        for dl, hz in order:
            st = fresh()
            set_all(-1 if dl == "trim" else 0, hz, st)
            pump(SETTLE, hz, st)
            st = fresh(); t_block = time.time()
            for i in range(N):
                st["rtt"], st["wait"] = None, time.time()
                c.mav.command_long_send(c.target_system, 1, 512, 0, 242, 0, 0, 0, 0, 0, 0)
                t0 = time.time()
                while st["rtt"] is None and time.time() - t0 < TIMEOUT:
                    pump(0.1, hz, st)
                out.write(json.dumps(dict(round=rnd, dl=dl, hz=hz, i=i, rtt=st["rtt"])) + "\n"); out.flush()
                st["wait"] = None
                pump(max(0.0, GAP - (time.time() - t0)), hz, st)
            secs = time.time() - t_block
            out.write(json.dumps(dict(round=rnd, dl=dl, hz=hz, block_s=round(secs, 1),
                                      downlink_per_s=round(sum(st["n"].values()) / secs, 1))) + "\n"); out.flush()
            pump(DRAIN, 0, fresh())
            print(f"round {rnd + 1} {dl} {hz}Hz done: downlink {sum(st['n'].values()) / secs:.1f}/s", flush=True)
finally:
    st = fresh()
    set_all(0, 0, st)                                      # every interval back to PX4's default
    pump(15, 0, st)
    fin = fresh(); pump(20, 0, fin)
    out.write(json.dumps(dict(event="restored", downlink_per_s=round(sum(fin["n"].values()) / 20, 1))) + "\n"); out.flush()
    print(f"RESTORED: downlink {sum(fin['n'].values()) / 20:.1f}/s", flush=True)
print("DONE", flush=True)
