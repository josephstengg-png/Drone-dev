#!/usr/bin/env python3
"""Why won't the handset arm it? Read-only, over USB. Sends only PARAM_REQUEST_READ.

    python3 arm_diag.py [seconds]      # default 120; flip switches while it runs

Prints the arming-relevant parameters once, then every change in mode, armed state,
ch3 (throttle) / ch6 (arm) / ch7 (mode) / ch8 (kill), RC rssi, and every STATUSTEXT.
"""
import sys
import time

from pymavlink import mavutil

PARAMS = ["COM_RC_IN_MODE", "RC_MAP_ARM_SW", "RC_MAP_KILL_SW", "RC_MAP_FLTMODE", "RC_MAP_THROTTLE",
          "COM_ARM_SWISBTN", "RC_ARMSWITCH_TH", "RC_KILLSWITCH_TH", "RC6_MIN", "RC6_MAX", "RC6_REV",
          "RC8_MIN", "RC8_MAX", "RC3_MIN", "RC3_MAX", "RC3_TRIM", "COM_FLTMODE1", "COM_FLTMODE4",
          "COM_FLTMODE6", "COM_RC_LOSS_T", "EKF2_ABL_LIM", "COM_ARM_IMU_ACC", "CBRK_USB_CHK"]
MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STAB"}

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
c = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200, source_system=250, source_component=191)
hb = c.wait_heartbeat(timeout=10)
if not hb:
    sys.exit("no heartbeat on /dev/ttyACM0")
print(f"sysid {c.target_system}")

got = {}
for name in PARAMS:
    for _ in range(3):
        c.mav.param_request_read_send(c.target_system, 1, name.encode(), -1)
        m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
        while m and m.param_id != name:
            m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
        if m:
            got[name] = m.param_value
            break
print("params:", "  ".join(f"{k}={v:g}" for k, v in got.items()))
missing = [p for p in PARAMS if p not in got]
if missing:
    print("not on this firmware / no reply:", " ".join(missing))

t0, last = time.time(), {}


def show(key, val, text):
    if last.get(key) != val:
        last[key] = val
        print(f"{time.time() - t0:7.1f}s  {text}", flush=True)


print(f"listening {dur:.0f} s -- flip ch7/ch8/ch6 now")
while time.time() - t0 < dur:
    m = c.recv_match(blocking=True, timeout=1)
    if not m or m.get_srcSystem() != c.target_system:
        continue
    t = m.get_type()
    if t == "HEARTBEAT" and m.get_srcComponent() == 1:
        mode = MODES.get((m.custom_mode >> 16) & 0xFF, str(m.custom_mode))
        armed = bool(m.base_mode & 128)
        show("hb", (mode, armed), f"mode {mode}  {'ARMED' if armed else 'disarmed'}")
    elif t == "RC_CHANNELS":
        q = lambda v: round(v / 50) * 50  # ignore jitter
        show("rc", (q(m.chan3_raw), q(m.chan6_raw), q(m.chan7_raw), q(m.chan8_raw), m.rssi),
             f"thr ch3 {m.chan3_raw}  arm ch6 {m.chan6_raw}  mode ch7 {m.chan7_raw}  "
             f"kill ch8 {m.chan8_raw}  rssi {m.rssi}  n {m.chancount}")
    elif t == "STATUSTEXT":
        print(f"{time.time() - t0:7.1f}s  TEXT [{m.severity}] {m.text}", flush=True)
    elif t == "SYS_STATUS":
        bad = m.onboard_control_sensors_enabled & ~m.onboard_control_sensors_health
        show("health", bad, f"unhealthy sensor bits 0x{bad:08x}  battery {m.voltage_battery} mV")
