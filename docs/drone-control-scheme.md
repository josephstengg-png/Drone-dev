# Drone control scheme — who is flying, and how control is taken back

ARTEX115 (sysid 115, jetson-1) and ARTEX113 (sysid 113, jetson-2).
Last checked against PX4 1.16 parameter metadata, 6 Oct 2026.
Handset channel assignments confirmed 9 Oct 2026.

## Who can be flying

| Source | How it reaches the aircraft | Used for |
|---|---|---|
| RadioMaster Pocket (ELRS) | RF straight to the receiver on the airframe | manual flight, and taking control back |
| Companion (`drone_agent.py` on the Jetson) | WiFi → ELRS backpack → relay → MAVLink | offboard arming tests, mission start/abort |
| Ground station (workstation) | ssh to the Jetson | authorising a run, aborting one |

Only the first has no software of ours in its path. Everything the companion and the
ground station can do depends on WiFi, the backpack, a relay process and ssh all working —
which is the chain an emergency may already have broken.

## Taking control back — in the order you should reach for them

### 1. The RC switch (authoritative)

A switch position on the Pocket requests a flight mode, and PX4 honours it directly.

**Channel 7 mapping, changed by the operator 14 Oct 2026 and read live off both aircraft:**

| Switch | Value | Slot | Mode | Who is flying |
|---|---|---|---|---|
| low | -100 (1000 us) | 1 | **Altitude** | the pilot -- this is the takeover position |
| centre | 0 (1500 us) | 4 | **Offboard** | the agent's stream -- **bench only** |
| high | +100 (2000 us) | 6 | **Position** | the pilot, needs a horizontal fix |

Before 14 Oct centre was Position. **Centre is now Offboard on both aircraft**, so it is no
longer a safe resting position for the switch in flight: the agent's offboard stream is
level attitude at ZERO thrust. If the pilot flips to centre after a takeover the agent
refuses to resume (`rc_handback_requested`, `resumed: false`) and, because it streams
nothing, PX4 denies the mode itself. Slot 2 differs between the aircraft (115 Position, 113
unassigned) but is not on a three-position switch's travel, so it has no effect.
Stick movement is a separate mechanism governed by `COM_RC_OVERRIDE`, and on this project
it is **deliberately disabled for OFFBOARD** (see the parameter table below) — so during an
offboard run the sticks are inert and **the channel 7 switch is the only way to take over**.

Needs doing in two places:

**On the handset (EdgeTX).** Assign a physical switch to a channel. A 6-position mode
channel is the usual arrangement; a 2-position channel is enough for the kill switch.
Nothing in the repo can set this — it lives on the Pocket.

| Role | Physical switch | Channel | Confirmed |
|---|---|---|---|
| Flight-mode switch | _(which switch not yet recorded)_ | **7** | 9 Oct 2026, both the 113 and 115 handsets |
| Kill switch | **right bumper** | **8** | 9 Oct 2026 |

Both handsets use the same channel assignment, so a procedure written for one applies to
the other. Read back off ARTEX113 on 6 Oct, the aircraft agrees with the handset:
`RC_MAP_FLTMODE=7`, `RC_MAP_KILL_SW=8`, `COM_FLTMODE1=1` (Altitude), `COM_FLTMODE4=7` (Offboard), `COM_FLTMODE6=2` (Position).
**ARTEX115's parameters have never been read**, so its side of this is assumed, not
verified — run `tools/rc_takeover.py --fleet fleet-artex115.yaml` once that aircraft is
reachable.

**On each aircraft** (QGroundControl, or `drone_agent.py prepare --set NAME=VALUE`):

| Parameter | Set to | Why |
|---|---|---|
| `COM_RC_IN_MODE` | 0 (RC only) or 2 (RC and joystick) | 4 means stick input is ignored entirely |
| `COM_RC_OVERRIDE` | **left at 1, deliberately** | Bitmask: bit 0 = stick override in AUTO, bit 1 = in OFFBOARD. PX4 ships 1, so the sticks CANNOT break OFFBOARD. Decided 6 Oct 2026 to keep it that way, so a knocked stick cannot drop an aircraft mid-run — **the mode switch on channel 7 is the takeover path**. Set 3 only if you later want the sticks to override too |
| `COM_RC_STICK_OV` | 30 (%) | how far a stick must move to count as an override |
| `RC_MAP_FLTMODE` | the mode channel | otherwise no switch position can request anything |
| `COM_FLTMODE1..6` | at least one hand-flown mode | Manual 0, Altitude 1, Position 2, Acro 6, Stabilized 8, Position Slow 9. Mission 3, Hold 4 and Return 5 are **still the autopilot flying**, not you |
| `RC_MAP_KILL_SW` | the kill channel | cuts the motors without the flight stack's cooperation. It will drop the aircraft, so it is for a crash already in progress |

Verify what is actually on the aircraft, rather than what you believe you set:

    cd ~/drone-dev/FlightPlanner
    python3 tools/rc_takeover.py --fleet fleet-artex-pair.yaml      # read-only

QGroundControl > Vehicle Setup > Flight Modes is the easiest way to fill in the mode
channel and its six slots, because it shows the switch moving as you flip it.

**What the companion does when you take over.** The agent watches for the transition out of
OFFBOARD or AUTO into a hand-flown mode. On seeing it, it latches "pilot has it" and stands
down for good: no more setpoints, no mode changes, no attempt to re-enter offboard, no
automatic abort, and **it will not disarm while the aircraft is airborne**. It reports an
`rc_takeover` event and gets out of the way. Two aborts fighting each other is worse than
either one.

### 2. The guard on the companion (autonomous)

`tools/safety_rules.py`, carried out by `drone_agent.py`. Works with the ground station
switched off, needing only the companion-to-aircraft link. Four rungs, climb-only:
WARN → HOLD (AUTO.LOITER) → LAND (AUTO.LAND, then disarm once down) → DISARM (on the
ground only). Triggers on a severity-weighted error rate over a sliding minute, battery,
tilt, telemetry gap, estimator health. Full table in `FlightPlanner/README.md`.

### 3. The ground station

    # fastest: in the pair_arm_test console, type 'a' + ENTER
    python3 tools/takeover.py --fleet fleet-artex-pair.yaml --now     # out-of-band, over ssh
    python3 tools/takeover.py --fleet fleet-artex-pair.yaml --clear   # before the next run

The out-of-band path writes a flag file each agent polls, so it survives the orchestrator
being wedged or killed.

## Still open

- **ARTEX115 has never been read.** Channel 7 / channel 8 are confirmed on the handset for
  both aircraft and confirmed on ARTEX113's parameters, but 115's side is assumed. Run
  `tools/rc_takeover.py --fleet fleet-artex115.yaml` once it is reachable.
- **Which physical switch drives the mode channel is not recorded** — the kill switch is
  the right bumper; the mode switch has not been named. Worth writing down, because the
  takeover path depends on reaching for the correct one under pressure.
- **The takeover has never been exercised in the air.** On the bench, 9 Oct, flipping
  channel 7 to Altitude latched "pilot has it" on ARTEX113. The new hand-back refusal
  (centre = Offboard) has been unit-tested but not yet exercised on an aircraft.
- HOLD and the in-air LAND path have never run on hardware — every test so far has been a
  props-off bench arm.
- A backup of ARTEX113's 1,080 parameters, taken before any change, is at
  `~/Desktop/ARTEX113-params-before-2026-10-07.params`.

## Indoor flight: what the airframes actually carry (confirmed 14 Oct 2026)

**No optical flow sensor on either aircraft.** The operator confirmed it, and the firmware
agrees: `pmw3901`, `paw3902`, `paa3905`, `px4flow` and `thoneflow` are all "command not
found" over the nsh shell on both. What is aboard: IMU, barometer, magnetometer, a downward
**TFmini** on `/dev/ttyS3` (driver running, 0 comm errors), and a GPS module that is useless
indoors. With GPS gone the EKF reports attitude, vertical velocity, vertical position and
height above ground, and **no horizontal velocity or position**.

An IMU and a single downward beam give height, not horizontal motion. Horizontal position
from the IMU alone is a double integral of acceleration and drifts as t^2: a residual bias
of 0.05 m/s^2 is about 2.5 m after 10 s. So within these limits the EKF can hold **height
and attitude**, and PX4's Position and Hold modes are not available indoors.
