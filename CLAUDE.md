# CLAUDE.md — FlightPlanner (search-and-rescue drone)

This file travels with the repo, so it works on any machine. If you are reading it on a
freshly cloned laptop, start with **Bootstrap** below.

The fuller environment notes for the original workstation live in `docs/workstation.md`;
they describe one particular box and are history, not requirements.

## Bootstrap on a new machine

```bash
git clone <remote> FlightPlanner && cd FlightPlanner
bash tools/bootstrap_ground_station.sh --iface <wired-iface> --ip 192.168.144.51
python3 tools/provision.py --all --verify # are the companions flight-ready?
python3 tools/preflight.py --fleet fleet-artex-pair.yaml   # GO / NO-GO per aircraft
```

The bootstrap script builds the venv, installs the dependencies, configures the static
address on the DHCP-less drone LAN, installs the ssh config and reports what it cannot do
for you. `--check` verifies without changing anything.

You also need, and these are NOT in the repo:
- an ssh key the companions accept (`~/.ssh/id_ed25519`, see `companions.yaml`)
- a **manually configured static address** on the ground LAN -- that subnet has no DHCP,
  so a laptop on "wired, automatic" gets a link-local address and reaches nothing
- the ELRS backpack WiFi PSK, if a companion ever needs re-pairing (never stored here)

**Moving the ground station to a different machine: `docs/ground-station.md`.**

## How the system fits together

```
aircraft ──ELRS RF──► TX backpack ──WiFi/UDP──► Jetson ──UDP/localhost──► agent
                          10.0.0.1                  │
                                                    └── ssh ──► ground station (you)
```

Only the RF hop is lossy and slow (~0.7 s round trip). Everything our code does is shaped
around that. **The transport is not negotiable:** the backpack speaks UDP only, MAVLink is
a datagram protocol, and TCP would add head-of-line blocking and stale retransmissions to
a link that needs neither.

**The firmware is a fork.** Both airframes report PX4 **v1.16.0, git branch `Artex`**, a
vendor build dated 12 Nov 2025 — read off the aircraft with
`tools/mav_shell.py --sysid 113 "ver all"`. Upstream PX4 documentation is a guide here, not
an authority, particularly on EKF internals. Flight controller is a MicoAir743-AIO.

| File | What it is |
|---|---|
| `companions.yaml` | the Jetsons: address, MAC, which aircraft each owns |
| `artex-registry.yaml` | the transmitters, **by BSSID** — the only real identity they have |
| `fleet-*.yaml` | slot → sysid → host → link, per run |
| `tools/drone_agent.py` | one vehicle's agent; **runs on the companion** |
| `tools/safety_rules.py` | the abort ladder; pure logic, heavily tested |
| `tools/elrs_link.py` | the relay that straddles the backpack's asymmetric ports |
| `tools/preflight.py` | GO/NO-GO for the whole chain — run this first, always |
| `tools/provision.py` | bare companion → flight-ready, idempotent |
| `tools/takeover.py` | out-of-band abort; works if the orchestrator is dead |
| `tools/pair_arm_test.py` | all-or-nobody simultaneous arming |
| `tools/mav_shell.py` | PX4's nsh over MAVLink (short commands only) |
| `docs/history.md` | **the engineering log — newest first. Read the top before working.** |
| `docs/drone-control-scheme.md` | who can fly the aircraft and how control is taken back |
| `docs/ground-station.md` | setting up a new machine as the ground station |

## Working agreements — these were learned the hard way

**Never arm a real aircraft without a fresh, per-run confirmation from the operator that
props are off and the area is clear.** It does not carry over from an earlier run, even
one minutes ago. The operator can see the room; you cannot.

**Never arm when the DISARM might not be delivered.** A lost arm is a non-event; a lost
disarm is not. `preflight.py` refuses above 10% loss or 150 ms, and that limit exists
because a run was correctly held on 5 Oct 2026 for exactly this.

**Read before writing, on anything with parameters.** On 7 Oct a plan to set
`RC_MAP_FLTMODE=5` would have broken a working configuration — the aircraft was already on
channel 7. Back up first (`~/Desktop/*.params` on the old box; keep doing it somewhere),
ask, then restore if it was for a test.

**Verify with the instrument before theorising.** The recurring failure mode of this
project, mine included: on 12 Oct a QGC uplink was being dropped by a default I had set an
hour earlier, and the counter that said so (`mirror-up=0 mirror-blocked=4108`) was one I
had added for that purpose. Check it first.

**Cumulative counters are not rates.** `drops_down`, `packets_up` and friends are lifetime
totals since the backpack powered on. Twice in one day they were misread as current loss,
producing a wrong "17% packet loss" and a wrong diagnosis. Sample twice and difference.

**Ordinary operational state is not a fault.** A missing relay when the transmitters are
off, a Jetson that is simply powered down, a GPS with no fix indoors — say what normal
state would produce the symptom before raising an alarm.

**Kill by numeric PID, never by pattern.** `pkill -f` / `pgrep -f` match the shell running
them, including over ssh, and `[x]`-bracketing does not save you when the command line also
contains the real path. This cost three self-inflicted failures in one day.

**One relay per companion.** Two bind the same port and split the downlink, which looks
exactly like a dead aircraft. `preflight.py` checks this.

**QGroundControl does not need closing before a test any more** -- the relay's agent
priority (`--mirror-floor-s 5`, the service default since 14 Oct) holds it off while an
agent talks. If preflight warns that QGC runs with no agent priority, the relay predates
that change.

**QGroundControl cannot be a ground station over ELRS.** Measured repeatedly: its connect
sequence and parameter download need far more round trips than the radio can carry, so it
shows "Not Ready", no Actions menu, and failed mission transfers. It is a telemetry
display here. Use the agent for anything that must work, and USB for parameter work.

## Safety architecture, in order of authority

1. **The RC transmitter.** Mode switch on channel 7 (-100 Altitude, centre **Offboard --
   bench only**, +100 Position; changed 14 Oct 2026), kill switch on channel 8 (right
   bumper). Reaches the flight controller with none of our software in
   the path. Nothing below is a substitute.
2. **The guard on the companion** (`safety_rules.py`): WARN → HOLD → LAND → DISARM, on a
   severity-weighted error rate over a sliding minute, battery, tilt, telemetry gap and
   estimator health. Climb-only — recovery is a human decision. **It never disarms an
   aircraft that is airborne**, and it stands down entirely on a pilot takeover or kill.
3. **The operator**: `a` + ENTER in the pair-test console, or `tools/takeover.py --now`
   out-of-band over ssh, which survives the orchestrator dying.

`COM_RC_OVERRIDE` is deliberately left at PX4's default 1, so stick movement does **not**
break OFFBOARD. Taking over is meant to be a deliberate switch throw, not a knocked stick.

## Known-unfinished, so nobody assumes otherwise

**Blocks actual flight**
- **Geofence and RTL are far below the mission.** `GF_MAX_VER_DIST` is 5 m (115) / 7 m (113)
  against a 30/32 m survey, `RTL_RETURN_ALT=5`, `NAV_RCL_ACT=3`. Flown as-is the plan
  breaches the fence on the climb. Found 14 Oct by the mission-arm contract; unresolved.
- **113 and 115 are configured differently**: `NAV_DLL_ACT` 3 vs 0 (113 has no datalink-loss
  action upstream-semantics), `EKF2_GPS_CTRL` 7 vs 5, `GF_ACTION` 5 vs 2. Pick one baseline.
- Offboard **flight** is not implemented -- only offboard arming. The streamed setpoint is
  level attitude at **zero thrust**: right on a bench, would drop an aircraft in the air.
- The guard's in-air LAND branch has never run on hardware.

**Simultaneous arming is not simultaneous**
- 6-run campaign 14 Oct: 4/6 clean, spread median 1.7 s, worst 2.6 s. Cause is the design:
  `pair_arm_test` releases a barrier but each agent then arms independently with a 3 s retry.
  `drone_agent.hold_for()` (shared epoch) exists and `swarm_deploy` uses it; the pair test
  does not. **RF contention was tested and is NOT supported** (0% loss, equal uplink).
- `arm_stability.py` does not detect a backpack counter reset, so a run can report negative
  deltas.

**Known wrong behaviour**
- A false "PILOT HAS IT" fires when PX4 drops out of offboard by itself; the guard reads its
  own consequence as a human takeover.
- 113 occasionally refuses to enter OFFBOARD (13 Oct) or loses its setpoint stream (12 Oct);
  transient, only ever seen under simultaneous operation.

**Environment**
- Companion clocks have no time source and boot at 1970.
- GPS position wanders 4-7 m on a stationary aircraft near the hangars while reporting
  1-2 m accuracy. Do not treat h_acc as hold accuracy. Cause (multipath?) unconfirmed.
- **No optical flow sensor on either airframe** (operator-confirmed; no flow driver in the
  firmware). Aboard: IMU, baro, mag, a downward TFmini and an outdoor-only GPS. Indoors
  the EKF has height and attitude but NO horizontal velocity or position, so Position and
  Hold are unavailable and indoor hover needs a horizontal source that does not exist yet.
  `EKF2_OF_CTRL=1` is set but harmless. Do not plan around a flow sensor.
- **Channel 7 centre is Offboard**, and the agent's offboard stream is zero thrust. Flipping
  to centre in flight is a fall, not a hand-back. The agent refuses to resume.
- No Tuas obstacle data exists in the repo; the planner only knows the OpsArea polygon and
  cannot see buildings or containers.
- `preflight.py` and `provision.py` have no unit tests.
