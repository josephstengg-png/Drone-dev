# CLAUDE.md — FlightPlanner (search-and-rescue drone)

This file travels with the repo, so it works on any machine. If you are reading it on a
freshly cloned laptop, start with **Bootstrap** below.

The fuller environment notes for the original workstation live in `docs/workstation.md`;
they describe one particular box and are history, not requirements.

## Bootstrap on a new machine

```bash
git clone <remote> FlightPlanner && cd FlightPlanner
python3 -m venv .venv && . .venv/bin/activate && pip install -e .
python3 tools/ssh_config.py --install     # companions become reachable by name
python3 tools/provision.py --all --verify # are the companions flight-ready?
python3 tools/preflight.py --fleet fleet-artex-pair.yaml   # GO / NO-GO per aircraft
```

You also need, and these are NOT in the repo:
- an ssh key the companions accept (`~/.ssh/id_ed25519`, see `companions.yaml`)
- the ELRS backpack WiFi PSK, if a companion ever needs re-pairing (never stored here)

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

**QGroundControl cannot be a ground station over ELRS.** Measured repeatedly: its connect
sequence and parameter download need far more round trips than the radio can carry, so it
shows "Not Ready", no Actions menu, and failed mission transfers. It is a telemetry
display here. Use the agent for anything that must work, and USB for parameter work.

## Safety architecture, in order of authority

1. **The RC transmitter.** Mode switch on channel 7 (slot 1 = Altitude), kill switch on
   channel 8 (right bumper). Reaches the flight controller with none of our software in
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

- Offboard **flight** is not implemented — only offboard arming. The streamed setpoint is
  level attitude at **zero thrust**, which is correct on a bench and would drop an
  aircraft in the air.
- The guard's in-air LAND branch has never run on hardware.
- A false "PILOT HAS IT" fires when PX4 drops out of offboard by itself; the guard reads
  its own consequence as a human takeover.
- Companion clocks have no time source and boot at 1970.
- Simultaneous arming works but with a ~3 s spread and one aircraft losing its setpoint
  stream; suspected RF contention, not yet measured properly.
