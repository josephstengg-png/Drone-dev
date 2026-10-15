# First propeller flight: draft policy for decision

Status: DRAFT for the lab manager. Nothing here has been agreed. It is built from what has
and has not been proven on ARTEX113 and ARTEX115; see `docs/indoor-flight-readiness.md`.

## Scope of the first flights

- **Pilot-flown, Altitude mode, one aircraft at a time.** The computer cannot fly: its
  offboard stream is level attitude at zero thrust, and indoors there is no horizontal
  position, so no autonomous mode is available.
- **Tethered**, hover target 1 m (the rangefinder only helps the height estimate below
  1 m). The vertical fence is 3 m, but it is not evaluated without GPS and can trip on GPS
  height noise, so the tether is the real limit.
- **No autonomous mission and no Offboard flight.** Channel 7 centre is Offboard and in flight
  it is a fall. The pilot flies with channel 7 at the low position.

## People

| Role | Rule |
|---|---|
| Named pilot | One person, handset in hand, kill switch (right bumper) reachable, throughout |
| Spotter | One person watching the aircraft and the tether, not the screen |
| Operator | Runs the ground station; never touches the aircraft while armed |
| Sign-off | A named person, in writing, before props go on (decision needed: who) |

No one else in the area while armed.

## Gates before props go on

1. Preflight GO on the aircraft, and ESCs quiet after battery insertion (flip channel 7 away
   from Offboard and back if they beep; never arm while beeping).
2. Failsafe baseline applied and verified on both aircraft (land in place on every loss, fence
   action that needs no position, battery thresholds in order).
3. Safety controls proven on the bench for THIS aircraft: pilot takeover, arm switch, kill.
4. Battery voltage recorded; the pack is charged and not the lowest in the set.
5. Propellers: correct direction on each motor, tight, undamaged. Motor 3 has shown
   unexplained start failures earlier on 15 Oct; confirm it spins before props go on.
6. Tether tested at the length used and anchored; area clear.

## During the flight

- Arm with the handset, in Altitude mode, from the ground.
- First test: throttle up to just light on the ground, check motors respond evenly, then land.
  Only then a hover at about 1 m.
- **Stop conditions, any one:** unexplained drift beyond what the pilot corrects, any motor
  that sounds or behaves differently, any beep or warning from the aircraft, link or telemetry
  dropout. Land and disarm.
- **Emergency order:** channel 7 to low (pilot has it) first; arm switch next; kill bumper
  last, because it drops the aircraft.

## What this policy does not cover (not yet proven)

- Any autonomous or computer-flown behaviour.
- The abort-land button (does not exist yet).
- Behaviour on loss of the radio link with the aircraft in the air, indoors: configured to
  land in place, never observed.
- Free flight without a tether.

## Decisions needed from the lab manager

1. Who signs off props-on, and what do they sign?
2. Tether: what material, what length, and anchored to what?
3. Minimum crew size, and is a second observer required?
4. Does a first flight happen in the lab, or somewhere with netting?
5. How many successful short tethered flights before anything less constrained is considered?
