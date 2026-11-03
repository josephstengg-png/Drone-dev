# Manual-takeover tests: agent flies offboard, pilot takes over at stage N

_Plan agreed with the operator 29 Oct 2026; kept as first written. Since then flight stages moved
to USB. Status 2 Nov: T1, T2 and T3 run 1 passed on ARTEX113. Current state: `docs/history.md`._

## Context
Goal: prove that the RC pilot can take the aircraft from the agent at **every** stage of a minimal
offboard flight (in place, 0 → 1 m → 0), with no false alarms. Each test is **separate**: the agent
runs stages 1…N, and the pilot takes over at stage N. Real aircraft, ELRS link only. The agent does
pre-arm, arm, take-off, hold, descent and landing in OFFBOARD.

## The seven tests

| Test | Agent runs | Pilot takeover at the last stage | Pass when |
|---|---|---|---|
| T1 | pre-arm (OFFBOARD selected, streaming, disarmed) | **kill ch8** | arming refused; agent logs "kill", stops commanding; no false alarm |
| T2 | pre-arm → arm | **kill ch8** (or arm switch ch6) | disarmed; agent stands down, no re-arm; no false alarm |
| T3 | pre-arm → arm → **take-off** (kill at ≤ 0.3 m) | **kill ch8** | disarmed in the air, drops ≤ 0.3 m; agent stands down; no false alarm |
| T4 | … → take-off → **hold 1 m** | **ch7 centre → −100 Altitude**, pilot flies, lands, disarms | mode ALTCTL ≤ 0.5 s after the flip; agent silent; pilot fully in control |
| T5 | … → hold → **descent** (at ~0.5 m) | ch7 → Altitude, pilot flies | same |
| T6 | … → descent → **landing** (≤ 0.15 m / touchdown) | ch7 → Altitude, pilot flies or settles | same; no bounce or re-take-off |
| T7 | … → landing → **disarm** stage | **kill ch8** before the agent's disarm | motors stop; agent stands down |

## T0 — the handset alone (added 3 Nov 2026; first test of every session)

The aircraft must obey the handset at all times, with or without our software. On 3 Nov the
operator powered up ARTEX113 and could not arm it from the handset. T1–T7 test the pilot taking
the aircraft back from the agent. T0 tests that the pilot never needed the agent in the first place.

**Conditions for a handset arm** (upstream PX4 1.16 `Commander.cpp:562-594` and
`manualControlCheck.cpp:60`; the firmware is a fork, so T0 is what confirms them):
- **a manual mode.** ch7 at −100 (Altitude). In Offboard (ch7 centre) the switch is refused:
  "Arming denied: switch to manual mode first" (seen 14 Oct). So arming from the handset at ch7
  centre fails by design;
- **throttle at or below centre** in Altitude, otherwise "Arming denied: throttle above center";
- **kill switch off** (ch8 1000), otherwise "Preflight Fail: Kill switch engaged";
- **health checks pass**, including accelerometer bias. These apply to every arm, the agent's too.

`COM_RC_IN_MODE` is 3 on ARTEX113 ("RC or joystick, keep first"). PX4 keeps whichever stick source
it hears first until reboot. If anything sent MAVLink `MANUAL_CONTROL` before the handset was heard,
the switches would be ignored until a reboot. None of our real-aircraft tools send it. The sim
harnesses that do now refuse any aircraft but the simulator. **Operator decision (parameter change:
back up, ask, restore):** set `COM_RC_IN_MODE` 0, RC only, so no other source can ever take the sticks.

**Procedure, props off, area clear.** Each step is done by the pilot on the handset. Nobody on a
computer touches the aircraft.
1. Before the battery goes in: ch7 −100 (Altitude), ch8 kill off, ch6 disarmed, throttle down.
2. Battery in. The ESCs are quiet; beeping means the aircraft booted in Offboard. Then wait 30 s and
   touch nothing, which keeps the accelerometer bias down.
3. ch6 to arm: the motors spin at idle within 1 s.
4. ch6 to disarm: the motors stop.
5. Arm again, then kill on ch8: the motors stop. Kill off, ch6 back to disarm.
6. Expected refusals, once per session, to prove the rules: ch7 centre then ch6 (refused), kill on
   then ch6 (refused), throttle above centre then ch6 (refused). Return all switches to step 1.

| Variant | Companion state | Pass |
|---|---|---|
| T0a | **Jetson off**: no software anywhere | steps 3–5 work 3 of 3, step 6 refuses as listed |
| T0b | handset switched on **after** the battery | steps 3–5 work once |
| T0c | Jetson on, relay running, no agent | steps 3–5 work once |
| T0d | agent streaming in the T1 pre-arm state; pilot flips ch7 to −100, then steps 3–5 | agent stands down; steps 3–5 work once |

**Gate:** T0a before anything else, every session. If the handset cannot arm, disarm and kill the
aircraft alone, nothing else runs that day. Run T0b–T0d once now, and again after any firmware,
parameter or companion-software change.

**Reading a refusal without a computer:** a refusal plays PX4's error tune if the board has a buzzer.
The reason is only readable over USB or telemetry. If a step fails unexpectedly, stop and read
STATUSTEXT over USB before trying anything else.

**Props on is the pilot's own flight** and is not part of this procedure.

If the pilot does **not** act within the wait window, the agent ends the test safely by itself:
- T1/T2: stay or go disarmed.
- T3–T5: PX4 LAND (Descend), then disarm.
- T6/T7: finish the landing and disarm.

## What must be built first (none of it exists for a real aircraft today)
1. **`drone_agent.py offboard-flight --stage {prearm,arm,takeoff,hold,descent,landing,disarm} --wait-s 15`**
   - Reuses `HeightHold` from tools/offboard_control.py (take-off ramp, frozen integrator, abort
     to PX4 LAND on lost height / tilt > 25° / height > 1.5 m / saturation).
   - Reuses `Guard` + pilot latch `_note_mode` (drone_agent.py:908), `PILOT_KILL_TEXTS`, and the
     `cmd_offboard_arm` skeleton (drone_agent.py:1196).
   - The agent runs to the named stage and holds there for the wait window. Height source:
     LOCAL_POSITION_NED (TFmini is blind below 0.4 m).
   - The SITL-only lock stays the default. The real aircraft needs explicit
     `--real-aircraft --expect-sysid 113`, a 1.2 m height cap and a tether.
2. **Tell a pilot takeover apart from PX4's offboard-loss failsafe.** Today both look like
   "PILOT HAS IT" (known bug).
   - Source of a takeover = ch7 / ch8 actually moved in RC_CHANNELS. A mode change without a switch
     move + "Failsafe activated" = link loss, logged as such.
   - Without this, "no false alarms" cannot be judged.
3. **Cut the setpoint rate to a steady ~5 Hz.** ELRS delivers 5–7/s anyway, and fewer, even packets
   may suffer fewer bursts. Tests for 1–3 against the existing patterns (tests/test_staged_safety.py,
   offboard_control tests).

## The ELRS risk, stated plainly
The height loop closes **across the radio**: ~0.7 s round trip, 5–7 setpoints/s, > 1 s gaps several
times a run (ulogs, 19 Oct). Each > 1 s gap makes PX4 drop to Altitude mid-air, then go back to
OFFBOARD when setpoints return.
- **Consequence for the test:** expect spurious mode changes. Item 2 classifies them, and any T3–T7
  run that has one is void, not a pass.
- **Consequence for safety:** the pilot's **throttle must sit centred** from arming onwards. Then a
  gap falls into Altitude = hold height, the same state as a takeover. The pilot is ready to fly
  from take-off in every airborne test.
- **Operator decision (param change: back up, ask, restore):** raise `COM_OF_LOSS_T` 1.0 → 2.0 s to
  ride through gaps. The trade-off is that PX4 then holds a stale thrust for up to 2 s. Recommend
  deciding after the sim with loss injected (below).

## Order of work
0. **First, in this order (operator, 29 Oct):**
   a. Save this plan as `docs/takeover-ladder-plan.md` in the repo, and publish it as an artifact
      (an HTML page of the same content).
   b. Back up everything discussed into history.md (both copies): an entry for 29 Oct covering the
      test design, the operator's corrections (agent flies T1–T7 in offboard; tests are separate,
      cumulative), the ELRS risk, the decisions still open, and the 27 Oct results it builds on.
   c. Only then switch to auto mode and build.
1. Build items 1–3; unit tests green.
2. **Sim, jetson-2:** T1–T7 × 10 in Gazebo with ELRS-like loss injected (5–7/s, bursty gaps) and
   ch7/ch8 driven through RC override. Gate: every takeover recognised, zero false alarms, no crash.
3. **Real, ARTEX113, props OFF:** T1, T2 × 3. Gate: 3/3.
4. Confirm option A params persisted after a power cycle; check 113's 2.7° bench roll with a level.
5. **Real, props ON, tethered** (tether shorter than the ceiling), one test per day-step: T3 → T7,
   ≥ 2 clean runs each before moving on. Stop the ladder at the first unexplained event.

Every real run, no carry-over:
- T0a passed this session (handset alone arms, disarms, kills)
- `preflight.py` GO (< 10 % loss, < 150 ms)
- fresh props-off/props-on-tethered + area-clear confirmation
- guard on **without `--allow-kill`** (DShot scaling unconfirmed)
- disarm confirmed
- ulog pulled over USB, entry in history.md

Measured per run:
- switch-to-mode-change latency
- agent stand-down time
- height and tilt at handover
- setpoint delivery rate and longest gap
- failsafes, classified
