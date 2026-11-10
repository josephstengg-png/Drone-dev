# Indoor flight readiness: ARTEX113 and ARTEX115

Status as of the engineering log entry of 15 Oct 2026. Every figure below is taken from
`docs/history.md`, where the date and the run it came from are recorded.

## 1. Bottom line

| Question | Answer |
|---|---|
| Can the aircraft be operated safely on the bench, props off? | **Yes.** Exercised repeatedly on both. |
| Can a **pilot-flown** indoor hover be justified? | **Conditionally.** The aircraft can hold height without GPS. It has never been flown. The conditions are in section 6. |
| Can **autonomous** indoor hold be justified? | **No, not with this hardware.** There is no sensor for horizontal motion. |
| Is the safety system complete? | **No.** Its layers work on the bench. Several have never been tested in the air. |

The decision for the discussion is the **scope**: pilot-flown indoor hover is within reach;
autonomous hold needs new hardware and a firmware build that includes its driver, and the
source for that build is not in this repository.

## 2. What the aircraft can sense indoors

| Sensor | State |
|---|---|
| Two IMUs (two independent estimators running) | working, healthy, both calibrated |
| Barometer | working |
| Magnetometer | working outdoors; **never evaluated indoors** |
| Downward rangefinder (TFmini) | working at driver level; **never used in flight** |
| GPS | present, **useless indoors** |
| Optical flow | **none fitted**; no flow driver exists in the firmware |
| External vision / motion capture | none |

**With GPS switched off, measured on both aircraft:** the estimator holds attitude and height
but reports **no horizontal position and no horizontal velocity**. Position, Hold, Mission
and Return modes depend on those and are unavailable.

**Why an IMU alone cannot replace them:** horizontal position from an IMU is a double
integral of acceleration, so error grows with the square of time. A residual bias of
0.05 m/s² is about **2.5 m after 10 s and 10 m after 20 s**. This is physics, not tuning.

**Rangefinder caveats:** it only assists the height estimate **below 1.0 m** on both
aircraft (the setting is 1.0; the PX4 default is 5.0). On the ground it reads below its own
40 cm minimum. It has not been tested at hover height.

## 3. Safety layers, in order of authority

| Layer | In plain terms | Evidence |
|---|---|---|
| **1. Pilot's handset** | Mode switch, arm switch and kill switch act on the flight controller directly, with none of our software in the way | see table below |
| **2. Watchdog on the companion** | Watches the aircraft's own warnings; pause, then land. Never cuts motors in the air. Steps aside when the pilot takes over or kills | Fired on hardware once wrongly (kill-switch messages, fixed) and correctly on real offboard-loss failsafes. **Never in the air.** One known fault remains: it can mistake PX4 leaving Offboard by itself for a pilot takeover |
| **3. Operator abort** | Keyboard abort from the laptop, works even if the control program hangs | Out-of-band route proven on 113; keyboard route in simulation only |
| **4. Pre-flight gates** | Refuses to start on a bad link, wrong transmitter, stale abort flag; a person must state props are off every run; two aircraft arm together or not at all | In use on hardware every run |
| **5. Flight controller's own rules** | 10 s auto-disarm, offboard-loss failsafe, geofence | Auto-disarm observed; the others unobserved indoors |

## 4. Pilot takeover protocol

**Switches on the handset:**

| Control | Position | Meaning |
|---|---|---|
| Channel 7 (mode) | low (−100) | **Pilot is flying** (Altitude mode). The takeover position |
| Channel 7 (mode) | centre | Computer's offboard mode. **Bench only. In flight this is a fall** |
| Channel 7 (mode) | high (+100) | Pilot is flying (Position; needs GPS, **unavailable indoors**) |
| Channel 6 (arm) | 1000 / 2000 µs | Disarm / arm. Disarms in any mode; will not start arming in Offboard |
| Channel 8 (kill, right bumper) | pressed | **Cuts the motors. Drops the aircraft.** Last resort |

**In an emergency, in this order:**
1. Flip channel 7 to **low**. The computer stops commanding and the pilot has the aircraft.
2. If it is on the ground and still armed: use the **arm switch**, or wait for the flight
   controller's own 10 s timer.
3. If it is falling or out of control: press the **kill bumper**.

**What the computer does on takeover:** it stops sending anything, including any disarm, and
warns "STILL ARMED" every 5 s. Disarming is deliberately left to the pilot. A commanded disarm
was unreliable (2 s in one run, failed for 12+ s in the next) and would fight a pilot who is
about to fly.

**What the pilot must not do:** leave channel 7 at centre in flight, or rely on the sticks to
override. Stick override is intentionally off in Offboard; the switch is the takeover path.

**Proven on hardware:**

| Protocol | ARTEX113 | ARTEX115 |
|---|---|---|
| Takeover (channel 7 to low) | yes | yes |
| Refusal to resume after a takeover | yes | yes |
| Kill switch cuts motors, disarms 5 s later | **yes** | **not tested** |
| Arm switch disarms an agent-armed aircraft | **not tested** | **yes** |
| Computer's "still armed" warning, no disarm sent | not tested | yes |

## 5. Risks specific to flying indoors

1. **Failsafes that need a position do not exist indoors.** Several of the aircraft's
   configured failsafe actions assume a position estimate. What the aircraft actually does
   indoors in those cases has **not been observed**.

   | Setting | ARTEX115 | ARTEX113 |
   |---|---|---|
   | Loss of radio control | Land in place | Land in place |
   | Loss of ground link (`NAV_DLL_ACT`) | Land | **None (disabled)** |
   | Geofence breach (`GF_ACTION`) | Land | **Hold**, which needs a position |
   | Loss of computer's offboard stream | Position mode | Position mode |

   113 and 115 are **not configured alike**, and 113 has two settings that would not work
   indoors.
2. **The vertical fence is measured from Home** (5 m on 115, 7 m on 113). Whether it operates
   with no position and no home indoors is **unverified**.
3. **Heading comes from the magnetometer**, and indoor steel structures disturb it. In one
   run the two aircraft reported different magnetometer-fusion flags (113 false outdoors,
   true in the no-GPS run), which has not been explained. Not evaluated indoors.
4. **The computer cannot fly.** Its offboard mode only arms; its setpoint is level attitude at
   zero thrust. Centre on channel 7 is therefore unsafe in flight.
5. **The kill switch drops the aircraft.** Indoors that means onto equipment or people.
6. **There is no containment policy yet** (tether, net, minimum people, who may authorise
   props-on).

## 6. What has never been tested

| Item | Why it matters |
|---|---|
| **Any flight with propellers** | Everything so far is bench, props off |
| Kill switch on 115; arm switch on 113 | Each safety control is proven on only one aircraft |
| The watchdog's hold and land in the air | Only exercised on the ground |
| Rangefinder at hover height | It is the only height reference near the floor |
| Magnetometer indoors | Heading drift would be silent |
| Offboard-loss and link-loss behaviour indoors | See section 5 |
| Motor balance under thrust | Only the idle floor and per-motor response were seen |
| Two aircraft armed together | 4 of 6 clean, arm instants 1.7 s apart (median). Radio contention was **tested and ruled out**; the spread is a software design gap |

## 7. Priorities for discussion

1. **Scope.** Pilot-flown indoor hover, or autonomous hold? Autonomous needs a flow sensor or
   on-aircraft vision, plus firmware that supports it.
2. **Who holds the firmware source?** Both aircraft run a vendor fork (PX4 1.16.0, branch
   `Artex`, built 12 Nov 2025). The source is not in this repository and it is not known who
   has it. Without it no sensor driver can be added.
3. **Policy for the first propeller flight.** Tether or net, minimum two people, a named pilot,
   and who signs off props-on.
4. **One failsafe baseline for both aircraft**, chosen for indoors: land in place on every
   loss, and a fence action that does not need a position.
5. **Close the untested items in section 6 on the bench first**, before any props-on test.
6. **Housekeeping that affects trust in the results:** aircraft-side clocks have no time
   source and boot at 1970; 22 commits are not yet on GitHub.

## 8. Proposed path, with an exit test for each stage

| Stage | What | Exit when |
|---|---|---|
| A. Bench, props off | Finish section 6's bench items | Kill on both, arm switch on both, rangefinder lift test, one failsafe baseline |
| B. Tethered, pilot-flown, **one** aircraft | Altitude mode, below 1 m so the rangefinder assists | Kill ready, two people, no uncommanded drift beyond what the pilot corrects |
| C. Extend | Height and area, then the second aircraft separately | Repeatable without intervention |
| D. Autonomous | **Only** with a horizontal-motion sensor | Not reachable on current hardware |
