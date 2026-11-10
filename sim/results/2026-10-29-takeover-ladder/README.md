# Takeover ladder, SITL campaign — 29 Oct 2026

> **Correction, same day:** finding 1 below is wrong. The sim *does* have a rangefinder, and the
> guard read it. Ground truth (`../2026-10-29-takeover-ladder-truth/`) shows the aircraft really
> reached 2 m. The agent's estimator height read up to 1.07 m low, so the guard's landings were
> real, not false alarms. See `../2026-10-29-takeover-ladder-run2/README.md`.

`sim/takeover_tests.py --instances 10 --outage 10,0.5,1.5 --seed 2910` on jetson-2
(started 11:43, finished 12:57). 8 scenarios × 10 runs, each on a freshly booted PX4 SITL
(gz_x500, indoor params, RC over `RC_CHANNELS_OVERRIDE`). The agent sent setpoints at 20 Hz,
with ELRS-like outages injected: bursts of 0.5–1.5 s, about one every 10 s. 1 525 of
18 069 setpoints were dropped (8.4 %).

Files: `ladder.json` (every case), `ladder.log` (one line per case), `logs/<scenario>_<seed>.*`
(agent log, flight trace CSV, SITL log).

## Verdicts

| Stage | PASS | VOID (link loss) | NOT REACHED | FAIL | Other |
|---|---|---|---|---|---|
| T1 prearm (kill) | 10 | | | | |
| T2 arm (kill) | 10 | | | | |
| T3 takeoff (kill) | 1 | | | 8 * | 1 sim didn't boot |
| T4 hold (ch7) | 9 | 1 | | | |
| T5 descent (ch7) | 5 | 2 | 3 | | |
| T6 landing (ch7) | 8 | 1 | 1 | | |
| T7 disarm (kill) | 5 | 2 | 3 | | |
| no takeover | 6 | 1 | | 1 | 2 "false alarm" † |

\* The kill worked every time: disarmed, falling from 0.20–0.33 m. These are marked FAIL only
because the harness kills at true height ≥ 0.2 m, and the agent ends "takeoff" (enters its
0.3 m pause, which it reports as "hold") at the same height. This is a labelling race, not a
takeover failure.

† The guard's 2 m ceiling fired, but the aircraft was not at 2 m. See the findings below.

## What held

- **Every switch flip was recognised and attributed correctly.**
  - 22/22 ch7 flips were classified `pilot_switch`.
  - Every kill was seen as `kill_switch`.
  - In no case was a pilot takeover claimed without a flip.
- **Every lost-link drop was classified as `link_loss`** (7/7, no switch touched), and the
  agent stood down without resuming.
- **ch7 → ALTCTL latency:** median 0.44 s, range 0.13–0.99 s (22 flips).
- **Kill → disarmed:** about 6 s at arm and takeoff. That is PX4's `COM_KILL_DISARM` (5 s); the
  motor cut itself happens at the kill. The harness timed the *disarmed* flag, not motor output.
  The disarm-stage kills (about 2.2 s) race PX4's own landed auto-disarm.
- Every run ended disarmed. Peak true tilt was 20.5°, and no vehicle crashed.

## What did not hold — the ten runs that never reached their stage

1. **Guard ceiling vs agent height use different zeros (5 runs: 2 false alarms, 3 NOT REACHED).**
   - The guard prefers the rangefinder, and falls back to raw `-LOCAL_POSITION_NED.z`
     (`drone_agent.py` Guard.observe).
   - The agent measures from `z0`, captured at the start of the flight.
   - The sim has no rangefinder, so the guard always used EKF z, which had drifted about
     0.9 m from its boot origin by take-off.
   - Example, `landing_868829403`: the agent read 1.11 m when the guard read 2.01 m and landed.
   - On the real aircraft the TFmini covers this above 0.4 m. Below that, and in sim, it doesn't.
2. **Controller ceiling aborts at 1.50–1.69 m (5 runs).**
   - These are take-off overshoots. The spool ramps thrust past hover while the vehicle is still
     on the ground, it leaves at about 0.53 thrust and climbs at about 2.5 m/s.
   - In 4 of the 5, an injected outage then held that stale thrust.
   - `disarm_164416054` overshot with no outage at all. Its first command was 0.8 thrust, with the
     vehicle already at 0.25 m.
3. **One run's simulator never sent a heartbeat** (`takeoff_136600126`). This was a harness
   start-up failure, not a flight event.

## Gate (plan step 2): NOT MET

The gate was: every takeover recognised, zero false alarms, no crash. Recognition passed. The
false-alarm count is 2, both from finding 1, and take-off overshoot keeps T5/T7 from being reached
reliably.
