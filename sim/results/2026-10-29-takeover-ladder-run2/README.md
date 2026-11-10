# Takeover ladder, SITL campaign 2, after fixes — 29 Oct 2026

This campaign used the same setup as campaign 1 (`../2026-10-29-takeover-ladder/`), cut to 5 runs
per scenario (40 runs):

- command: `--instances 5 --outage 10,0.5,1.5 --seed 2910`, on jetson-2;
- each run boots a fresh PX4 SITL (gz_artex) and the agent sends setpoints at 20 Hz;
- 829 of 9 707 setpoints were dropped (8.5 %) to mimic ELRS.

The seed was the same as campaign 1, so these runs replay campaign 1's runs 1–5 case for case,
including the radio dropouts.

The code was commit 47b9574:
- the guard measures height from the agent's zero, and the agent re-zeroes at arming;
- the take-off pause reports as take-off, and the harness kills at 0.1 m;
- the take-off spool freezes at the first sign of lift. This change was **reverted after this
  campaign** (see below).

A third set, `../2026-10-29-takeover-ladder-truth/`, re-flew the no-takeover control 4 times with
Gazebo ground truth, the rangefinder and the estimator all recorded (`*.truth.csv`).

## Verdicts

| Stage | PASS | VOID (link loss) | NOT REACHED | FAIL |
|---|---|---|---|---|
| T1 prearm (kill) | 5 | | | |
| T2 arm (kill) | 5 | | | |
| T3 takeoff (kill) | **5** | | | |
| T4 hold (ch7) | 5 | | | |
| T5 descent (ch7) | 2 | 1 | 2 | |
| T6 landing (ch7) | 4 | | 1 | |
| T7 disarm (kill) | 4 | | 1 | |
| no takeover | 4 | | | 1 "false alarm" |

The total is 34 PASS out of 40, against 27 out of 40 for the same runs in campaign 1.

## What held

- **Take-off: 5 of 5.**
  - Every kill landed in the take-off stage, at 0.10–0.15 m.
  - In campaign 1 these were 8 of 10 labelling failures.
- **Attribution was again 100 %.**
  - 11 of 11 ch7 flips were classified as `pilot_switch`.
  - Every kill was seen as `kill_switch`.
  - The one dropped link was classified as `link_loss`.
  - No run claimed a takeover without a flip.
- **ch7 → ALTCTL took a median 0.42 s** (range 0.19–0.93 s).
- Every run ended disarmed. Peak tilt was 5.9°, and nothing crashed.

## Correction to campaign 1: the guard's landings were real

Campaign 1 called these guard landings false alarms, blaming different height zeros and "no
rangefinder in the sim". **That diagnosis was wrong.**

- The gz_artex model has a downward lidar covering 0.4–12 m, modelled on the TFmini. The guard
  reads it above 0.4 m.
- Ground truth from the truth set:

| Run | True peak | Rangefinder vs truth | Rangefinder minus estimator |
|---|---|---|---|
| 11926496 (pass) | 0.92 m | ≤ 0.03 m | −0.26 to 0.00 m |
| 39083619 (pass) | 1.11 m | ≤ 0.03 m | +0.04 to +0.21 m |
| 513382955 (guard landed) | **2.02 m** | ≤ 0.06 m | **+0.14 to +1.07 m** |
| 621026918 (guard landed) | **2.00 m** | ≤ 0.07 m | **+0.31 to +0.91 m** |

**The aircraft really reached 2 m, and the guard was right.** The agent's height came from PX4's
estimator, `LOCAL_POSITION_NED.z`, and that read as much as 1.07 m low.

Run 621026918 shows the pattern:
- On the floor, as thrust rose, the estimator dipped to −0.22 m.
- It then lagged the lift-off by about 0.4 m (0.50 m estimated at a true 0.93 m).
- The gap widened to 0.9 m by a true 1.97 m.
- After touchdown it still read −0.64 m.

In the airframe, `EKF2_HGT_REF` is 0 (barometer) and `EKF2_RNG_A_HMAX` is 1.0. So range aiding stops
at exactly the 1 m hover height, and above that the estimate is barometer-only.

The controller's ceiling aborts (estimated height over 1.5 m) are the same problem: the true
height was higher still.

The z0 change from campaign 1's "fix 1" is kept, because it is correct for the guard's fallback
below 0.4 m. But it did not address this.

## The other non-passes

- **`landing_241914807`, NOT REACHED: a regression from the spool freeze.**
  - The estimator was still settling at arming: PX4 reported "heading estimate not stable", and
    the height drifted −1.5 m on the floor.
  - A climb-rate glitch read as lift and froze thrust at 0.134.
  - The aircraft never left the floor, and the take-off timeout aborted it to LAND. The outcome
    was safe.
  - Gating the freeze (the sign had to last 0.15 s above half the hover guess) removed its benefit
    in the 1-D model: lift-off climb went from 1.21 to 1.19 m/s.
  - So the freeze was **reverted**. It never addressed the overshoot, which is the estimator
    error above plus stale thrust in dropouts.
- `disarm_522316385` and `no_takeover_277209907`: controller ceiling aborts. Both had dropouts
  (11 and 17 setpoints).
- `descent_368104176` and `descent_207383990`: the guard landed at a rangefinder height of 2.02 m.

## Gate (plan step 2): NOT MET, and the blocker is now the height estimate

Recognition of takeovers is solid across 120 runs. Before the flight stages can pass, the agent
needs a height it can trust. Options:

- the agent flies on the rangefinder when it is valid (≥ 0.4 m), as the guard already does;
- or the estimator fuses it (`EKF2_RNG_A_HMAX` above the flight ceiling). That is a vehicle
  parameter change on the real aircraft.
