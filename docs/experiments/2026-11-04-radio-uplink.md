# Does the agent's setpoint rate choke the ELRS uplink? (4 Nov 2026)

**Theory.** On ARTEX113 over the ELRS radio, the agent's default 20 setpoints/s buries commands (the ARM, the ground-station
heartbeat) on the uplink, so ARM takes 6 s to never, and "No connection to the ground control station" appears. A lower rate fixes it.

**It is falsified if** (a) in Experiment 1 the 20 Hz blocks are not clearly worse than idle, or idle A and idle B differ as much
as idle and 20 Hz (the link just drifts), or (b) in Experiment 2 the 20 Hz and 3 Hz arms are not different.

## Experiment 1: command round trip against setpoint rate (done by Claude, aircraft disarmed)
`/tmp/exp1.py` on jetson-2, result in `fc-backups/2026-11-04/exp1.jsonl`, analysis
`python3 tools/experiment_report.py exp1 fc-backups/2026-11-04/exp1.jsonl`. Rates 0 (twice, as controls A and B), 1, 3, 5, 10, 20 Hz, three
rounds in random order, 6 commands per block, 8 s of silence between blocks. Prediction: round trip and loss rise with rate, 20 Hz is far
worse than idle, 3 Hz is indistinguishable from idle.

**Result (done):** supported. Idle A vs idle B p = 0.125 (no drift); loss 0 % at 1 and 3 Hz, 6 % at 5 Hz, 17 % at 10 Hz, **72 % at 20 Hz**
(median 15 s, all p < 0.0001 against idle). See history.md, 4 Nov night.

## Experiment 2: does the ARM itself arrive faster at 3 Hz? (the operator, props OFF)
Profiles `EXP2-A arm stage 20 Hz` and `EXP2-B arm stage 3 Hz` (bench, radio, arm stage, window 15 s). **Run them in this order,
one run each, ten runs:** `B A A A B B B A B A` (random, fixed in advance). Do not edit the profiles. For each run: ch5 on, ch7 centre,
kill off, props OFF, press ARM at READY, and when the agent finishes (or the aircraft arms) disarm with ch6 and wait for the page to say
the run is done. Leave about 30 s between runs. Primary result: seconds from "ARM sent" to "Armed by external command". Then
`python3 tools/experiment_report.py exp2`. Prediction: 3 Hz arms in a few seconds nearly every time; 20 Hz is slower and fails some.
Props off, so every arm is harmless; each run still needs your fresh confirmation in the GUI.

## Experiment 2 result (done)
3 Hz: 1.8, 2.3, 3.5, 1.6, 2.6 s (5 of 5 armed). 20 Hz: 3.7, 9.6, 7.3, no arm, 7.1 s (4 of 5). Mann-Whitney p = 0.012. Supported. Runs 9 and 10 were swapped
against the plan; limits and next steps in history.md, 4 Nov night.

## Operator observation during Experiment 2 (not instrumented)
After runs 2 and 3 (both A, 20 Hz) the aircraft **tried to arm again almost immediately after the pilot disarmed it in Offboard**. Not seen after
run 1 (B, 3 Hz). The run records cannot show it, because the agent exits within about half a second of the disarm. Consistent with the theory:
the agent re-sends ARM every 3 s until it sees the aircraft armed, so a delayed stream leaves several copies queued in the radio, and the
later ones arrive after the first has armed it and after the pilot's disarm. **A stale ARM re-arming an aircraft after its pilot disarmed it
is a hazard with props on.** Nothing in the agent is changed during the experiment (it would confound it); fixes are listed in history.md after.

## Experiment 3: does Offboard survive an armed hold at 3, 2 and 20 Hz? (the operator, props OFF)
Why: Experiment 2 showed the ARM is quicker at 3 Hz; it does not show PX4 stays in Offboard. At 20 Hz run 142907 (4 Nov) lost Offboard 3.7 s after arming
(classified `link_loss`). PX4's offboard-loss timeout is 1.0 s, so a slow stream might lose it more often, a fast one clogs the uplink.
Profiles (bench, radio, **arm stage, window 60 s**): `EXP3-A ... 3 Hz`, `EXP3-B ... 2 Hz`, `EXP3-C ... 20 Hz`. **Order, 12 runs, random and fixed in advance:
`B A C C A A C B B C A B`.** Per run: ch5 on, ch7 centre, kill off, props OFF; authorise and start as usual; at READY press ARM; then **HANDS OFF for the 60 s window**
(touch nothing unless something looks wrong). The agent ends the run itself after the window and disarms; if it lost Offboard it stands down (`link_loss`) and you
disarm with ch6. Outcome: "survived" (60 s in Offboard) or "bounce" (and when). A run where you used a switch is void. Analysis:
`python3 tools/experiment_report.py exp3`. Prediction: 3 Hz and 2 Hz survive nearly always; 20 Hz loses Offboard in some runs.
The agent now also retries ARM every 8 s over a radio (was 3 s), counts the ARMs it sent, and for 20 s after a disarm answers a stale "Armed by external
command" with DISARM: these runs also show whether anything re-arms.
