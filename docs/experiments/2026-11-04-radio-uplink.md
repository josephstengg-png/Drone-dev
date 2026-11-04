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

## Experiment 2: does the ARM itself arrive faster at 3 Hz? (the operator, props OFF)
Profiles `EXP2-A arm stage 20 Hz` and `EXP2-B arm stage 3 Hz` (bench, radio, arm stage, window 15 s). **Run them in this order,
one run each, ten runs:** `B A A A B B B A B A` (random, fixed in advance). Do not edit the profiles. For each run: ch5 on, ch7 centre,
kill off, props OFF, press ARM at READY, and when the agent finishes (or the aircraft arms) disarm with ch6 and wait for the page to say
the run is done. Leave about 30 s between runs. Primary result: seconds from "ARM sent" to "Armed by external command". Then
`python3 tools/experiment_report.py exp2`. Prediction: 3 Hz arms in a few seconds nearly every time; 20 Hz is slower and fails some.
Props off, so every arm is harmless; each run still needs your fresh confirmation in the GUI.
