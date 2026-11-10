# T3 run 1 fixes, SITL — 29 Oct 2026

Each fix flown on its own: 2 runs each, `--only no_takeover --outage "" --seed 3110`, with
agent args `--height 0.3` plus the fix's switch. Perfect link (as over USB). Truth recorded:
height, rangefinder, estimator and horizontal drift.

| Fix | Seed | True peak | Max climb | Accel up / braking | Drift | Lift-off after arm |
|---|---|---|---|---|---|---|
| 1 `--lift-by-climb` | 1029224485 | 0.31 m | 0.30 m/s | 0.6 / −1.5 m/s² | **0.79 m** | 4.6 s |
| 1 | 194827946 | 0.39 m | 0.32 m/s | 0.8 / −0.7 m/s² | 0.08 m | 4.7 s |
| 2 `--spool-fast-frac 0.9` | 1029224485 | 0.30 m | 0.49 m/s | 1.1 / −2.2 m/s² | 0.06 m | **3.8 s** |
| 2 | 194827946 | (harness lost truth; agent peak 0.31) | | | | |
| 3 `--ground-attitude` | 1029224485 | 0.39 m | 0.56 m/s | 1.1 / −2.6 m/s² | 0.04 m | 4.7 s |
| 3 | 194827946 | (harness lost truth; agent peak 0.31) | | | | |

All 6 flights passed: profile flown, landed, disarmed by PX4. For comparison, the gentle
take-off without fixes on the same seeds gave a true peak of 0.68 m and 0.29 m.

Caveats:
- **The floor dip that caused the 0.68 m run did not recur** in any of these six boots (raw
  estimator dip 0.00 to −0.09 m). So fix 1's main job, seeing lift-off when the estimate has
  fallen, is not demonstrated here. The real ARTEX113 log (−0.27 m) and the unit tests cover it.
- The sim floor is flat and has no cable, so fix 3 can only show it does no harm. It didn't.
- Two runs lost the harness's own GCS link, so no truth was recorded; the agent's flights were normal.

New findings:
- **Holding at 0.3 m is in the rangefinder's blind zone.** In fix 1 run 1 the agent held "0.30 m"
  while the true height sagged to 0.10 m, then recovered. Below 0.4 m its height is the estimator
  again. A hold at 0.5 m or more is on the rangefinder.
- **Drift without position hold, even on a flat floor:** 0.79 m in one run, a steady 0.04 m/s over
  17 s from a small sideways speed at lift-off. The other runs drifted 0.04–0.08 m.
