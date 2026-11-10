# Height from the rangefinder: SITL truth check — 29 Oct 2026

`--instances 4 --only no_takeover --outage "" --seed 3010` on jetson-2, perfect link (as over
USB). The agent's height comes from `oc.HeightSource`: rangefinder when fresh, else the
estimator carried on from the rangefinder's last offset.

| Seed | Verdict | Agent height in hold | True peak |
|---|---|---|---|
| 802856496 | PASS | 0.99 m (max 1.00) | 0.98 m |
| 302458443 | PASS | 0.99 m (max 1.00) | 0.98 m |
| 114482150 | "CRASH" (harness) | 0.99 m (max 1.00) | not recorded |
| 775318234 | "CRASH" (harness) | 1.00 m (max 1.02) | not recorded |

The two "CRASH" verdicts are the **harness's** ground-station link (14550) dropping early:
"Connection to ground station lost" appears in its texts, and no truth or heartbeats were
recorded afterwards. The agent's own logs show a normal flight each time: take-off, hold,
landing, "Disarmed by landing".

Compare: before this change, clean runs with dropouts held at a true 1.08–1.11 m, and the
excursions reached a true 2.0 m (`../2026-10-29-takeover-ladder-truth/`). This is a small sample.
