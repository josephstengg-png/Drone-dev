# Lean downlink keeps PX4 in Offboard over the ELRS radio (props-off bench evidence)

Date: 5 Nov 2026. Aircraft: ARTEX113 (MicoAir743-AIO, PX4 v1.16.0 `Artex`). Radio: RadioMaster Pocket 2.4 GHz, ExpressLRS **F500, telemetry ratio 1:2 (9921 bps)**.
Written for the operator and the lab manager, who need to decide whether to go props-on over the radio. Everything below was done with **props OFF, on the ground**.

## Result in one paragraph

Over this radio, PX4 kept losing the Offboard signal within a few seconds of arming, at every setpoint rate we tried. The cause was not the setpoint rate but the aircraft's own
telemetry: its default downlink (about 27 messages a second) used about 86 % of the radio's telemetry capacity, and the half-duplex link then delivered our uplink
commands and setpoints late and in bursts. Switching off the streams the agent never reads (the **Indoors** downlink profile) cut command round trips by about 3.5x and kept the setpoint
stream just under PX4's 1.0 s timeout. At 3 Hz, **8 of 9 Indoors armed runs stayed in Offboard for the whole armed window, against 1 of 8 with the default downlink** (Fisher exact p = 0.003).
It is bench evidence only. It does not show that a flight holds.

## What we did, in order

| Step | Finding |
|---|---|
| Experiment 3 (12-slot plan, default downlink, 2/3/20 Hz) | Every armed run lost Offboard or ran out the window; 20 Hz mostly never armed. The 60 s hold was unreachable: PX4 auto-disarms about 10 s after arming with no take-off. |
| PX4 flight logs (USB) | In every armed run the failsafe was `offboard_control_signal_lost`. `gcs_connection_lost` fired in only 2 of 16, both after the Offboard loss. Gaps between setpoints reaching PX4 were 1.35-2.9 s; the loss limit is `COM_OF_LOSS_T` = 1.0 s. |
| Experiment 4 (disarmed, command round trips, 18 per cell) | Switching off all periodic downlink streams: median round trip 0.82 s -> 0.26 s idle, 0.98 s -> 0.24 s at 3 Hz (p < 0.001). At 20 Hz the setpoint stream still choked the uplink (14/18 commands lost, 8/18 with the trim). |
| Built the **Indoors / Outdoors** downlink setting (GUI profile field, `--downlink` on the agent) | Outdoors = PX4 defaults, nothing touched (the default). Indoors switches off 16 unused streams and slows ATTITUDE to 3 Hz; runtime commands only, restored at the end. |
| Experiment 5 (armed hold, props off, 3 Hz) | Results below. |

## Experiment 5 results (3 Hz, props off, armed ~10 s until PX4's auto-disarm)

"Clean" = no `offboard_control_signal_lost` in the armed window (checked in PX4's own log for every run from 119 on).

| Downlink | Clean | Failsafe | Longest gap between setpoints at PX4 |
|---|---|---|---|
| **Indoors** | **8** | **1** (`164515`, one loss at 1.8 s in the log) | 0.51-1.02 s in the clean runs; **1.13 s** in the failsafe run |
| Outdoors (default) | 1 | 7 | 1.35-2.9 s |

Fisher exact: p = 0.003 over all runs. Voided and not counted: `153848` (aircraft refused to arm, High Accelerometer Bias) and `155027` (the offboard switch moved during the run).
Indoors received about 19 messages a second, against about 25-26 with the default downlink (the figure includes the first seconds before the profile took effect).

**Confound, found and checked.** The offboard switch (ch5) read OFF in logs 121-127 (three Indoors runs and all four Outdoors control runs). Within that group the result is the same
(Indoors 3 clean of 3, Outdoors 0 of 4, p = 0.029). With ch5 confirmed ON: Indoors 5 clean of 6 against Outdoors 1 of 4 (p = 0.19; too few runs on their own, but the direction is the same).
The agent had also mistaken "ch5 already off" for a pilot takeover in three runs; that is fixed (see below).

## What this shows, and what it does not

Shows
- The delay on the uplink came largely from downlink traffic competing for the half-duplex radio. Trimming it fixes most of the Offboard losses on the bench.
- The effect is visible inside PX4 (setpoint gaps under the 1.0 s timeout), not only in the agent's own timing.

Does not show
- That a **flight** holds. The aircraft was on the ground with no props and was disarmed by PX4 about 10 s after arming each time. Nothing here tests height control, a longer window or motion.
- That it is safe. One Indoors run in nine still lost Offboard (gap 1.13 s). In flight, a loss makes PX4 fall back to Altitude mode, where it holds height but drifts, while the agent stands down and sends nothing. The pilot would be flying it.
- Anything for outdoor or long-range radio settings. F500 is a short-range mode; range, packet rate and telemetry ratio all change the picture.

## Changes made today (all in the repo, deployed to jetson-2)

- `--downlink {outdoors,indoors}` on `drone_agent.py offboard-flight`; "Downlink" dropdown in the GUI profile editor; `tools/restore_downlink.py` to put the streams back without a reboot.
- `classify_mode_exit` no longer calls a switch that was already off a pilot takeover, and the takeover event records the ch5/ch7 readings. READY now warns (agent log and GUI) if ch5 reads OFF.
- The experiment report and the ulog script (`tools/experiment_report.py`, `tools/ulog_failsafes.py`) count a failsafe in the armed window as a loss.
- Raw data: `fc-backups/2026-11-05/` (ulogs 101-132, `exp4.jsonl`); the run records are in `runs/`.

## Decisions for the operator

1. **Do not treat this as flight-ready.** A props-on radio flight would rely on the 8-in-9 bench figure, with a drift-and-pilot-takeover outcome in the 1-in-9 case.
2. **If radio flight is wanted, the next evidence is a longer armed hold**, which needs PX4's 10 s auto-disarm out of the way (`COM_DISARM_PRFLT`; a saved parameter, so back it up, ask, restore), or a tethered props-on low-thrust test with take-off detection.
3. **Consider also raising `COM_OF_LOSS_T`** (1.0 s) as a second line of defence. That makes PX4 hold stale thrust longer, which is a risk with props on; not recommended before the longer hold test.
4. **Try the radio settings.** A lower telemetry ratio (more uplink share) or a different packet rate might help further; we have not tested them. They are handset settings.
5. **USB remains the safe flight route** until the above is done.
