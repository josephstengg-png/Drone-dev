# Ground-station GUI: design brief (draft 1)

Status: draft for the operator to steer. Nothing here is built. Everything is derived from
how the existing tools were actually used during indoor-demo preparation (see
`docs/history.md`, 14-15 Oct 2026).

## 1. What the GUI is for

One operator, one or two aircraft, props-off bench work now and tethered flight later.
The GUI replaces typing a sequence of commands from memory with a fixed sequence of
**gates**, each of which answers one question and leaves evidence behind.

It is a **front end over the existing tools**. It does not reimplement them. The tested
logic in `tools/safety_rules.py`, `preflight.py`, `drone_agent.py` and friends stays the
single authority; the GUI calls them and shows what they report.

## 2. Principles

1. **A gate, not a script, is the unit of work.** The operator sees "Link", "Preflight",
   "Takeover map", not file names.
2. **Every gate shows GO / NO-GO / NOT RUN with its evidence** (the number, the text, the
   time). A green tick with nothing behind it is not allowed.
3. **Props-off confirmation is fresh per run and cannot persist.** It is a deliberate
   action each time, tied to one aircraft and one run, and it expires when the run ends.
4. **Abort is always visible and never depends on the GUI's own connection.** It calls the
   out-of-band `takeover.py` path.
5. **Anything that changes a parameter shows its restore state** and refuses to be closed
   while a restore is unverified.
6. **Say what normal state would produce a symptom** before showing red. A powered-down
   Jetson or a GPS with no fix indoors is not a fault.
7. **Nothing is hidden behind "OK".** Counters are shown as rates (sampled twice and
   differenced), never as lifetime totals.

## 3. Screens

### 3.1 Fleet overview (home)
One card per aircraft (ARTEX113, ARTEX115): link state, loss % and round trip, armed
state, mode, estimator state (GNSS-aided / no-GPS), and the worst gate currently failing.
A single fleet-wide ABORT.

### 3.2 Aircraft detail: the gate list
Ordered, top to bottom. Each row: state pill, one-line evidence, "run" and "details".

| Gate | Question | Feeds from | Evidence shown |
|---|---|---|---|
| Ground LAN | Is this machine on the drone network? | `bootstrap_ground_station.sh --check` | address, interface |
| Companion | Is the Jetson flight-ready? | `provision.py --verify` | missing or stale files |
| Radio chain | Right backpack, one relay, link healthy? | `elrs_pair.py`, `preflight_probe.py` | BSSID, loss %, RTT, relay count |
| Preflight | GO or NO-GO for the chain | `preflight.py` | per-check list, verdict |
| Takeover map | Can the pilot take it back? | `rc_takeover.py` | channel 7 slots, any Offboard slot flagged |
| Arm cycle | Does arm, hold, disarm work? | `drone_agent.py` / `pair_arm_test.py` | arm time, spread, disarm source |
| No-GPS state | What does the estimator do indoors? | `no_gps_check.py` | estimator flags, restore status |
| Motors | Does each motor answer on its own channel? | `motor_test.py` | per-motor response |
| Parameters | Do 113 and 115 match the chosen baseline? | `mav_shell.py` param reads | diff table |

### 3.3 Live panel (always on during any run)
Modelled on the `guardwatch` observer used for takeover and disarm tests:
channel 6 (arm), channel 7 (mode), channel 8 (kill) position; flight mode; armed state;
last PX4 message; guard state; and a **STILL ARMED** banner that mirrors the agent's
5-second warning. A timeline strip records events with timestamps.

### 3.4 Run wizard
A guided path through a single test (for example "disarm after takeover"): states what
will happen in plain language, asks for the fresh props-off confirmation, runs it, shows
the outcome and writes the history entry. This is the layman walkthrough used so far.

### 3.5 Evidence log
Reads and appends to `docs/history.md` (newest first). Every run produces a block with
date, aircraft, gate, result and raw evidence, so the log stays the record of truth.

### 3.6 Parameter safety
Shows the saved original of any parameter a tool has changed, whether the restore was
verified by read-back, and blocks exit while unverified. Mirrors `no_gps_check.py`
(`/tmp/gps_ctrl_orig_<sysid>.txt`, 4 verified retries).

## 4. States and colour semantics

| State | Meaning | Operator action |
|---|---|---|
| GO | Check ran, evidence within limit | none |
| NO-GO | Check ran, evidence outside limit | do not proceed |
| NOT RUN | No evidence yet | run it |
| STALE | Evidence older than the session or the last parameter change | re-run |
| RUNNING | Tool in progress | wait or abort |
| UNKNOWN | Tool could not reach the aircraft | check the link, not the aircraft |

Armed state and the kill/abort controls use a separate, louder treatment from gate
state, so "armed" can never be confused with "passed".

## 5. Backend contract

- GUI calls each tool as a subprocess and reads **exit code plus JSON output**. Tools
  that lack a JSON mode get one before the GUI depends on them (open item).
- One long-running status service owns the MAVLink observation (what `guardwatch.py`
  does today) so several screens do not open competing connections. This also avoids the
  suspected 15-20 s "No connection to the ground control station" gap caused by extra
  short-lived connections.
- Nothing in the GUI arms an aircraft without the run wizard's fresh confirmation.
- The abort path shells out to `takeover.py --now` over ssh, independent of the status
  service.

## 6. Out of scope for the first version

- Mission planning and map views (the outdoor pipeline: `flight-planner`,
  `plot_mission`, `swarm_*`). These come later.
- Any control of the aircraft in flight. The computer cannot fly (offboard stream is
  level attitude at zero thrust); the pilot flies.
- Remote or multi-user access.

## 7. Open questions for the operator

1. **Form factor:** a desktop window on the laptop, or a local web page? (Web is easier
   to view from a second device on the drone LAN.)
2. **How much guidance:** expert dashboard, or the layman walkthrough as the default with
   the dashboard behind it?
3. **Day-of-demo view:** should one screen collapse the gate list to a single GO/NO-GO
   per aircraft for an audience?
4. **Who may press what:** is there ever a second person (spotter or lab manager) who
   should only see state and never trigger a run?
5. **Visual direction:** dense instrument-panel look, or calm and spacious? Light, dark,
   or both (a hangar and a lab are very different lighting)?
6. **Naming:** keep ARTEX113 / ARTEX115, or friendlier labels on screen?

## 8. Suggested build order

1. Status service plus the live panel (read-only, no risk).
2. Gate list with the read-only gates (link, preflight, takeover map, parameters).
3. Abort control.
4. Run wizard for arm cycle, then no-GPS and motor tests (the ones that change state).
5. Evidence log writer.
