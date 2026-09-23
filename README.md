# Multi-Agent Flight Planner — SAR Coverage Programme

A search-and-rescue mission-planning stack that turns a **real operational area**
(drawn as KML) into flyable QGroundControl `.plan` files for a **swarm** of
multirotors carrying a **SIYI ZR30** gimbal camera.

**This is the active pipeline for the drone-dev workspace.** The original single-drone
`planning/` pipeline was retired to `legacy/planning/` on 09 Sep 2026; see
`../legacy/README.md` for what carried forward.

It is the consolidation of two earlier efforts: the ground-station **Multi-Agent
Flight Planner** (KML in, multi-drone partition, deterministic geofence guardrail,
optional Gemma operator briefing) and the **drone-dev** coverage-planning
workspace (altitude-driven camera physics, blur-safe cruise speed, and complete
takeoff→lawnmower→RTL missions proven in PX4 SITL). The planner keeps the former's
operational skeleton and the latter's imaging rigour.

Two knobs matter, and only two:

* **Where** you search comes from the KML (`OpsArea` inside a `GeoFence`).
* **How well** you image comes from one number — **flight altitude** — from which
  the ZR30 geometry re-derives the ground swath, the line spacing, the ground
  sample distance (GSD), and the motion-blur-safe cruise speed. No imaging number
  is ever chosen twice.

Everything else (drone count, scan pattern, overlap) is configuration.

---

## Pipeline

```
KML (GCSlocation + GeoFence + OpsArea)
   │  kml_parser.py
   ▼
project to local metres (equirectangular @ home)          geometry.py
   │
   ▼
camera model: altitude → swath, GSD, line spacing,        camera.py   ← from drone-dev
              blur-safe cruise speed, altitude ceiling
   │
   ▼
partition OpsArea for N drones                            scanner_tools.py
   ├─ lawnmower : one contiguous strip per drone
   └─ voronoi   : one cell per drone
   │
   ▼
scan each sub-area, lanes aligned to its long axis        scanner_tools.py
   │
   ▼
GuardrailPlannerAgent: every waypoint AND every leg       agents.py
   inside the original GeoFence — hard-stop on failure
   │
   ▼
build .plan:  DO_CHANGE_SPEED → NAV_TAKEOFF → lawnmower →  qgc_plan.py  ← from drone-dev
              NAV_RTL (frame 2)
   cruise speed derived and actually pushed to the vehicle
   │
   ├─ mission.plan            (combined, tagged comment: "drone-N")
   ├─ mission_drones/drone-N.plan   (one single-vehicle plan each)
   └─ mission.baseline.plan   (home + fence only)
   │
   ▼ (optional, advisory)
OllamaReviewer + RAG → mission.briefing.txt               ollama_client.py / rag.py
```

The LLM sits **after** planning and guardrail. It never chooses a coordinate; if
it fails or times out, the mission is written exactly the same.

---

## Module layout

| Path | Role |
|---|---|
| `src/flight_planner/kml_parser.py` | Pulls `GCSlocation` (home), `GeoFence`, `OpsArea` out of the KML, ignoring folders/styles/wrappers. |
| `src/flight_planner/geometry.py` | Local-metre projection, point-in-polygon, polygon clipping, scanline fill. |
| `src/flight_planner/camera.py` | **Single-knob camera model** — altitude → spacing, GSD, blur-safe speed, ceiling. |
| `src/flight_planner/models.py` | `Coordinate`, `MissionInput`, `PlannerConfig`, `DronePlan`, `PlanningResult`. |
| `src/flight_planner/scanner_tools.py` | Partitioning (lawnmower strips / voronoi cells) + orientation-aware lawnmower scan. |
| `src/flight_planner/agents.py` | `FlightPlannerAgent` (plans) + `GuardrailPlannerAgent` (fence check) + `MultiAgentPlanner`. |
| `src/flight_planner/qgc_plan.py` | Writes QGC `.plan`: takeoff + lawnmower + RTL, per-drone split, embedded geofence. |
| `src/flight_planner/ollama_client.py` / `rag.py` | Optional Gemma field briefing, grounded on `templates/*.md`. |
| `src/flight_planner/cli.py` | Wires the whole run; hard-stops if the guardrail rejects the plan. |
| `tests/` | Unit tests for parser, scanners, plan writer, guardrail, RAG, CLI. |

---

## Parameter reference

Every knob in the chain — camera optics, imaging model, mission config, PX4 vehicle
parameters, plan-format constants, algorithm tolerances and SITL environment — is
catalogued in [`parameters.yaml`](parameters.yaml) with what each one changes and
where it lives. Entries are tagged `active`, `derived`, `constant` or `GAP`, the last
being parameters that *should* be set and currently are not.

`tests/test_parameters_yaml.py` checks the file against the live code, so the
reference cannot silently drift from what the planner actually does.

---

## The single-knob camera model (`camera.py`)

Adapted from drone-dev's `mission_params.py`. Pure geometry, no dependencies.
Defaults are the ZR30 at 1× zoom, 720p.

```
swath width   W  = 2 · h · tan(HFOV/2)
along-track   L  = 2 · h · tan(VFOV/2)
GSD              = W / image_width
line spacing     = W · (1 − side_overlap)          → coverage width for the scanner
speed cap (blur) = GSD / exposure_time             ← almost always the binding cap
speed cap (fwd)  = L · (1 − fwd_overlap) · fps
cruise speed     = min(caps) · 0.5                 (50 % margin)
```

Defaults: HFOV 58.1°, VFOV 35°, 1280×720, 30 fps, 25 % overlaps, 1/1000 s
exposure, 0.5 m target (a person).

At 30 m this yields a 33.3 m swath, 2.60 cm/px GSD, **25.0 m line spacing**, and a
**13.0 m/s** cruise. The `--ceiling` query returns the maximum altitude that still
resolves a target at a requested GSD, so an operator can't unknowingly plan a scan
that can't see a person.

`PlannerConfig.lane_spacing_m` and `PlannerConfig.cruise_speed_m_s` both read from
this model — so altitude, spacing and speed can never drift out of agreement.

---

## Coverage & partitioning (`scanner_tools.py`)

* **lawnmower** — the `OpsArea` is split into one contiguous strip per drone along
  its longer axis; each strip is boustrophedon-scanned.
* **voronoi** — one cell per drone, grown by half-plane clipping around seed points
  spread across the bounding box; seeds are de-duplicated so no two drones inherit
  the same cell.

Either way each drone gets a **spatially separate** sub-area, which keeps the
swarm de-conflicted by geography rather than needing mid-air avoidance.

Scan lanes are laid along **each sub-area's own long axis**, found by testing every
convex-hull edge for the minimum-width direction, so long thin strips are covered
with few long passes instead of many short turn-heavy ones — the coverage-efficiency
behaviour previously provided by Fields2Cover, without the build dependency.

Lanes are **centred within the sub-area** rather than started on its boundary: the
camera swath is wider than the lane pitch, so a centred block still covers both
edges, and no lane degenerates onto the boundary itself where it would scan nothing.

---

## Safety: the guardrail (`agents.py`)

`GuardrailPlannerAgent` projects the original KML `GeoFence` to local metres and
checks that **every waypoint and every leg between waypoints** lies inside it
(point-in-polygon on vertices, segment-in-polygon on the connecting lines). It
runs on every mission regardless of whether the LLM is enabled, and the CLI
**raises and writes no mission** if the check fails. Planning and
safety-validation stay separate modules on purpose.

---

## Plan output (`qgc_plan.py`)

Each mission is a complete, self-contained flight:

* `MAV_CMD_DO_CHANGE_SPEED` (command 178, frame 2) — **before** takeoff,
* `NAV_TAKEOFF` (command 22) at the first waypoint,
* the lawnmower waypoints (command 16, frame 3 = `MAV_FRAME_GLOBAL_RELATIVE_ALT`),
* `NAV_RETURN_TO_LAUNCH` (command 20, **frame 2 = `MAV_FRAME_MISSION`**).

### Why the speed item exists, and why it comes first

A `.plan`'s `cruiseSpeed` field is a **ground-station planning value** — it is used for
time estimates and never reaches the aircraft, which flies `MPC_XY_CRUISE` (5 m/s by
default). Measured on the Pulau Ubin flight: the plan asked for 13.0 m/s and the vehicle
held a median of **5.00 m/s** for the entire survey, so the blur-safe speed — the whole
output of the camera model — was being discarded.

`DO_CHANGE_SPEED` fixes it. PX4 reads the speed from **param2** and ignores param1's
speed type (`navigator_main.cpp`), and it caches and replays the item after a mission
resume, so one item governs the flight. It is placed **before** `NAV_TAKEOFF` because
takeoff flies the vehicle all the way to the first waypoint — with the item after
takeoff, 155 s of a 443 s mission crawled to the survey area at the default speed.
Moving it ahead of takeoff was measured on drone-3 at Pulau Ubin:

| phase | speed item after takeoff | speed item before takeoff |
|---|---|---|
| transit to survey area | median 5.00 m/s | **median 13.00 m/s** |
| survey legs | median 7.70, peak 13.37 m/s | median 7.70, peak 13.29 m/s |
| RTL return | median 5.00 m/s | median 5.00 m/s (PX4 resets it) |
| time to reach the area | 155 s | **79 s** |
| survey complete at | 443 s | **366 s** |

The survey median sits below the commanded 13 m/s because `MPC_ACC_HOR = 2.0` means the
vehicle spends roughly 42 m accelerating and 42 m braking on each 200 m lane; the peak
shows the commanded speed is genuinely reached.

`MPC_XY_VEL_MAX` must also be raised (PX4's 12.0 default would clamp 13.0); it is in
`sar_turn_params.params` at 20.0, mirrored as `models.VEHICLE_MAX_SPEED_M_S`. Above
roughly 46 m the blur-safe speed exceeds that ceiling and the planner emits a warning
that the airframe, not the optics, has become the limit.

`cruiseSpeed` is written from the camera model, not a fixed constant, and the
`planner.imaging` block records the altitude, spacing, GSD and cruise speed the plan
came from, so any `.plan` can be traced back to the one knob that produced it. The
KML `GeoFence` is embedded as an inclusion polygon so QGC shows the boundary. The
combined plan tags each item `comment: "drone-N"`; `split_plan_by_drone` then
emits one ready-to-fly single-vehicle plan per drone.

---

## Configure and run

```bash
cp config.example.json config.json      # then edit
PYTHONPATH=src python3 -m flight_planner.cli "ops/SAFTI city.kml" \
    --config config.json --output mission.plan

# altitude is the knob -- override it without editing the config
PYTHONPATH=src python3 -m flight_planner.cli "ops/SAFTI city.kml" --alt 40

# "how high can I fly and still resolve a person at 3 cm/px?"
PYTHONPATH=src python3 -m flight_planner.cli --ceiling 3.0     # -> 34.6 m
```

```json
{
  "number_of_drones": 3,
  "camera_fov_degrees": 58.1,
  "scan_altitude_m": 30.0,
  "scan_overlap_percent": 25.0,
  "scan_type": "lawnmower",
  "ollama": { "enabled": false, "base_url": "http://localhost:11434",
              "model": "gemma4:26b", "timeout_seconds": 120 }
}
```

`--scan-type lawnmower|voronoi` and `--alt <metres>` override the config without
editing it. Outputs:
`mission.plan`, `mission_drones/drone-N.plan`, `mission.baseline.plan`, and — only
when Ollama is enabled and succeeds — `mission.briefing.txt`.

---

## Running against PX4 SITL

```bash
# 1. plan a sector
PYTHONPATH=src python3 -m flight_planner.cli "ops/pulau-ubin-west.kml" --output missions/ubin/mission.plan

# 2. see it on satellite imagery before you fly it
python3 tools/plot_mission.py ops/pulau-ubin-west.kml missions/ubin/mission.plan missions/ubin/sector.png

# 3. fly one drone's plan and record the track
python3 tools/fly_mission.py missions/ubin/mission_drones/drone-1.plan \
    --params sar_turn_params.params --track missions/ubin/track.json

# 4. compare what was flown against what was planned
python3 tools/plot_track.py ops/pulau-ubin-west.kml missions/ubin/mission_drones/drone-1.plan \
    missions/ubin/track.json missions/ubin/flown.png
```

`fly_mission.py` uploads through MAVSDK's *QGroundControl plan importer*, so it exercises
the real `.plan` file rather than a hand-built item list — if PX4 accepts the upload, the
file is genuinely valid.

### Four things that will bite you (all found the hard way, 08 Sep 2026)

1. **Under Gazebo, `PX4_HOME_LAT/LON` is ignored.** Home comes from the world SDF's
   `<spherical_coordinates>`. If the world origin and the KML sector disagree, every
   waypoint is thousands of metres away and PX4 rejects the mission with
   *"Geofence violation for waypoint 1"*. Copy a world, move its origin to your AO, and
   select it with `PX4_GZ_WORLD=<name>`:

   ```bash
   W=$PX4/Tools/simulation/gz/worlds
   sed -e 's|<latitude_deg>.*|<latitude_deg>1.405367</latitude_deg>|' \
       -e 's|<longitude_deg>.*|<longitude_deg>103.950961</longitude_deg>|' \
       "$W/default.sdf" > "$W/ubin.sdf"
   ```

2. **PX4 SITL persists the mission and geofence** in `build/px4_sitl_default/rootfs/dataman`
   across restarts. A stale fence from a previous session will reject a new mission that is
   perfectly valid. `rm` it when changing sites.

3. **Run the SITL binary with `-d`.** Without it the `pxh>` shell redraws its prompt into a
   non-TTY forever — a background run wrote a 2.7 GB log in a few minutes.

4. **`MIS_YAWMODE` does not exist in PX4 v1.16** — it is `MPC_YAW_MODE`, and the
   yaw-then-translate behaviour is value **4** ("towards waypoint, yaw first"), not 1.
   `sar_turn_params.params` has been corrected; the old line was silently a no-op.

---

## Optional: Gemma field briefing

With `ollama.enabled: true`, `OllamaReviewer` sends a summary of the finished,
guardrail-checked mission (drone count, scan type, altitude, pass/fail, per-drone
waypoint counts) plus the QGC reference retrieved by the offline TF-IDF RAG step
(`rag.py`, grounded on `templates/*.md`) and asks Gemma for a plain-text operator
briefing. A network error, timeout, or empty response is logged as a warning; the
mission files are written identically either way. It is a sanity read before
launch, never a source of truth about coverage.

---

## Testing

```bash
python3 -m pip install -e ".[dev]"
python3 -m pytest
```

Covers KML parsing (incl. missing-geofence rejection), both scanners, guardrail
accept/reject, `.plan` structure (takeoff/RTL/frame), per-drone split, the RAG
retriever, and the Ollama client's fail-safe path.

---

## Developer's Notes

Provenance of this build. The main body above is written as the finished system;
this section records what actually changed and where each part came from.

### Changed from the Google Drive export (`FlightPlanner.zip`)

1. **Import crash fixed — `agents.py`.** `MultiAgentPlanner` used mutable dataclass
   instances as field defaults (`flight_planner: FlightPlannerAgent = FlightPlannerAgent()`),
   which raises `ValueError: mutable default … use default_factory` on Python ≥ 3.11.
   The export ran only on 3.10; it now uses `field(default_factory=…)` and imports
   on 3.12.
2. **Complete missions — `qgc_plan.py`.** The export emitted **only** `NAV_WAYPOINT`
   items (verified: 18/18 command 16 on the SAFTI run) — no takeoff, no
   return-to-launch. Now every plan is `NAV_TAKEOFF → lawnmower → NAV_RTL`, with the
   RTL item on **frame 2** (`MAV_FRAME_MISSION`); frame 3 causes PX4 to reject the
   item.
3. **Cruise speed derived, not hardcoded — `qgc_plan.py` / `models.py`.** The export
   hardcoded `cruiseSpeed: 15` with no link to optics, so a low-altitude scan could
   motion-blur every frame. Speed now comes from the camera model's blur cap — and
   since 09 Sep is pushed to the vehicle as a `DO_CHANGE_SPEED` item, because the
   `.plan` field alone never reaches it (see above).
4. **Orientation-aware lanes — `scanner_tools.py`.** The export always scanned
   east–west (`_lawnmower_points` stepped constant-y lanes) regardless of area
   shape, and its partition axis could sit orthogonal to the scan direction,
   producing many short turn-heavy lanes. Lanes now follow each sub-area's long
   axis, and are centred within it so no lane lands on the boundary. (The lane count
   carries a float tolerance: rotating the polygon perturbs its extent in the last
   bits, and without it an exact multiple of the spacing tips over a `ceil()`
   boundary and adds a spurious edge lane.)
5. **Guardrail checks legs, not just corners — `agents.py`.** The export validated
   only waypoint vertices; a leg between two in-fence corners could still clip
   outside a concave fence. Segment-in-polygon checking was added.
6. **Voronoi seed de-duplication — `scanner_tools.py`.** When too few seeds landed
   inside the polygon the export padded with a duplicate seed, giving two drones a
   degenerate/overlapping cell. Seeds are now de-duplicated before cell growth.

**Kept unchanged from the export:** the KML parser, the equirectangular
projection, the strip/voronoi partition structure, the guardrail hard-stop in the
CLI, per-drone plan splitting, and the Gemma + RAG briefing (advisory, fail-safe).

### Adapted from prior work (`drone-dev`)

1. **`camera.py`** is a port of drone-dev's `mission_params.py` — the swath/GSD/
   spacing/speed derivation and the ZR30 default optics.
2. **The motion-blur and forward-overlap speed caps and the 50 % de-rate**, plus
   the `--ceiling` altitude query, come from the same source.
3. **The takeoff→lawnmower→RTL plan structure and the frame-2 RTL fix** are lifted
   from drone-dev's `swaths_to_plan.py`, where the frame-2 requirement was
   discovered against PX4 SITL.
4. **The ZR30 defaults** (HFOV 58.1°, VFOV 35°, 720p, 30 fps, 25 % overlaps,
   1/1000 s exposure, 0.5 m target) are drone-dev's validated numbers.
5. **The lane-direction optimisation** reproduces what Fields2Cover gave drone-dev,
   carried over as a lightweight rule rather than the heavy dependency.
6. **`sar_turn_params.params`** (yaw-then-translate) ships alongside the plans for
   loading in QGC. Corrected 08 Sep: drone-dev's `MIS_YAWMODE 1` does not exist in
   PX4 v1.16 and was silently a no-op — it is now `MPC_YAW_MODE 4` ("towards waypoint,
   yaw first"). `NAV_ACC_RAD 2.0`, `MPC_ACC_HOR 2.0` and `MPC_TILTMAX_AIR 25` carried
   over unchanged, and `MPC_XY_VEL_MAX 20.0` was added so the derived cruise speed is
   not clamped by PX4's 12.0 default.

### Known limitations / next

* Projection is equirectangular; drone-dev's AEQD is marginally more accurate but
  the difference is negligible at SAR scale. Left as-is.
* Multi-drone de-confliction is geographic only — no altitude layering or temporal
  separation yet.
* **The partition still splits the OpsArea's *long* axis** (kept from the export).
  Now that lanes are orientation-aware, this is the remaining efficiency lever: on
  the SAFTI area (305 x 721 m, 3 drones) splitting the long axis gives 30 lanes over
  7.57 km, while splitting the *short* axis gives 15 lanes over 8.45 km — half the
  turns for ~12% more scan length. Worth revisiting once turn cost is measured in
  SITL.
* `camera.py` models one gimbal (nadir ZR30); zoom/oblique framing is out of scope.
* **Done 08-09 Sep 2026.** Flown end-to-end in Gazebo over Pulau Ubin: hand-drawn KML
  through the planner, uploaded to PX4 via MAVSDK's QGroundControl plan importer, flown
  takeoff → lawnmower → RTL → landing. Survey cross-track **max 4.95 m, mean 0.46 m**
  over 41,170 samples; altitude held 29.95-30.04 m against a commanded 30.
* **RTL always returns at `MPC_XY_CRUISE`.** PX4 calls `reset_cruising_speed()` when
  entering RTL, so the mission's speed item does not apply to the return leg.
* Every tunable is catalogued in `parameters.yaml`, and `tests/test_parameters_yaml.py`
  fails if it drifts from the code.
  `sar_turn_params.params` sits in the repo root for loading in QGC alongside the
  generated plans.

---

# Addendum — guardrail and droneCount fixes (09 Sep 2026)

Two defects recorded in `parameters.yaml` under `gaps:` are now closed. This section
records how, because both changes alter what the planner will *reject*, and that is the
kind of thing that looks like a regression if you meet it without context.

## 1. The guardrail now checks the legs the planner never chose

**What was wrong.** `GuardrailPlannerAgent.validate` walked `zip(local, local[1:])` over
survey waypoints. But every plan is `DO_CHANGE_SPEED → NAV_TAKEOFF → lawnmower → RTL`, so
the vehicle also flies **home → waypoint 0** and **last waypoint → home**. Neither was
validated. On the Pulau Ubin sortie those transits were 269, 585 and 664 m — for drone-3,
roughly a third of its total flight was unchecked. They stayed inside the fence by where
the GCS happened to sit, not because anything verified it.

**How it was fixed.** The leg chain is conceptually `[home] + survey + [home]`:

```python
home = project(mission.gcs_location, mission.gcs_location)
...
if local and home_inside:
    if not segment_in_polygon(home, local[0], fence):
        violations.append(f"drone {id} takeoff transit (home -> waypoint 0) leaves GeoFence")
    if not segment_in_polygon(local[-1], home, fence):
        violations.append(f"drone {id} RTL return (waypoint {n-1} -> home) leaves GeoFence")
```

Three decisions worth knowing about:

- **Home is checked once, not per drone.** Every aircraft launches from and returns to the
  same point, so a fence excluding it fails the whole sortie. Reporting that as `2N` leg
  failures would bury the actual cause, so it surfaces as a single
  `GCS location is outside the GeoFence`, and the per-drone transit checks are skipped
  when it fires.
- **Waypoint indices are unchanged.** Home is not a waypoint the planner chose, so it is
  not added to the containment loop — existing messages like `waypoint 7 is outside
  GeoFence` still mean the same thing.
- **Proper-intersection semantics are unchanged.** A transit that merely grazes the fence
  at a vertex is not a breach. This matters more now than before: a GCS placed on the
  boundary would otherwise fail every mission. (An earlier fixture did exactly that, which
  is how the case was found.)

**What this changes for you.** Missions that previously passed can now be rejected — a
staging point outside the AO's lobe on a concave fence will hard-stop. That is correct
behaviour, not a regression. Both real sectors still pass with transits checked, and
there are regression tests pinning that.

## 2. `droneCount` reports plans that exist

**What was wrong.** `build_qgc_plan` wrote `droneCount: len(result.drones)` while
`_mission_items` skips drones whose cell came out empty. A plan could advertise five
drones while `mission_drones/` held three files.

**How it was fixed.** Rather than swapping in the smaller number and losing the
information, both are reported:

```python
"x-multiDrone": {
    "droneCount": len([d for d in result.drones if d.waypoints]),
    "requestedDroneCount": len(result.drones),
    "assignment": "comment field on each mission item",
},
```

`droneCount` now means what every consumer already assumed — the number of
`drone-N.plan` files `split_plan_by_drone` will produce — and `requestedDroneCount`
keeps the discrepancy visible. `_missing_waypoint_warnings` already names which drones
were starved, and those warnings ride along in `planner.warnings`, so the plan carries the
full story rather than a quietly rounded number.

**Finding a case that actually starves a drone took some doing**, which is worth
recording: neither real sector starves at any drone count from 3 to 16, in either scan
mode, and nor do small synthetic squares down to ~55 m. The earlier Voronoi seed
de-duplication and lane-centring fixes made it genuinely hard to reach. The reproducible
trigger is a **degenerate, zero-area OpsArea** (three collinear points) with 8 drones,
which yields 5 flying under `lawnmower` and 7 under `voronoi`. That case is now a test, so
the count logic is exercised against real starvation rather than a hand-built fixture
alone.

## Verification

- **90 tests pass** (was 81). Eleven guardrail tests, including the takeoff transit, the
  RTL return, home-outside-fence reported once, boundary-grazing accepted, and both real
  sectors passing with transits checked.
- Regenerating the Pulau Ubin mission gives
  `{"droneCount": 3, "requestedDroneCount": 3}`, guardrail passed, 63 items — 3 drones ×
  (speed + takeoff + 18 waypoints + RTL).

---

# Addendum — altitude layering, turn shaping and anti-centripetal yaw (11–14 Sep 2026)

Three more changes landed after the guardrail/droneCount addendum above, all driven by
flying the real plans in SITL rather than by inspection. Full numbers for every sortie are
in `../history.md`; this section is the "what changed and why", kept next to the guardrail
addendum it follows on from.

## 1. Altitude layering (11 Sep) — `PlannerConfig.for_drone`, `agents.py`

Each drone in a swarm now gets its own altitude layer (default 2 m spacing: 28 / 30 / 32 m
for drones 1–3), not one shared altitude. `PlannerConfig.drone_altitude_m(index)` derives
the layer and `for_drone(index)` rebuilds the whole imaging profile (spacing, GSD, cruise
speed) for that altitude, so a staggered drone's camera geometry is still correct rather
than reusing drone-1's numbers at a different height. The guardrail
(`_separation_violations`) hard-stops a plan if two drones' paths come within
`MIN_HORIZONTAL_SEPARATION_M` (20 m) while on layers closer than
`MIN_VERTICAL_SEPARATION_M` (2 m) — closing the collision risk found by inspecting the
Toa Payoh voronoi overlap (drones' cells touch; without layering their flight paths could
coincide there in both space and altitude).

**Known gap, confirmed flown, not fixed:** `RTL_RETURN_ALT` is 30 m in this SITL and PX4
never returns *below* the vehicle's current altitude, so a 28 m-layer drone climbs onto the
30 m drone's layer to come home — the layering collapses on the return leg. Fix design is in
`history.md`'s parked to-do list (a per-vehicle `RTL_RETURN_ALT`, or explicit RTL waypoints).

## 2. Turn shaping — corners drawn into the `.plan` (14 Sep), `geometry.py`, `qgc_plan.py`

Flown 11 Sep, sharp lawnmower corners put the vehicle's bank past 15° on 36 of 37 turns
(`MPC_YAW_MODE 3`, "along trajectory", banks and yaws together through a corner rather than
stopping to yaw first). The fix moved from *params only* to also *drawing* the turn:

- `PlannerConfig.turn_radius_m` (0 = automatic: half the drone's own lane spacing) and
  `turn_style` (`"circle"`, the default) select a **circular turn drawn as the polygon
  around the circle** — a vertex every `arc_step_deg` (45°), each carrying an acceptance
  radius of half its leg (`geometry.py :: rounded_turns / _circumscribed_turn`). PX4 fillets
  each vertex to exactly the turn radius, so consecutive fillets meet with 0.000 m gap and
  the vehicle should, in principle, fly one continuous circle.
- `MIN_TURN_RADIUS_M` (3 m) and a cut-depth cap (`TURN_CUT_FOOTPRINT_FRACTION`, tied to the
  camera's along-track footprint) stop a corner from being drawn so tight the camera would
  miss it, or so tight PX4 would crawl round a near-zero radius.
- **Flown result (Toa Payoh drone-3, same params as 11 Sep):** survey time back to the sharp
  corners' 481 s (486.5 s), U-turns almost twice as fast as an earlier (now superseded)
  pointwise-arc attempt, and the tightest path-tracking of any version (p95 0.34 m). **Not
  yet a steady aircraft-like bank** — PX4 v1.16 re-plans a straight line to each new
  waypoint on every switch and uses the fillet only to cap corner speed, so the bank arrives
  as one ~16° pulse per vertex (4 per U-turn) rather than one held bank. A truly steady bank
  needs a continuous trajectory (offboard setpoints or PX4's orbit mode), not waypoints —
  tracked as an open item.

## 3. Anti-centripetal yaw — `turn_style: "pre_yaw"` (14 Sep, evaluated, not the default)

An alternative tactic: keep corners sharp, but have every waypoint carry the compass
heading of the leg *leaving* it (`param4`), so PX4 (which will not mark a heading-carrying
waypoint "reached" until the nose is within `MIS_YAW_ERR`, 12°) rotates the nose onto the
next leg before the corner rather than at it. A pass-through waypoint `pre_yaw_lead_s`
(default 3 s of flight) before each corner keeps the nose on the current lane until then.

**Flown and measured against all four earlier Toa Payoh runs — does not reduce the bank.**
A multirotor turns by tilting toward the new direction, not by yawing the nose (unlike a
fixed wing); pre-rotating the nose only moves the deceleration from pitch into roll. Total
tilt (roll + pitch combined) matched the sharp-corner flight almost exactly (95th pct 19.5°
vs 19.3°), while peak roll (22.2°) and peak yaw rate (152°/s, ~20° of overshoot per
rotation) were the worst of any version tried. **`turn_style` stays `"circle"`** — `pre_yaw`
is kept as a config option (`turn_style: "pre_yaw"` in a config JSON; no CLI flag) for cases
where controlling *where the nose points* — e.g. a forward-looking sensor cued onto the next
lane early — matters more than bank angle, and only with the gimbal in heading-lock mode
(a follow-mode gimbal would swing the camera with every pre-rotation).
