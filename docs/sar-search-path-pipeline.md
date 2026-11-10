# SAR Search-Path Pipeline

How the mission-planning modules chain together to turn a real map and a flight altitude
into a Gazebo-flyable drone search pattern. Canonical build-log + architecture record.

## The core idea

Two inputs, and only two.

- **Where** — a KML drawn over real terrain: a `GeoFence` the aircraft must not leave and
  an `OpsArea` it must cover.
- **How well** — ONE knob, flight altitude. Every downstream imaging number re-derives
  from the ZR30 camera geometry. No value is chosen twice.

```
ops/*.kml (GCSlocation + GeoFence + OpsArea)
  -> kml_parser.py       (KML -> home, fence, area)
  -> geometry.py         (lat/lon -> local metres)
  -> camera.py           (ALTITUDE -> swath, GSD, lane spacing 25 m, cruise 13 m/s)
  -> scanner_tools.py    (partition for N drones; lanes along each sub-area's long axis)
  -> agents.py           (guardrail: every waypoint AND leg inside the fence, hard-stop)
  -> qgc_plan.py         (DO_CHANGE_SPEED -> takeoff -> lawnmower -> RTL, frame 2)
  -> mission.plan + mission_drones/drone-N.plan
  -> PX4 SITL + QGC, or tools/fly_mission.py
  -> tools/plot_track.py (flown vs planned)
```

Everything lives in `~/drone-dev/FlightPlanner/`. The original single-drone pipeline was
retired to `legacy/planning/` on 09 Sep 2026 — see `legacy/README.md` for what carried
forward and what did not.

## The modules

1. **`kml_parser.py`** — pulls `GCSlocation` (a Point), `GeoFence` and `OpsArea`
   (Polygons) out of the KML, ignoring folders, styles and wrappers.

2. **`geometry.py`** — local equirectangular projection about home, point-in-polygon,
   polygon clipping, scanline fill, convex hull, minimum-width direction, and
   segment-in-polygon for the guardrail.

3. **`camera.py`** — ZR30 optics (HFOV 58.1°, VFOV 35°, 1280×720, 30 fps) + altitude ->
   swath width, GSD, line spacing (25% side overlap), motion-blur speed cap,
   pixels-on-target. At 30 m: **swath 33.3 m, GSD 2.60 cm/px, spacing 25.0 m,
   cruise 13.0 m/s**. Ported from `legacy/planning/mission_params.py`; the numbers are
   asserted in `tests/test_camera.py`. `--ceiling 3.0` inverts it -> 34.6 m.

4. **`scanner_tools.py`** — partitions the OpsArea (equal-width strips, or one Voronoi
   cell per drone grown by half-plane clipping around de-duplicated seeds), then
   boustrophedon-fills each sub-area with lanes along **its own long axis**, found by
   testing every convex-hull edge for the minimum-width direction. This reproduces the
   swath-count objective Fields2Cover optimised for, without the C++ dependency.
   Lanes are centred within each sub-area so none degenerates onto the boundary.

5. **`agents.py`** — `FlightPlannerAgent` plans, `GuardrailPlannerAgent` validates every
   waypoint and every leg between waypoints against the original fence, and the CLI
   raises and writes nothing if the check fails.

6. **`qgc_plan.py`** — writes the `.plan`. Per drone: `DO_CHANGE_SPEED` (178, speed in
   param2, before takeoff) -> `NAV_TAKEOFF` (22) -> the lawnmower (16, frame 3) ->
   `NAV_RETURN_TO_LAUNCH` (20, **frame 2**). Every item tagged `comment: "drone-N"`;
   `split_plan_by_drone` cuts on that tag to emit one flyable file per drone.

7. **`tools/`** — `plot_mission.py` (sector over satellite imagery), `fly_mission.py`
   (upload via MAVSDK's QGC plan importer, arm, fly, record the track),
   `plot_track.py` (flown vs planned with cross-track error).

Every tunable in the chain is catalogued in `FlightPlanner/parameters.yaml`, and
`tests/test_parameters_yaml.py` fails if that file drifts from the code.

## Run it

```bash
source ~/drone-dev/venv/bin/activate && cd ~/drone-dev/FlightPlanner

flight-planner "ops/pulau-ubin-west.kml" --output missions/ubin/mission.plan
python tools/plot_mission.py ops/pulau-ubin-west.kml missions/ubin/mission.plan sector.png

# SITL, headless, daemon mode, world origin matching the AO
cd ~/drone-dev/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=ubin GZ_IP=127.0.0.1 HEADLESS=1 \
  ../../../../bin/px4 -d < /dev/null > sitl.log 2>&1 &

# then either QGC, or:
python tools/fly_mission.py missions/ubin/mission_drones/drone-1.plan \
    --params ../sar_turn_params.params --track missions/ubin/track.json
```

## Flight tuning — turn behavior

x500 is a MULTIROTOR (no stall/runaway — that's fixed-wing) and the ZR30 is a 3-axis
gimbal (holds nadir through airframe roll). Since 11 Sep 2026 corners are flown
**banked, yawing and rolling together** — about 15° of bank — instead of stopping to
yaw first. Yaw-first cost ~8 s a corner (measured on three sorties); the gimbal, not
the airframe heading, keeps the camera on target.

`sar_turn_params.params` — load in QGC: Vehicle Setup -> Parameters -> Tools -> Load
from file:

- `MPC_YAW_MODE = 3` — "along trajectory": the heading follows the path, so the yaw
  happens while the vehicle banks. (4, "yaw first", brakes to a standstill at every
  corner and was used until 11 Sep 2026.)
- `MPC_ACC_HOR = 2.6` — ≈15° of bank in a corner (9.81 · tan 15°). `MPC_TILTMAX_AIR = 25`
  stays the absolute ceiling.
- `NAV_ACC_RAD = 5.0` — how far out a corner may start. PX4 rounds a corner at
  √(accel · radius · tan(angle/2)): 3.6 m/s through a 90° lane corner, cutting 2.1 m
  inside it — the camera's 9.5 m half-footprint still covers the lane end.
- `MPC_YAWRAUTO_MAX = 90` — lets the heading keep up with the bank (66°/s is needed at a
  tight 2 m corner; PX4's default is 45).
- `MPC_XY_VEL_MAX = 20.0` — ceiling only; the actual cruise arrives as a mission item.

Turn *behaviour* for a multirotor is set by PARAMS. Since 14 Sep 2026 the turn *shape* is
drawn in the `.plan`. Flown as two sharp corners, the 11 Sep sortie braked to ~2.4 m/s and
snapped into 17–20° of bank. The first drawn version (points *on* an arc of half the lane
spacing, 22.5° apart, 2 m acceptance radius), flown 14 Sep, kept the bank under 15° (0.6 s
above it, against 24.6 s) and halved the mean cross-track (0.24 → 0.13 m) — but PX4's
corner rounding at each point was too short to reach the next, so the roll swung ±11–14°
once per waypoint and the arcs were flown at ~2.7 m/s; the survey took 22 % longer.

Current drawing (evening of 14 Sep): every corner is a circle of `turn_radius_m` (0 = half
the lane spacing, the widest turn that fits between neighbouring lanes), drawn as the
polygon *around* the circle — a vertex every 45°, each with an acceptance radius of half its
leg (param2, which PX4 honours per waypoint for multicopters). PX4 rounds each vertex with a
fillet of radius d / tan(bend/2), which here is exactly the turn radius, and each fillet
starts where the last one ended — one continuous circle, on paper.

Flown the same evening (Toa Payoh drone-3, SITL): each U-turn took 7.0 s at 3.4–6.1 m/s
(arcs: 13.5 s), the survey matched the sharp corners' time (486.5 s against 480.6 s), and
the roll no longer swings side to side (9 flips per turn → 2). But the bank still arrives
as one pulse per vertex — four ~16° pulses per U-turn, back to 0° between, pulses 2.24 s
apart against waypoint switches 2.34 s apart, with the yaw rate briefly reversing
(−19°/s). PX4 v1.16 re-plans a straight line to each new waypoint and uses the fillet only
to cap the corner speed, so waypoint geometry alone cannot make it fly a circle. A steady,
aircraft-like bank needs a continuous trajectory — e.g. the companion computer streaming
setpoints along the arc in offboard mode. The old "never curve the `.plan`" rule
dates from yaw-first flying, when every extra waypoint was another full stop. Corners
beside an excluded notch stay sharp with a tight 2 m radius, so the turn cannot cut
towards the exclusion.

Imaging through the arc (checked 14 Sep against the ZR30 spec): the gimbal's ±45° roll
range covers the 15° bank several times over; blur is ~0.2 px from speed and ~0.35 px
from yaw at the image corners on an ideal arc (the 11 Sep 142°/s yaw spikes smeared
~1.8 px — and the 14 Sep arc flight still peaked at 142°/s, from the per-waypoint
corrections); the arc
passes 4.8–5.5 m inside each lane-end corner, within the camera's 8.8–10.1 m along-track
half-footprint, so lane ends stay covered.

## Version notes / gotchas

- **`MIS_YAWMODE` does not exist in PX4 v1.16.** It appears nowhere in the tree. It is
  `MPC_YAW_MODE`, and the numeral does not carry over: old `MIS_YAWMODE=1` ("towards
  next waypoint") maps to `MPC_YAW_MODE=0`; 4 is yaw-first (flown until 11 Sep 2026) and
  3, along-trajectory, is current. Writing 1 gives "towards home" — nose pinned at the
  launch point all survey.
  Found 08 Sep 2026 when a MAVSDK `set_param` timed out with no ack.
- **A `.plan`'s `cruiseSpeed` never reaches the vehicle.** It is a ground-station
  planning field. Measured 09 Sep: the plan asked for 13.0 m/s and the aircraft held a
  median 5.00 m/s — PX4's `MPC_XY_CRUISE` default — for the entire survey. Fixed by
  emitting `MAV_CMD_DO_CHANGE_SPEED` (178) with the speed in **param2**, placed **before**
  `NAV_TAKEOFF` (takeoff flies all the way to the first waypoint, so a later item leaves
  that transit at the default). Verified on drone-3 at Ubin: transit speed 5.00 -> 13.00
  m/s median, time to reach the search area 155 s -> 79 s, survey complete 443 s -> 366 s.
  `MPC_XY_VEL_MAX` raised from PX4's 12.0 to 20.0 so 13.0 is not clamped. PX4 caches and replays the speed item across a mission resume.
  RTL still resets to `MPC_XY_CRUISE` — PX4 calls `reset_cruising_speed()` on RTL.
- **Under Gazebo, `PX4_HOME_LAT`/`PX4_HOME_LON` are ignored.** Home comes from the world
  SDF's `<spherical_coordinates>`. `default.sdf` here is set to **Tuas**; `ubin.sdf` was
  added for Pulau Ubin. A mismatch gives *"Geofence violation for waypoint 1"*.
- **PX4 SITL persists mission + geofence** in `build/px4_sitl_default/rootfs/dataman`.
  Delete it when changing sites or a stale fence rejects a valid mission.
- **Run the px4 binary with `-d`.** Without it the `pxh>` prompt redraws into a non-TTY
  forever — a backgrounded run wrote 2.7 GB in minutes.
- **The simulated battery does not recharge.** After a few long flights arming is denied
  with *"Preflight Fail: Battery unhealthy"*. Restart SITL.
- **NEVER run `make distclean`** in PX4 — de-inits ALL submodules. Use
  `rm -rf build/px4_sitl_default`.
- **RTL must be frame 2** (`MAV_FRAME_MISSION`); frame 3 makes PX4 reject the whole
  mission ("Invalid item").
- **QGC 4.4.3 AppImage (Qt5)** on 22.04; the master 5.x build needs 24.04+. It retains a
  stale plan and stale error banners across reconnects — restart it when changing sites,
  and kill the real binary, not just the AppRun wrapper.
- **Protobuf conflict** (only if rebuilding Fields2Cover, now legacy-only). F2C's
  OR-Tools installed protobuf 25.3 + abseil into `/usr/local`, shadowing system protobuf
  3.12.4 -> PX4 gz build failed. Fix = stash the /usr/local protobuf to /opt/pb-stash
  (bin/protoc*, libprotobuf*.a, libabsl_*.a, include/google/protobuf, include/absl,
  lib/cmake/{absl,protobuf,utf8_range}, lib/pkgconfig/*.pc), leave `libortools.so.9` in
  place, `ldconfig`, rebuild. Revert before rebuilding F2C.
- PX4 build needs Python deps in the ACTIVE venv:
  `pip install -r ~/drone-dev/PX4-Autopilot/Tools/setup/requirements.txt`.
- The ROS 2 Humble overlay puts `launch_testing`/`launch_ros` pytest plugins on the path;
  they import `lark`, which the venv lacks, and crash pytest before collection.
  `FlightPlanner/pyproject.toml` blocks both by name.

## Open items / next

- **Swarm behaviour** is offline spatial partitioning only — N independent missions that
  happen not to overlap on the ground. No workload balancing (measured 2.13x spread on
  Ubin), altitude layering, launch scheduling, transit deconfliction, RTL separation,
  endurance model, reallocation or collision avoidance. Catalogued in
  `FlightPlanner/parameters.yaml` under `swarm_not_implemented`.
- **Exposure is not configurable.** It sets the binding blur speed cap, so a dawn/dusk
  launch needs it changed and currently cannot without editing source.
- **NVIDIA driver** — not installed. That's why the Gazebo GUI hangs; running HEADLESS
  meanwhile. `sudo ubuntu-drivers autoinstall` + reboot to get the 3D view.
- **ZR30 nadir-lock (later).** Plain `gz_x500` has NO camera/gimbal — don't add a gimbal
  command to its `.plan`. When ready: `make px4_sitl gz_x500_gimbal` + a "gimbal pitch
  -90" command at mission start; real ZR30 = set its gimbal to nadir-lock mode.

## Done

- **01 Sep**: idempotent setup script; PX4+Gazebo, QGC, geo/Python stack; Fields2Cover
  2.1.0 built; `mission_params.py` + `swaths_to_plan.py`; stages 2 and 3 proven.
- **03 Sep**: fixed PX4 SITL build (kconfig, gz models submodule, protoc exec bit,
  protobuf conflict); installed QGC 4.4.3; fixed the RTL frame-2 bug; flew the full
  synthetic-square mission headless in Gazebo; tuned turns.
- **08 Sep**: merged the Multi-Agent Flight Planner export with the drone-dev imaging
  model — camera model ported, complete takeoff→RTL missions, derived cruise speed,
  orientation-aware lanes, leg-checking guardrail, Voronoi seed de-duplication.
  Closed the "real map" item: authored a Pulau Ubin KML, planned 3 drones, flew drone-1
  end-to-end in Gazebo. **Survey cross-track max 4.95 m, mean 0.46 m** over 41,170
  samples; altitude held 29.95–30.04 m against a commanded 30.
- **09 Sep**: wrote `parameters.yaml` (annotated, tested against the code); found and
  fixed the cruise-speed gap — peak flown speed rose from 5.13 to 13.37 m/s; moved the
  speed item ahead of takeoff, cutting time-to-search-area from 155 s to 79 s; retired
  `planning/` to `legacy/`; rewrote README.md, CLAUDE.md and both records. Closed the
  last two recorded gaps: the guardrail now validates the takeoff transit and RTL
  return, and `droneCount` reports plans that exist. 90 tests. Method recorded in the
  addendum to `FlightPlanner/README.md`.
