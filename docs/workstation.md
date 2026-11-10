# CLAUDE.md — Drone SAR project

Search-and-rescue drone. This box is the ground-station / dev workstation.
Everything lives under `~/drone-dev/`. Run Claude Code from here.

Fuller records: `@sar-search-path-pipeline.md`, `@dev-workstation-setup.md` and
`@FlightPlanner/README.md`. Read those on demand — don't reload them for unrelated
tasks. Every tunable in the system is catalogued in `@FlightPlanner/parameters.yaml`.

**`history.md` is the running engineering log** — **newest first**, so the entries to read
when resuming are at the TOP of its `# Entries` section, not the bottom. Add new blocks
there too. Read the last few when resuming work, especially on the real aircraft: test
results, measurements and the reasoning behind them live there, not in the code.

**This workspace is NOT a git repository.** There is no undo. Archive rather than
delete (see `legacy/`), and never overwrite a file you have not read. The one exception
is `FlightPlanner/`, which IS its own git repo (branch `main`, remote `Drone-dev`) — commit
work there as it completes.

## Environment (facts, not preferences)

- Workstation OS: **Ubuntu 22.04 LTS**. ROS 2 runs **in Docker, not on the metal**.
- Active board: **Jetson AGX Orin**, JetPack 6.2.x (Jetson Linux 36.5), Ubuntu
  22.04, ROS **Humble**. Target board later: NX / AGX Thor, JetPack 7.1, 24.04,
  ROS **Jazzy** (Orin→Thor = container swap, not an OS reinstall).
- SITL checkout = **PX4 v1.16.0** (`6ea3539157`, verified 16 Sep 2026). The *physical*
  flight controller — a **MicoAir743-AIO** over USB — runs **PX4 v1.16.2**, so the two do
  not match; see the FC note below. Camera = **SIYI ZR30** (3-axis gimbal). Airframe in
  sim = **x500 multirotor**. YOLO/object-ID is a different team.
- venv for all Python work: `source ~/drone-dev/venv/bin/activate`
- Env overrides: `DRONE_HOME` (default `~/drone-dev`), `PX4_VERSION` (`v1.16.0`).

## Golden rules — these have already bitten us, do not relearn them

- **NEVER run `make distclean`** in PX4 — it de-inits ALL submodules. To clear a
  build use `rm -rf build/px4_sitl_default`.
- **`HEADLESS=1` is now optional.** This rule used to read "no NVIDIA driver installed, so
  the Gazebo 3D GUI software-renders and hangs" — that is **no longer true**. Verified
  16 Sep 2026: the driver is installed and `gz sim -g` hardware-renders (331 MiB VRAM,
  `libGLX_nvidia` mapped, ~37 % CPU, not pegged). Keep `HEADLESS=1` for long unattended
  runs — it is still faster — but the 3D view is available when you want to watch.
- **Run the PX4 binary with `-d`** for anything backgrounded or logged. Without it the
  `pxh>` shell redraws its prompt into a non-TTY forever — measured: a **2.7 GB log in
  a few minutes**. `make px4_sitl gz_x500` is interactive-only.
- **Under Gazebo, `PX4_HOME_LAT`/`PX4_HOME_LON` are IGNORED.** Home comes from the
  world SDF's `<spherical_coordinates>`. If the world origin and the mission disagree,
  PX4 rejects it with *"Geofence violation for waypoint 1"*. Make a world per site and
  select it with `PX4_GZ_WORLD=<name>`. Present: `ubin.sdf`, `toapayoh.sdf`, `safti.sdf`;
  `default.sdf` is at Tuas. These are **bare ground planes** — `toapayoh.sdf` contains one
  model (`ground_plane`) plus `sunUTC`, no buildings — so they place home correctly but the
  3D view shows the aircraft over featureless grey. Use QGC or `tools/plot_track.py` to see
  the mission against real ground.
- **PX4 SITL persists the mission and geofence** in
  `build/px4_sitl_default/rootfs/dataman` across restarts. A stale fence from a previous
  site will reject a valid new mission. `rm` it when changing sites.
- **`MIS_YAWMODE` DOES NOT EXIST in PX4 v1.16** — it is `MPC_YAW_MODE`, and the numeral
  does not carry over. Writing 1 gives "towards home" — nose pinned at the launch point
  for the whole survey. Since 11 Sep 2026 the survey flies **`MPC_YAW_MODE = 3`** ("along
  trajectory": yaw and bank together through a corner). 4 ("yaw first") stops dead at
  every corner to yaw — ~8 s each. Set in `sar_turn_params.params`; a test pins it.
- **A `.plan`'s `cruiseSpeed` field never reaches the vehicle.** It is a ground-station
  planning value. Without a `MAV_CMD_DO_CHANGE_SPEED` (178) item the aircraft flies
  `MPC_XY_CRUISE` — 5 m/s by default. The planner now emits that item before takeoff.
- **RTL waypoint in a `.plan` MUST use frame 2 (MAV_FRAME_MISSION).** Frame 3 →
  PX4 rejects the whole mission ("Command not supported" / "Invalid item").
- **Plain `gz_x500` has no camera/gimbal.** Do NOT add a gimbal command to its
  `.plan` — it re-triggers the invalid-item error. Gimbal work needs `gz_x500_gimbal`.
- **QGroundControl = the v4.4.3 AppImage (Qt5).** The master 5.x build needs 24.04+.
  It keeps a stale plan and stale error banners across reconnects — restart it when
  changing sites, and kill the real binary, not just the AppRun wrapper.
- **Protobuf conflict is real** (only if you rebuild Fields2Cover, now legacy-only).
  F2C's OR-Tools drops protobuf 25.3 + abseil into `/usr/local` and shadows the system
  protobuf 3.12.4 that apt-Gazebo uses → PX4 gz build fails. Fix = stash `/usr/local`
  protobuf to `/opt/pb-stash`, keep `libortools.so.9`, `ldconfig`, rebuild. Full file
  list in the pipeline doc. Revert before rebuilding F2C.
- PX4 build needs its Python deps **in the active venv**:
  `pip install -r ~/drone-dev/PX4-Autopilot/Tools/setup/requirements.txt`
  (fixes `No module named menuconfig`).
- **Never `pkill -f` a pattern that appears in your own command line** — it matches the
  shell running the command and kills it mid-script. Kill by numeric PID.
- Watch for scrambled terminal pastes (`^[[200~` bracketed-paste leak). Type
  short commands by hand.

## Key commands

```bash
source ~/drone-dev/venv/bin/activate
cd ~/drone-dev/FlightPlanner

# plan a sector from a KML
flight-planner "ops/pulau-ubin-west.kml" --output missions/ubin/mission.plan
flight-planner --ceiling 3.0                    # max altitude for 3 cm/px

# visualise before flying
python tools/plot_mission.py ops/pulau-ubin-west.kml missions/ubin/mission.plan out.png

# SITL (terminal 1) — headless, daemon mode, world matching the AO
cd ~/drone-dev/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=ubin GZ_IP=127.0.0.1 HEADLESS=1 \
  ../../../../bin/px4 -d < /dev/null > sitl.log 2>&1 &

# QGroundControl (terminal 2)
~/drone-dev/QGroundControl.AppImage ~/drone-dev/FlightPlanner/missions/ubin/mission_drones/drone-1.plan

# or fly it programmatically and record the track
python tools/fly_mission.py missions/ubin/mission_drones/drone-1.plan \
    --params ../sar_turn_params.params --track missions/ubin/track.json

python -m pytest -q                             # 135 tests
```

## The pipeline (one knob → whole mission)

```
KML (GeoFence + OpsArea)  →  camera.py (altitude → spacing 25 m, GSD, cruise speed)
                          →  partition for N drones (strips or voronoi)
                          →  one altitude layer per drone (30 / 32 / 28 m), imaging per layer
                          →  lanes along each sub-area's long axis, connectors routed
                             inside the OpsArea round any excluded notch
                          →  every turn drawn as a circle PX4 flies in one bank (turn_radius_m)
                          →  guardrail, hard-stop: legs inside the fence, survey and its
                             rounded corners inside the OpsArea, no two drones able to meet
                          →  .plan: DO_CHANGE_SPEED → takeoff → lawnmower → RTL
                          →  PX4 SITL + QGC / tools/fly_mission.py
```

Pick **flight altitude**; every downstream number re-derives from ZR30 geometry.
Turn *behaviour* (bank, yaw, acceleration) is set by PARAMS (`sar_turn_params.params`).
Since 14 Sep 2026 the turn *shape* is drawn in the `.plan`: every corner is a circle of
`turn_radius_m` (0 = half the lane spacing), drawn as the polygon *around* the circle — a
vertex every 45°, each with an acceptance radius of half its leg (param2) — so PX4's corner
geometry at each vertex matches one circle. Flown 14 Sep pm: U-turns in 7 s, survey as fast
as sharp corners, better tracking — but PX4 still re-plans a straight line at every
waypoint switch, so the bank comes as one ~16° pulse per vertex, not one steady bank. A
truly steady bank needs a continuous trajectory (offboard from the companion), not waypoints
— see `history.md`. (The first version, points *on* the circle with a 2 m radius, swung the
roll ±11–14° once per waypoint at ~2.7 m/s.) The old
rule — "never curve the `.plan`" — dates from yaw-first flying, when every extra waypoint
was another full stop. Corners beside an excluded notch stay sharp, with a tight 2 m radius.
Alternative, `turn_style: pre_yaw` (the "anti-centripetal" tactic, 14 Sep): sharp corners,
each waypoint carrying the heading of the leg leaving it, so the nose turns to the next leg
before the corner. A waypoint with a heading is not "reached" until the nose is within
`MIS_YAW_ERR` 12° — if the rotation isn't done, PX4 hovers at the corner. Flown 14 Sep:
the nose does turn first, but a multicopter's bank comes from changing direction, not from
yaw — total tilt was unchanged and the braking moved into roll (max 22°). Not the default.

Flown end-to-end over Pulau Ubin 08–09 Sep 2026: survey cross-track max 4.95 m,
mean 0.46 m; altitude held 29.95–30.04 m against a commanded 30.

## Where things live

```
~/drone-dev/
├── FlightPlanner/           THE ACTIVE PIPELINE
│   ├── src/flight_planner/  kml_parser · geometry · camera · scanner_tools ·
│   │                        agents · qgc_plan · cli · ollama_client · rag
│   ├── tools/               plot_mission · fly_mission · plot_track
│   ├── ops/                 KML sectors (SAFTI city, pulau-ubin-west)
│   ├── missions/            generated plans, tracks, screenshots
│   ├── parameters.yaml      every knob, annotated — tested against the code
│   └── tests/               135 tests
├── legacy/planning/         RETIRED single-drone pipeline (provenance only)
├── PX4-Autopilot/           SITL + Gazebo (worlds: default.sdf=Tuas, ubin.sdf=Ubin,
│                            toapayoh.sdf=Toa Payoh, safti.sdf=SAFTI — all bare ground planes)
├── Fields2Cover/            legacy-only; nothing on the active path imports it
├── QGroundControl.AppImage  QGC v4.4.3 (Qt5)
├── sar_turn_params.params   MPC_YAW_MODE 3 · NAV_ACC_RAD 5 · MPC_ACC_HOR 2.6 ·
│                            MPC_YAWRAUTO_MAX 90 · MPC_TILTMAX_AIR 25 · MPC_XY_VEL_MAX 20
└── venv/                    source venv/bin/activate
```

Note: `setup.sh`, described in `dev-workstation-setup.md`, is **not present** in this
workspace.

## Current open items

- **Swarm behaviour** is offline planning only: drones are split on the ground and
  stacked in 2 m altitude layers (since 11 Sep 2026), with no runtime coordination.
  Remaining gaps — workload balancing (measured 2.13× spread), launch scheduling (all
  drones still climb from one GCS point through each other's layers), RTL separation
  (confirmed broken 11 Sep: `RTL_RETURN_ALT` is 30 m, so the 28 m drone climbs onto
  drone-1's layer to come home), endurance, reallocation, collision avoidance —
  are catalogued in `FlightPlanner/parameters.yaml` under `swarm_not_implemented`.
- **RTL speed** — PX4 calls `reset_cruising_speed()` on RTL, so the return leg always
  flies `MPC_XY_CRUISE` (5 m/s) regardless of the mission's speed item.
- **NVIDIA driver** — RESOLVED. Installed and working as of 16 Sep 2026: RTX 6000 Ada
  Generation, driver 580.178.04, CUDA 13.0, 49 GB VRAM. The Gazebo 3D GUI
  hardware-renders; see the `HEADLESS=1` rule above.
- **Corner speed is capped by geometry, not by tuning** (measured 16 Sep 2026, toa-payoh
  drone-3, 28 m layer, 12.2 m/s commanded). The survey averaged **8.13 m/s** and spent
  **40 % of its time below 6 m/s**: 12 m/s down each lane, braking to ~3.5 m/s at every
  turn, 13 slow-downs in a 300 s window, pitch swinging −23.7°/+12.1°. Altitude held to
  0.25 m throughout, so this is not instability — it is PX4's per-waypoint speed limit
  `sqrt(MPC_ACC_HOR × param2 × tan(bend/2))`, which on a drawn-circle vertex
  (`param2` 4.83 m, 45° step) is **2.28 m/s**. Holding 12.2 m/s round the 11.65 m auto
  radius would need **52.5° of bank** against `MPC_TILTMAX_AIR` 25°; the radius needed is
  57 m at `MPC_ACC_HOR` 2.6. Since `turn_radius_m` auto-derives as half the lane spacing,
  and lane spacing comes from altitude, **altitude indirectly caps turn speed far below
  the blur-safe cruise the same altitude produces.** Consequence to remember: every
  mission-duration and endurance estimate derived from cruise speed is ~50 % optimistic.
- **ZR30 nadir-lock** — later; needs `gz_x500_gimbal` + a gimbal-pitch command.

## Reference (external docs, fetch when relevant)

- PX4 flight controllers: https://docs.px4.io/main/en/flight_controller/
- PX4 v1.16 Ubuntu dev setup: https://docs.px4.io/v1.16/en/dev_setup/dev_env_linux_ubuntu
- Jetson AGX Orin quick start:
  https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/IN/QuickStart.html
- Fields2Cover (legacy only): https://fields2cover.github.io/index.html
