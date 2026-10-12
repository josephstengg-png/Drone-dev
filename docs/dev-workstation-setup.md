# Dev Workstation Setup — Drone SAR

How we prep a new Ubuntu dev box for the SAR drone project, plus the version
decisions and why we made them.

> **Status (09 Sep 2026):** the `setup.sh` script described at the bottom of this file is
> **not present in this workspace**. The layer breakdown below still records what has to
> be installed and why, so treat it as the checklist rather than as documentation of a
> script you can run. Also note this workspace is **not a git repository** — there is no
> undo, so archive rather than delete.

## Hardware targets (what we're building for)

| | AGX Orin (have now) | NX / AGX Thor (target) |
|---|---|---|
| JetPack | 6.2.x (Jetson Linux 36.5) | 7.1 |
| Onboard Ubuntu | 22.04 | 24.04 (kernel 6.8) |
| Native ROS 2 | Humble | Jazzy |
| Orin on JP7? | not yet — coming in JP 7.2 | — |

The ecosystem is mid-migration 22.04/Humble -> 24.04/Jazzy, and our two boards
straddle the gap. FCC = PX4. Camera = SIYI ZR30. YOLO/object-ID is a separate
team's responsibility.

## The Ubuntu-version decision

**Workstation OS = Ubuntu 22.04 LTS. Run ROS in Docker, not on the metal.**

Why:
- Active board is the Orin on JP6/Humble; matching the metal to it = zero friction
  on the hardware we're actually touching, and the Jetson container ecosystem
  (Isaac ROS, jetson-containers) is most mature on JP6/Humble right now.
- The Orin -> Thor transition then becomes a container swap (Humble image ->
  Jazzy image), not an OS reinstall.
- SDK Manager v2.4.1 runs on 20.04/22.04/24.04 hosts and flashes BOTH Orin (JP6)
  and Thor (JP7), so the flasher doesn't force the choice.
- PX4 v1.16 builds fine on 22.04 and 24.04.

24.04 + Jazzy on the metal is a defensible alternative if we want to bias toward
Thor, but it runs slightly ahead of where the Orin/JP6 tooling is comfortable.

## Install stack (layers)

- **0 base**: git, curl, build-essential, cmake, python3 dev/venv.
- **GPU (auto-detected)**: NVIDIA driver + Docker + NVIDIA Container Toolkit —
  only if a card is present.
- **1 flight stack**: QGroundControl (AppImage), PX4-Autopilot + `Tools/setup/
  ubuntu.sh` (pulls toolchain + Gazebo for SITL), MAVSDK-Python + pymavlink
  (programmatic mission upload = pipeline stage 3).
- **2 planning stack** (in a venv): rasterio/pyproj/shapely/geopandas + GDAL
  (georeferencing), opencv-python (homography), numpy/matplotlib, PyYAML, Pillow.
  **Fields2Cover is no longer required.** The coverage engine is now in-tree
  (`FlightPlanner/src/flight_planner/scanner_tools.py` picks lane direction from the
  minimum-width hull edge), so the C++ source build — and the protobuf/OR-Tools conflict
  that came with it — is only needed if you want to run the retired
  `legacy/planning/` pipeline.
- **2b flight tooling** (same venv): `mavsdk` (uploads a `.plan` through its
  QGroundControl importer and flies it), `pymavlink`, and `python-xlib` (used by the
  screenshot helper to capture the QGC window on an X display).
- **3 Jetson tools**: NVIDIA SDK Manager (manual — needs dev login). Flashes Orin
  -> JP6.2.x and Thor -> JP7.x.
- **4 ROS 2 bridge**: ROS 2 in Docker (Humble for Orin, Jazzy for Thor) + uXRCE-DDS
  agent / px4_ros_com — the modern PX4<->ROS2 link (MAVROS is the older path).
- **5 editor**: VS Code + Python/C++/Docker/ROS/Remote-SSH extensions.

## Where the KML comes from

Stage 1 is "draw the area of operations". Today that is done by hand — a polygon drawn in
Google Earth, or authored directly as KML — producing `GCSlocation` (a Point),
`GeoFence` and `OpsArea` (Polygons). `FlightPlanner/ops/` holds the sectors in use.
Anything the parser cannot find by name is rejected loudly rather than guessed at.

## Gemini's role

Not a "download" — it's the vision brain for stage 1 (drawing the AO boundary box
off the satnav image). It's an API key + the `google-genai` SDK, cloud-only.
Fine for ground-station planning; it will NOT run onboard the Jetson offline.
Architecture stays: Gemini draws the box on the ground -> georeference + plan ->
upload mission. Anything the drone must decide in-air with no link = onboard
YOLO/CV, not Gemini.

## The setup.sh script (not present — kept as the intended design)

Idempotent: on each run it inspects installed assets, compares versions, then
installs (missing) / updates (older) / skips (current). Safe to re-run.

```
./setup.sh                 # Layers 0-2 (+GPU if a card is found)
./setup.sh --dry-run       # report only, change nothing  <- run this first
./setup.sh --all           # everything incl. --jetson/--ros/--vscode
./setup.sh --no-gpu        # force-skip GPU layer
./setup.sh --force         # re-pull/rebuild source items even if present
```

Env overrides: `DRONE_HOME` (default `~/drone-dev`), `PX4_VERSION` (default
`v1.16.0`). SDK Manager and the ROS Docker images stay manual/opt-in because
they need an NVIDIA login / are heavy.

## Sources
- SDK Manager system requirements: https://docs.nvidia.com/sdk-manager/system-requirements/index.html
- JetPack 7.1 / Thor: https://jetsonhacks.com/2026/01/12/jetpack-7-1-and-jetson-t4000-now-available/
- JetPack 6.2 / Orin: https://developer.nvidia.com/embedded/jetpack-sdk-62
- PX4 Ubuntu dev setup (v1.16): https://docs.px4.io/v1.16/en/dev_setup/dev_env_linux_ubuntu
- Fields2Cover: https://fields2cover.github.io/index.html

## Verifying a box is ready

```bash
source ~/drone-dev/venv/bin/activate
cd ~/drone-dev/FlightPlanner
python -m pytest -q                      # 90 tests, no hardware needed
flight-planner --ceiling 3.0             # -> 34.6 m, exercises the camera model
```

For a SITL check, note that Gazebo takes its home position from the world SDF, not from
`PX4_HOME_LAT`/`PX4_HOME_LON`, and that the PX4 binary needs `-d` when backgrounded.
Both are covered in `CLAUDE.md` under Golden rules.
