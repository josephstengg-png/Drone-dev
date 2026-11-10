#!/usr/bin/env bash
# Install the generated ARTEX Gazebo model + SITL airframe into a Jetson's PX4 tree and rebuild.
#     bash sim/deploy_artex_sim.sh jetson-2
set -euo pipefail
host=${1:?usage: deploy_artex_sim.sh <host>}
here=$(cd "$(dirname "$0")" && pwd)
scp -q -r "$here/gz_artex/models/artex_base" "$here/gz_artex/models/artex" "$host:PX4-Autopilot/Tools/simulation/gz/models/"
scp -q "$here/gz_artex/airframes/4099_gz_artex" "$host:PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/"
ssh "$host" 'cd ~/PX4-Autopilot && f=ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
  grep -q "4099_gz_artex" $f || sed -i "s/^\(\s*\)4001_gz_x500$/&\n\14099_gz_artex/" $f
  grep -n "4099_gz_artex" $f && make px4_sitl_default 2>&1 | tail -2'
