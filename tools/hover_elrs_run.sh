#!/bin/bash
# Operator-run 0.8 m hover on ARTEX113: take off, hover, descend, land, disarm in OFFBOARD.
#
#   ssh -t Orin_NX@192.168.144.102 'cd ~/FlightPlanner && bash tools/hover_elrs_run.sh [radio|usb]'
#
# Written 3 Nov 2026 at the lab manager's request, with the operator taking responsibility for
# the run. The agent only arms when THE PERSON AT THIS TERMINAL types ARM; nothing else sends it.
#
# KNOWN RISK, stated so it is on the page: over `radio` the height loop runs on readings up to 2 s
# old (ELRS carries LOCAL_POSITION_NED at 0.5 Hz). A 0.05 error in the hover-thrust guess moves
# the aircraft ~2 m in that time (29 Oct analysis), so it can overshoot the tether or hit the
# floor before the agent reacts. Only the pilot's switches act in real time. With NAV_DLL_ACT 3,
# a ground-station heartbeat gap over 5 s also makes PX4 LAND against the pilot.
set -u
LINK_KIND="${1:-radio}"
case "$LINK_KIND" in
  radio) LINK="udpout://127.0.0.1:14540"; EXTRA="--allow-radio-flight --height-timeout-s 3" ;;
  usb)   LINK="serial:/dev/ttyACM0";      EXTRA="" ;;
  *) echo "usage: $0 [radio|usb]"; exit 2 ;;
esac
TRACE="/tmp/hover08_$(date +%H%M%S).csv"

cat <<EOF

=== ARTEX113 0.8 m hover, link: $LINK_KIND ===
Pilot, before this starts:
  - tether tied to something FIXED, stopping the aircraft by about 1.0 m
  - ch5 ON (Offboard), ch7 CENTRE (Altitude), ch8 kill OFF, ch6 off, throttle centred
  - TAKE OVER = ch5 OFF.  STOP = ch8 kill, HELD 5 s (a shorter kill leaves it armed)
  - take over at once on drift over 20 cm, tilt, tether pull, or any oscillation
  - clean reboot done within the last few minutes
EOF
read -r -p "Type exactly  PROPS ON TETHERED AREA CLEAR  to continue: " ok
[ "$ok" = "PROPS ON TETHERED AREA CLEAR" ] || { echo "not confirmed -- nothing sent"; exit 1; }

# the agent refuses a real aircraft without --real-aircraft, --expect-sysid and --gate (it asks for ARM at READY)
exec python3 tools/drone_agent.py offboard-flight --link "$LINK" --expect-sysid 113 --real-aircraft --gate \
  $EXTRA --stage disarm --height 0.8 --wait-s 15 --takeoff-pause-m 0.5 --lift-by-climb --ground-attitude \
  --trace "$TRACE"
