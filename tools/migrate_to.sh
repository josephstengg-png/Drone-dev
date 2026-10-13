#!/usr/bin/env bash
# Hand the ground-station role to another machine, with the work and the history.
#
#     bash tools/migrate_to.sh --target user@192.168.144.51 --dry-run
#     bash tools/migrate_to.sh --target user@192.168.144.51
#     bash tools/migrate_to.sh --target user@192.168.144.51 --with-logs   # + 13 GB tlogs
#
# Run it ON the old ground station. It pushes; the new machine needs only sshd and a
# credential this one can use.
#
# WHAT IT CARRIES, and why each:
#   ~/drone-dev            the work itself, including PX4 and the repo
#   ~/.ssh                 the keys the companions and GitHub accept -- without these the
#                          new machine can reach no aircraft and push nothing
#   ~/.claude              Claude Code's memory, settings and this project's transcripts
#   ~/Desktop              parameter backups taken before live changes
#   ~/.gitconfig           authorship
#   ~/.config/QGroundControl.org   QGC's link config and parameter cache
#
# WHAT IT DOES NOT CARRY, deliberately:
#   ~/Documents/QGroundControl     13 GB of telemetry logs. Flight records, worth keeping,
#                                  but they are not needed to fly and they dominate the
#                                  transfer. --with-logs includes them.
#   apt packages                   ROS Humble, ~132 Gazebo packages, Docker. These must be
#                                  INSTALLED on the target, not copied; a package list is
#                                  written to the target for reference.
#
# AFTER THIS, on the new machine: the Claude memory is keyed to the PROJECT PATH, so the
# work must live at the same path (~/drone-dev) or the memory will not attach to it.

set -uo pipefail

TARGET=""; DRY=""; WITH_LOGS=0
while [ $# -gt 0 ]; do
    case "$1" in
        --target)    TARGET="$2"; shift 2 ;;
        --dry-run)   DRY="--dry-run"; shift ;;
        --with-logs) WITH_LOGS=1; shift ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
[ -n "$TARGET" ] || { echo "need --target user@host" >&2; exit 2; }

note() { echo "[migrate] $*"; }

note "checking $TARGET is reachable"
ssh -o BatchMode=yes -o ConnectTimeout=10 "$TARGET" true 2>/dev/null \
    || { echo "[migrate] cannot ssh to $TARGET without a password." >&2
         echo "[migrate] copy a key first:  ssh-copy-id $TARGET" >&2; exit 1; }
note "reachable"

# -a preserves permissions, which matters for ~/.ssh: a private key that arrives
# world-readable is refused by ssh and the new machine silently cannot log in anywhere.
RSYNC="rsync -aHAX --info=progress2 --human-readable $DRY"

send() {  # send <local path> <remote parent>
    local src="$1" dst="$2"
    [ -e "$src" ] || { note "skip (absent): $src"; return; }
    note "---- $src  ->  $TARGET:$dst"
    $RSYNC "$src" "$TARGET:$dst"
}

ssh "$TARGET" 'mkdir -p ~/.config' 2>/dev/null

send "$HOME/drone-dev"                     "~/"

# NEVER overwrite the target's authorized_keys: it holds the key THIS machine is logged in
# with, so clobbering it can lock the migration out of the machine it is still writing to.
# known_hosts is the target's own trust record and is equally none of our business.
note "---- $HOME/.ssh  ->  $TARGET:~/  (keeping the target's authorized_keys/known_hosts)"
$RSYNC --exclude authorized_keys --exclude 'known_hosts*' "$HOME/.ssh" "$TARGET:~/"
send "$HOME/.claude"                       "~/"
send "$HOME/Desktop"                       "~/"
send "$HOME/.gitconfig"                    "~/"
send "$HOME/.config/QGroundControl.org"    "~/.config/"

if [ "$WITH_LOGS" = 1 ]; then
    send "$HOME/Documents/QGroundControl" "~/Documents/"
else
    note "skipping ~/Documents/QGroundControl (telemetry logs); pass --with-logs for them"
fi

if [ -z "$DRY" ]; then
    note "writing the installed-package list to the target for reference"
    dpkg --get-selections | ssh "$TARGET" 'cat > ~/workstation-packages.txt'
    note "fixing permissions on the keys"
    ssh "$TARGET" 'chmod 700 ~/.ssh 2>/dev/null; chmod 600 ~/.ssh/id_* ~/.ssh/*deploy 2>/dev/null; true'
fi

cat <<EOF

[migrate] transfer done. On the NEW machine:

  1. address on the drone LAN (no DHCP there):
       cd ~/drone-dev/FlightPlanner
       bash tools/bootstrap_ground_station.sh --iface <wired-iface> --ip 192.168.144.51

  2. verify the chain:
       python3 tools/ssh_config.py --check
       python3 tools/provision.py --all --verify
       python3 tools/preflight.py --fleet fleet-artex-pair.yaml

  3. the heavy toolchains are NOT copied -- install if you want SITL and planning:
       ROS 2 Humble, Gazebo, Docker.  ~/workstation-packages.txt lists what was here.
       PX4-Autopilot came across but its build is machine-specific: rebuild with
       'rm -rf build/px4_sitl_default' then make, never 'make distclean'.

  4. only ONE machine should drive the aircraft at a time. Two ground stations on the
     same companions means two commanders.
EOF
