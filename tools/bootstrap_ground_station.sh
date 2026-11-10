#!/usr/bin/env bash
# Turn a fresh Ubuntu machine into the ground station. Run it ON the new machine.
#
#     bash tools/bootstrap_ground_station.sh --iface enp0s31f6 --ip 192.168.144.51
#     bash tools/bootstrap_ground_station.sh --check          # verify only, change nothing
#
# WHY THIS EXISTS. Moving off the original workstation (13 Oct 2026) exposed how much of
# "it works" lived on one box rather than in the repo. Three things are needed and none of
# them arrive with a git clone: an address on a LAN that has no DHCP, an ssh key the
# companions accept, and a Python environment. This does the parts that can be automated
# and tells you plainly about the parts that cannot.
#
# IT WILL NOT touch the aircraft or the companions. Those run their own relay as a systemd
# service and hold their own network configuration, so they do not care which ground
# station is plugged in -- which is the point of running the agent on the companion.

set -uo pipefail

IFACE=""; IP="192.168.144.51/24"; CHECK=0; PROFILE="drone-lan"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

while [ $# -gt 0 ]; do
    case "$1" in
        --iface) IFACE="$2"; shift 2 ;;
        --ip)    IP="$2"; [[ "$IP" == */* ]] || IP="$IP/24"; shift 2 ;;
        --check) CHECK=1; shift ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

note() { echo "[bootstrap] $*"; }
warn() { echo "[bootstrap] WARNING: $*" >&2; }

# ---------------------------------------------------------------- 1. python
note "python environment"
if [ ! -d "$ROOT/.venv" ]; then
    if [ "$CHECK" = 1 ]; then
        warn "no .venv -- run without --check"
    else
        python3 -m venv "$ROOT/.venv" || { warn "could not create .venv"; exit 1; }
        note "created $ROOT/.venv"
    fi
fi
if [ -d "$ROOT/.venv" ]; then
    # shellcheck disable=SC1091
    . "$ROOT/.venv/bin/activate"
    if [ "$CHECK" = 0 ]; then
        pip install --quiet --upgrade pip
        pip install --quiet -e "$ROOT" || warn "pip install -e failed"
    fi
    for m in pymavlink yaml; do
        python3 -c "import $m" 2>/dev/null \
            && note "  import $m: ok" \
            || warn "  import $m: MISSING -- the flight tools will not run"
    done
fi

# ---------------------------------------------------------------- 2. the drone LAN
note "ground LAN (no DHCP on this subnet -- the address must be set by hand)"
if [ -z "$IFACE" ]; then
    IFACE=$(ls /sys/class/net | grep -E '^(en|eth)' | head -1)
    [ -n "$IFACE" ] && note "  guessed wired interface: $IFACE"
fi
if [ -z "$IFACE" ]; then
    warn "  no wired interface found; pass --iface"
else
    carrier=$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || echo 0)
    note "  $IFACE carrier=$carrier  (1 = a cable to something powered)"
    [ "$carrier" = 1 ] || warn "  no carrier -- check the cable and that the switch is ON"
    have=$(ip -4 -br addr show "$IFACE" 2>/dev/null | awk '{print $3}')
    note "  current address: ${have:-none}"
    if [ "$CHECK" = 0 ] && [[ "$have" != 192.168.144.* ]]; then
        if nmcli -t -f NAME c show 2>/dev/null | grep -qx "$PROFILE"; then
            nmcli con mod "$PROFILE" ipv4.addresses "$IP" ipv4.method manual \
                ipv4.never-default yes ipv6.method ignore
        else
            # never-default matters: without it the drone LAN can capture the laptop's
            # default route and take its internet away.
            nmcli con add type ethernet ifname "$IFACE" con-name "$PROFILE" \
                ipv4.method manual ipv4.addresses "$IP" ipv4.never-default yes \
                ipv6.method ignore >/dev/null
        fi
        nmcli con up "$PROFILE" >/dev/null 2>&1 && note "  brought up $PROFILE at $IP" \
            || warn "  could not bring up $PROFILE"
    fi
fi

# ---------------------------------------------------------------- 3. ssh
note "ssh to the companions"
if [ ! -f "$HOME/.ssh/id_ed25519" ]; then
    warn "  no ~/.ssh/id_ed25519. Either copy the keypair from the old ground station,"
    warn "  or ssh-keygen here and append the new PUBLIC key to each companion's"
    warn "  ~/.ssh/authorized_keys -- do that while the old machine still works."
else
    note "  key present"
    if [ "$CHECK" = 0 ]; then
        python3 "$ROOT/tools/ssh_config.py" --install 2>&1 | sed 's/^/  /'
    fi
    for h in $(python3 - <<'PY'
import yaml, pathlib
spec = yaml.safe_load((pathlib.Path("companions.yaml")).read_text())
print(" ".join(spec.get("companions", {})))
PY
    ); do
        if timeout 12 ssh -n -o BatchMode=yes -o ConnectTimeout=8 "$h" true 2>/dev/null; then
            note "  $h: reachable"
        else
            warn "  $h: NOT reachable (address, key, or the companion is off)"
        fi
    done
fi

echo
note "next:  python3 tools/provision.py --all --verify"
note "then:  python3 tools/preflight.py --fleet fleet-artex-pair.yaml"
note "QGroundControl is optional and is a telemetry display only over ELRS --"
note "  see CLAUDE.md. Download the v4.4.3 AppImage if you want it."
