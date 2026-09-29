#!/usr/bin/env bash
# Bring a Jetson up on an UNMANAGED switch, where nothing serves DHCP.
#
#     tools/switch_link.sh discover                  # who is on the wire, by link-local
#     tools/switch_link.sh setup fe80::xxx%enp130s0 192.168.144.101 jetson-1
#     tools/switch_link.sh check                     # both ends, addresses and reachability
#
# THE PROBLEM THIS SOLVES. On the office LAN both machines took DHCP and found each other.
# An unmanaged switch serves nothing: it forwards frames and has no IP of its own, so DHCP
# fails at both ends and neither side has an IPv4 address to be reached at. That is a layer-3
# outage only -- the switch is passing traffic fine, 1000 Mb carrier on every populated port.
#
# THE WAY IN IS IPv6 LINK-LOCAL. Every interface with carrier self-assigns an fe80:: address
# with no server, no lease and no configuration, and every IP host answers the all-nodes
# multicast ff02::1. So a Jetson with no usable IPv4 is still reachable:
#
#     ssh Orin_NX@fe80::de4a:9eff:fee1:f2a6%enp130s0
#
# That is what `discover` finds and what `setup` uses to install a static IPv4, after which
# the ordinary addresses work and nothing else in the stack has to know this happened.

set -uo pipefail

IFACE="${IFACE:-enp130s0}"
USER_NAME="${JETSON_USER:-Orin_NX}"
JETSON_OUI="4c:bb:47"          # both Orin NX carrier boards
SUBNET="${SUBNET:-192.168.144}"
WS_IP="${WS_IP:-192.168.144.50}"

note() { echo "[switch-link] $*"; }

discover() {
    note "carrier on $IFACE: $(cat /sys/class/net/$IFACE/carrier) at $(cat /sys/class/net/$IFACE/speed 2>/dev/null)Mb"
    note "soliciting every host on the wire (ff02::1) ..."
    # An address like fe80::eec3:258:8625:bd28%enp130s0: is field 4 with a trailing colon.
    # Do NOT cut at the first colon -- that truncates every address to "fe80".
    ping6 -c 4 -I "$IFACE" ff02::1 2>/dev/null | awk '/bytes from/{sub(/:$/,"",$4); print $4}' \
        | sort -u > /tmp/.sl_hosts
    local mine; mine="$(ip -6 addr show "$IFACE" | awk '/inet6 fe80/{print $2}' | cut -d/ -f1)"
    local found=0
    while read -r ll; do
        [ -z "$ll" ] && continue
        [ "${ll%\%*}" = "$mine" ] && continue
        local mac; mac="$(ip -6 neigh show dev "$IFACE" | awk -v a="${ll%\%*}" '$1==a{print $3}')"
        found=$((found+1))
        if [[ "$mac" == "$JETSON_OUI"* ]]; then
            echo "  JETSON  $ll  mac $mac"
            echo "          ssh $USER_NAME@$ll"
        else
            echo "  other   $ll  mac ${mac:-unknown}"
        fi
    done < /tmp/.sl_hosts
    rm -f /tmp/.sl_hosts
    [ "$found" -eq 0 ] && note "nothing else on the wire -- the far end is powered off, or its NIC has no carrier"
    return 0
}

setup() {
    local target="$1" newip="$2" label="${3:-jetson}"
    [ -n "$target" ] && [ -n "$newip" ] || { echo "usage: setup <fe80::x%iface> <ipv4> [label]"; exit 2; }
    note "configuring $label at $target -> $newip/24"
    ssh -o StrictHostKeyChecking=accept-new "$USER_NAME@$target" bash -s <<REMOTE
set -e
ETH=\$(nmcli -t -f DEVICE,TYPE dev status | awk -F: '\$2=="ethernet"{print \$1; exit}')
echo "  ethernet device: \$ETH"
# A static profile, added rather than editing whatever is there, so the DHCP one survives
# for when this board goes back on a served network.
echo jetson | sudo -S -p '' nmcli con add type ethernet ifname "\$ETH" con-name switch-static \
    ipv4.method manual ipv4.addresses $newip/24 ipv6.method link-local \
    connection.autoconnect yes connection.autoconnect-priority 20 2>/dev/null || \
echo jetson | sudo -S -p '' nmcli con modify switch-static ipv4.addresses $newip/24
echo jetson | sudo -S -p '' nmcli con up switch-static >/dev/null
sleep 2
ip -br addr show "\$ETH"
REMOTE
}

check() {
    echo "=== workstation ($IFACE, switch port 5) ==="
    ip -br addr show "$IFACE"
    echo "  mac $(cat /sys/class/net/$IFACE/address)"
    echo
    echo "=== reachability across the switch ==="
    for ip in "$SUBNET".101 "$SUBNET".102; do
        printf "  %-16s " "$ip"
        ping -c2 -W2 "$ip" >/dev/null 2>&1 && echo "UP" || echo "no response"
    done
}

case "${1:-discover}" in
    discover) discover ;;
    setup)    shift; setup "$@" ;;
    check)    check ;;
    *) echo "usage: $0 {discover|setup <fe80::x%iface> <ipv4> [label]|check}"; exit 2 ;;
esac
