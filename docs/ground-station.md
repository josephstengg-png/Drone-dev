# Making a different machine the ground station

The ground station is whichever computer runs the orchestration tools and talks to the
companions over ssh. Nothing in this repo assumes it is any particular box — but three
things live outside the repo and have to be set up deliberately on a new one.

Written after moving off the original workstation (`aftershock-01`, 12 Oct 2026).

## 1. An address on the ground LAN — the one that catches people

**The wired drone LAN has no DHCP server.** The switch is unmanaged and nothing hands out
addresses: the Jetsons hold static addresses from their own `switch-static` NetworkManager
profile, and the workstation held `192.168.144.50/24` the same way. A laptop plugged into
that switch with a default "wired, automatic" profile will sit there with a link-local
169.254.x.x address and reach nothing, while looking perfectly healthy.

Configure it manually, and keep it off your default route so it cannot hijack your
internet:

```bash
nmcli con add type ethernet ifname <iface> con-name drone-lan \
    ipv4.method manual ipv4.addresses 192.168.144.51/24 ipv4.never-default yes \
    ipv6.method ignore
nmcli con up drone-lan
```

Use **.51**, not .50, so the old workstation can stay plugged in without a clash. Any free
address in `192.168.144.0/24` works; record it in `companions.yaml` under
`ground_station`.

Check it:

```bash
cat /sys/class/net/<iface>/carrier      # 1 = a cable to something powered
ping -c3 192.168.144.101                # jetson-1
```

A `carrier` of 0 means the cable or the switch, not the aircraft. That exact confusion
cost an afternoon on 12 Oct 2026, when the switch was simply unpowered and both companions
looked dead.

## 2. An ssh key the companions accept

The Jetsons authorise one key (see `companions.yaml` for the user and expected path).
Either:

**Copy the existing keypair** — simplest, works immediately, no change on the companions:

```bash
scp old-workstation:~/.ssh/id_ed25519{,.pub} ~/.ssh/ && chmod 600 ~/.ssh/id_ed25519
```

**Or authorise a new key** — better hygiene, and it lets you revoke one machine without
affecting the other. Do this *while the old machine still works*:

```bash
# on the new laptop
ssh-keygen -t ed25519
# then, from the old workstation, for each companion
ssh jetson-1 "echo '<paste the laptop public key>' >> ~/.ssh/authorized_keys"
ssh jetson-2 "echo '<paste the laptop public key>' >> ~/.ssh/authorized_keys"
```

Then make the host names resolvable — they are ssh aliases, not DNS:

```bash
python3 tools/ssh_config.py --install
```

## 3. The Python environment

```bash
python3 -m venv .venv && . .venv/bin/activate && pip install -e .
```

`pymavlink` and `pyyaml` come with it. LibreOffice and `pandoc` are *not* required by any
flight tool — they only matter for document generation.

## Verify, in this order

```bash
python3 tools/ssh_config.py --check            # names resolve to the right addresses
python3 tools/provision.py --all --verify      # companions flight-ready, code deployed
python3 tools/preflight.py --fleet fleet-artex-pair.yaml    # GO / NO-GO per aircraft
```

If `provision --verify` reports missing code, run it without `--verify` to deploy.

## What does NOT travel, and never should

- **The ssh private key** — copy it deliberately or issue a new one, as above.
- **The ELRS backpack WiFi PSK** — passed through the environment to `tools/jetson_net.sh`
  and never written to this repo.
- **Claude Code's per-project memory**, which is stored under `~/.claude/` on each machine
  and keyed to the project path. The durable, portable version of that knowledge is the
  root `CLAUDE.md` and `docs/history.md`, both of which are in the repo precisely so a new
  machine starts informed.

## What the companions do not care about

Nothing above changes the aircraft or the companions. They run the relay as a systemd
service and hold their own network configuration, so they will keep flying with whichever
ground station is plugged in — or none at all. That is deliberate: the agent and its
safety ladder run on the companion so that a ground station failure is not an aircraft
failure.
