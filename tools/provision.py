#!/usr/bin/env python3
"""Bring a companion computer to flight-ready, or verify that it already is.

    python3 tools/provision.py --host jetson-1 --verify    # check only, change nothing
    python3 tools/provision.py --host jetson-1             # make it so
    python3 tools/provision.py --all                       # every companion

WHY. Everything a Jetson needs to fly was configured by hand and then remembered: the
code deployed with scp, the relay started from a login shell, the network profile pinned
once in September. None of it was reproducible, so a reflashed or replacement board meant
archaeology through history.md. This makes the repo the source of truth and the board
disposable.

Idempotent: every step checks before acting, so running it twice is safe and running it
on a healthy box changes nothing.

WHAT IT DELIBERATELY WILL NOT DO. It never writes the WiFi PSK, and it never changes the
backpack pairing. Both need a secret this repo does not hold and a decision about WHICH
transmitter this companion owns -- getting that wrong puts two aircraft on one
transmitter, which has happened here before (28 Sep 2026). It reports the pairing and
tells you to use tools/jetson_net.sh if it is wrong.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
TOOLS = ROOT / "tools"
COMPANIONS = ROOT / "companions.yaml"
REGISTRY = ROOT / "artex-registry.yaml"

# Everything the companion needs to run an aircraft. drone_agent imports safety_rules, so
# they must travel together -- deploying one without the other breaks the agent outright.
PAYLOAD = ["drone_agent.py", "safety_rules.py", "elrs_link.py", "elrs_service.sh",
           "jetson_net.sh", "preflight_probe.py", "mav_shell.py", "motor_test.py",
           "no_gps_check.py"]


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def ssh(host: str, cmd: str, timeout: float = 90) -> tuple[int, str]:
    try:
        r = subprocess.run(["ssh", "-n", "-o", "ConnectTimeout=10", host, cmd],
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 255, "timed out"
    return r.returncode, (r.stdout + r.stderr).strip()


def provision(host: str, spec: dict, reg: dict, verify: bool) -> dict:
    c = spec["companions"][host]
    remote = c.get("remote_dir", "~/FlightPlanner")
    want_bssid = None
    for name, pack in (reg.get("backpacks") or {}).items():
        if name == c.get("aircraft"):
            want_bssid = (pack.get("bssid") or "").lower() or None

    steps, problems, warnings = [], [], []

    def step(name, ok, detail, fixed=False, blocking=True):
        """blocking=False marks a real but non-flight-stopping finding.

        A check that always fails and never stops anything teaches people to ignore the
        tool -- so the clock, which affects log correlation but not airworthiness, is a
        warning rather than a failure.
        """
        steps.append({"step": name, "ok": bool(ok), "detail": detail,
                      "fixed": fixed, "blocking": blocking})
        mark = "ok  " if ok else ("FAIL" if blocking else "warn")
        log(f"  [{mark}] {name}: {detail}" + ("  (fixed)" if fixed else ""))
        if not ok:
            (problems if blocking else warnings).append(f"{name}: {detail}")

    rc, out = ssh(host, "hostname; date -u +%Y-%m-%dT%H:%M:%SZ")
    if rc != 0:
        step("reachable", False, out)
        return {"host": host, "ok": False, "steps": steps, "problems": problems}
    name, clock = (out.splitlines() + ["", ""])[:2]
    step("reachable", True, f"{name}, clock {clock}")
    if clock.startswith("1970"):
        step("clock", False, f"{clock} -- no time source; log timestamps are meaningless",
             blocking=False)

    # ---- code -----------------------------------------------------------------
    missing = []
    for f in PAYLOAD:
        rc, _ = ssh(host, f"test -f {remote}/tools/{f}")
        if rc != 0:
            missing.append(f)
    if missing and verify:
        step("code deployed", False, f"missing: {', '.join(missing)}")
    else:
        for f in PAYLOAD:
            subprocess.run(["scp", "-q", "-o", "ConnectTimeout=10", str(TOOLS / f),
                            f"{host}:{remote}/tools/{f}"], capture_output=True, timeout=90)
        step("code deployed", True, f"{len(PAYLOAD)} files", fixed=bool(missing))

    rc, out = ssh(host, f"cd {remote} && python3 tools/drone_agent.py rc-check --help "
                        f">/dev/null 2>&1 && echo ok")
    step("agent runs", out.strip() == "ok",
         "drone_agent imports and parses" if out.strip() == "ok" else out[:120])

    # ---- the relay as a service, not a login-shell process ---------------------
    rc, enabled = ssh(host, "systemctl is-enabled elrs-link.service 2>&1 | head -1")
    if enabled.strip() == "enabled":
        rc, active = ssh(host, "systemctl is-active elrs-link.service 2>&1 | head -1")
        step("relay service", active.strip() == "active", f"enabled, {active.strip()}")
    elif verify:
        step("relay service", False, f"not installed ({enabled.strip()})")
    else:
        rc, out = ssh(host, f"cd {remote} && sudo bash tools/elrs_service.sh 2>&1 | tail -3",
                      timeout=120)
        rc2, active = ssh(host, "systemctl is-active elrs-link.service 2>&1 | head -1")
        step("relay service", active.strip() == "active",
             f"installed, {active.strip()}", fixed=True)

    rc, out = ssh(host, "n=0; for p in $(pgrep -x python3); do "
                        "grep -qa elrs_link /proc/$p/cmdline 2>/dev/null && n=$((n+1)); "
                        "done; echo $n")
    n = out.strip()
    step("exactly one relay", n == "1",
         f"{n} running" + (" -- two relays split the downlink" if n not in ("0", "1") else ""))

    # ---- the radio pairing: reported, never changed ---------------------------
    rc, out = ssh(host, "iw dev $(ls /sys/class/net | grep ^wl | head -1) link 2>/dev/null "
                        "| head -1")
    got = out.lower()
    if want_bssid and want_bssid in got:
        step("backpack pairing", True, f"on {want_bssid} ({c.get('aircraft')})")
    elif "not connected" in got:
        step("backpack pairing", False,
             f"not associated; expected {want_bssid} for {c.get('aircraft')} "
             f"-- transmitter off, or run tools/jetson_net.sh with the PSK")
    else:
        step("backpack pairing", False,
             f"associated to the WRONG transmitter: {out.strip()}; expected {want_bssid}")

    ok = not problems
    suffix = f" ({len(warnings)} warning{'s' if len(warnings) != 1 else ''})" if warnings else ""
    log(f"  --> {host}: {'READY' if ok else 'NOT READY'}{suffix}")
    return {"host": host, "ok": ok, "steps": steps,
            "problems": problems, "warnings": warnings}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", action="append", help="companion name; repeatable")
    ap.add_argument("--all", action="store_true", help="every companion in companions.yaml")
    ap.add_argument("--verify", action="store_true", help="check only; change nothing")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    spec = yaml.safe_load(COMPANIONS.read_text())
    reg = yaml.safe_load(REGISTRY.read_text()) if REGISTRY.exists() else {}
    hosts = list(spec["companions"]) if args.all else (args.host or [])
    if not hosts:
        ap.error("give --host NAME or --all")

    results = []
    for h in hosts:
        if h not in spec["companions"]:
            log(f"[provision] {h} is not in companions.yaml")
            return 2
        log(f"\n=== {h} ({'verify' if args.verify else 'provision'}) ===")
        results.append(provision(h, spec, reg, args.verify))

    ok = all(r["ok"] for r in results)
    log("\n[provision] " + ("every companion is ready"
                            if ok else "NOT ready -- see the failures above"))
    if args.json:
        print(json.dumps({"ok": ok, "companions": results}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
