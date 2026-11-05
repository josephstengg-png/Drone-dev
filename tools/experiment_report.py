#!/usr/bin/env python3
"""Analysis for the 4 Nov 2026 radio-uplink experiments. Pure Python.

    python3 tools/experiment_report.py exp1 fc-backups/2026-11-04/exp1.jsonl
    python3 tools/experiment_report.py exp2 [runs/]

exp1: command round trip against setpoint rate (tools-free experiment, aircraft disarmed). A command with no ack in 15 s counts as 15 s for
      ranking and as a loss. Each rate is tested against the two pooled idle blocks with a Mann-Whitney U test, and idle A is tested against
      idle B as the control for link drift.
exp3: the GUI runs named "EXP3..." (arm stage, props off, window 60 s, hands off): does Offboard survive the hold at each setpoint rate?
exp2: the GUI runs named "EXP2..." (arm stage, props off, setpoint rate 20 vs 3): seconds from ARM sent to "Armed by external command".
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

TIMEOUT_S = 15.0


def mann_whitney(x: list[float], y: list[float]) -> tuple[float, float]:
    """(U of x, two-sided p) by the normal approximation with tie correction and continuity correction."""
    n1, n2 = len(x), len(y)
    if not n1 or not n2:
        return float("nan"), float("nan")
    pooled = sorted([(v, 0) for v in x] + [(v, 1) for v in y])
    ranks, i = [0.0] * len(pooled), 0
    tie_term = 0.0
    while i < len(pooled):
        j = i
        while j + 1 < len(pooled) and pooled[j + 1][0] == pooled[i][0]:
            j += 1
        for k in range(i, j + 1):
            ranks[k] = (i + j) / 2 + 1
        t = j - i + 1
        tie_term += t ** 3 - t
        i = j + 1
    r1 = sum(r for r, (_, g) in zip(ranks, pooled) if g == 0)
    u1 = r1 - n1 * (n1 + 1) / 2
    n = n1 + n2
    sd = math.sqrt(n1 * n2 / 12 * ((n + 1) - tie_term / (n * (n - 1))))
    if sd == 0:
        return u1, 1.0
    z = (abs(u1 - n1 * n2 / 2) - 0.5) / sd
    return u1, math.erfc(max(z, 0.0) / math.sqrt(2))


def pct(xs: list[float], q: float) -> float:
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))] if s else float("nan")


def summarise(rows: list[dict]) -> dict:
    """rows: dicts with rtt (seconds or None). Returns n, loss %, median, p90 (losses counted as TIMEOUT_S)."""
    vals = [TIMEOUT_S if r["rtt"] is None else r["rtt"] for r in rows]
    return dict(n=len(vals), loss=100.0 * sum(r["rtt"] is None for r in rows) / len(rows) if rows else float("nan"),
                median=statistics.median(vals) if vals else float("nan"), p90=pct(vals, 0.9), vals=vals)


def exp1(path: str) -> int:
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    cmds = [r for r in rows if "rtt" in r]
    blocks = [r for r in rows if "block_s" in r]
    by = {}
    for r in cmds:
        by.setdefault(r["cond"], []).append(r)
    idle = summarise(by.get("idleA", []) + by.get("idleB", []))
    print(f"{'condition':8s} {'n':>3s} {'loss%':>6s} {'median s':>9s} {'p90 s':>7s} {'vs idle p':>10s}   downlink msg/s  heartbeat/s")
    for cond in ("idleA", "idleB", "1Hz", "3Hz", "5Hz", "10Hz", "20Hz"):
        if cond not in by:
            continue
        s = summarise(by[cond])
        _, p = mann_whitney(s["vals"], idle["vals"]) if not cond.startswith("idle") else (0, float("nan"))
        bl = [b for b in blocks if b["cond"] == cond]
        dl = statistics.mean(b["downlink_per_s"] for b in bl) if bl else float("nan")
        hb = statistics.mean(b["heartbeat_per_s"] for b in bl) if bl else float("nan")
        print(f"{cond:8s} {s['n']:3d} {s['loss']:6.0f} {s['median']:9.2f} {s['p90']:7.2f} {p:10.4f}   {dl:13.1f}  {hb:11.2f}")
    if "idleA" in by and "idleB" in by:
        _, p = mann_whitney(summarise(by["idleA"])["vals"], summarise(by["idleB"])["vals"])
        print(f"\ncontrol, idle A vs idle B (drift): p = {p:.3f}  (a small p here would mean the link drifts too much to trust the rest)")
    return 0


def exp2(runs_dir: str = "runs") -> int:
    groups: dict[str, list[dict]] = {}
    for f in sorted(Path(runs_dir).glob("*.json")):
        r = json.loads(f.read_text())
        name = (r.get("profile") or {}).get("name", "")
        if not name.upper().startswith("EXP2"):
            continue
        feed = r.get("feed") or []
        sent = next((x["t"] for x in feed if x["kind"] == "sent" and x["text"] == "ARM"), None)
        armed = next((x["t"] for x in feed if x["kind"] == "event" and x["data"].get("event") == "armed"), None)
        ready = next((x["t"] for x in feed if x["kind"] == "event" and x["data"].get("event") == "ready"), None)
        gcs = sum("ground station lost" in x.get("text", "") for x in feed if x["kind"] == "log")
        hz = (r.get("profile") or {}).get("setpoint_hz")
        key = f"{'20 Hz' if hz == 20 else '3 Hz' if hz == 3 else str(hz) + ' Hz'}"
        groups.setdefault(key, []).append(dict(run=r["id"], arm_s=None if sent is None or armed is None else armed - sent,
                                               sent=sent is not None, gcs=gcs,
                                               ready_s=None if ready is None else ready - (feed[0]["t"] if feed else ready)))
    if not groups:
        print("no runs named EXP2... found in", runs_dir)
        return 1
    allv = {}
    for k, rs in sorted(groups.items()):
        sent = [x for x in rs if x["sent"]]
        vals = [TIMEOUT_S if x["arm_s"] is None else x["arm_s"] for x in sent]
        allv[k] = vals
        ok = sum(x["arm_s"] is not None for x in sent)
        print(f"{k:6s} runs {len(rs):2d}  ARM sent {len(sent):2d}  armed {ok:2d}  "
              f"median {statistics.median(vals) if vals else float('nan'):6.1f} s  worst {max(vals) if vals else float('nan'):6.1f} s  "
              f"'ground station lost' per run {statistics.mean(x['gcs'] for x in rs):.1f}   times: {[round(v, 1) for v in vals]}")
    if "20 Hz" in allv and "3 Hz" in allv:
        _, p = mann_whitney(allv["20 Hz"], allv["3 Hz"])
        print(f"\n20 Hz vs 3 Hz time to arm (a failure counts as {TIMEOUT_S:.0f} s): Mann-Whitney p = {p:.3f}")
    return 0


def fisher_exact(a: int, b: int, c: int, d: int) -> float:
    """Two-sided Fisher exact p for [[a, b], [c, d]]."""
    n1, n2, k = a + b, c + d, a + c
    tot = math.comb(n1 + n2, k)
    prob = lambda x: math.comb(n1, x) * math.comb(n2, k - x) / tot          # noqa: E731
    p0 = prob(a)
    return min(1.0, sum(prob(x) for x in range(max(0, k - n2), min(n1, k) + 1) if prob(x) <= p0 + 1e-12))


def exp3(runs_dir: str = "runs") -> int:
    groups: dict[str, list[dict]] = {}
    for f in sorted(Path(runs_dir).glob("*.json")):
        r = json.loads(f.read_text())
        name = (r.get("profile") or {}).get("name", "")
        if not name.upper().startswith("EXP3"):
            continue
        res, hz = r.get("result") or {}, (r.get("profile") or {}).get("setpoint_hz")
        feed = r.get("feed") or []
        armed = any(x["kind"] == "event" and x["data"].get("event") == "armed" for x in feed)
        fs = sum("Failsafe activated" in x.get("text", "") for x in feed if x["kind"] == "log")
        src = res.get("takeover_source")
        texts = [(x["t"], x.get("text", "")) for x in feed if x["kind"] == "log"]
        t_armed = next((x["t"] for x in feed if x["kind"] == "event" and x["data"].get("event") == "armed"), None)
        t_fs = next((t for t, tx in texts if "Failsafe activated" in tx and t_armed and t >= t_armed), None)
        by_switch = any("Disarmed by RC switch" in tx or "kill" in tx.lower() and "Disarmed" in tx for _, tx in texts)
        # a failsafe in the armed window is a loss of Offboard even when the mode never dropped (20 Hz, 5 Nov)
        kind = ("not armed" if not armed else "void (pilot or kill)" if src and src != "link_loss" or by_switch
                else "bounce" if src == "link_loss" or t_fs else "survived" if res.get("outcome") == "no_takeover"
                else f"other:{res.get('outcome')}")
        t = round(t_fs - t_armed, 1) if t_fs and t_armed else None
        groups.setdefault(f"{hz:g} Hz" if hz else "?", []).append(dict(run=r["id"], kind=kind, t=t, fs=fs))
    if not groups:
        print("no runs named EXP3... found in", runs_dir)
        return 1
    counts = {}
    for k, rs in sorted(groups.items(), key=lambda kv: float(kv[0].split()[0]) if kv[0] != "?" else 0):
        valid = [x for x in rs if x["kind"] in ("survived", "bounce")]
        surv = sum(x["kind"] == "survived" for x in valid)
        counts[k] = (surv, len(valid) - surv)
        print(f"{k:6s} runs {len(rs):2d}  valid {len(valid):2d}  survived the hold {surv:2d}  Offboard lost {len(valid) - surv:2d}"
              f"  (bounce at {[x['t'] for x in valid if x['kind'] == 'bounce']} s after arming, first failsafe)  'Failsafe activated' per run "
              f"{statistics.mean(x['fs'] for x in rs):.1f}   other: {[x['kind'] for x in rs if x['kind'] not in ('survived', 'bounce')]}")
    ks = list(counts)
    for i in range(len(ks)):
        for j in range(i + 1, len(ks)):
            (a, b), (c, d) = counts[ks[i]], counts[ks[j]]
            if a + b and c + d:
                print(f"{ks[i]} vs {ks[j]}: Fisher exact p = {fisher_exact(a, b, c, d):.3f}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "exp3":
        raise SystemExit(exp3(sys.argv[2] if len(sys.argv) > 2 else "runs"))
    if len(sys.argv) >= 3 and sys.argv[1] == "exp1":
        raise SystemExit(exp1(sys.argv[2]))
    if len(sys.argv) >= 2 and sys.argv[1] == "exp2":
        raise SystemExit(exp2(sys.argv[2] if len(sys.argv) > 2 else "runs"))
    print(__doc__)
    raise SystemExit(2)
