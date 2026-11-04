import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import experiment_report as er  # noqa: E402


def test_mann_whitney_finds_a_clear_difference_and_ignores_none():
    low, high = [0.9, 1.0, 1.1, 1.2, 0.8, 1.0, 0.9, 1.1], [5.0, 6.0, 15.0, 7.0, 15.0, 5.5, 6.5, 15.0]
    assert er.mann_whitney(high, low)[1] < 0.01
    assert er.mann_whitney(low, [x + 0.01 for x in low])[1] > 0.3          # nearly the same sample: no effect
    assert er.mann_whitney(low, low)[1] == 1.0 or er.mann_whitney(low, low)[1] > 0.9


def test_a_lost_command_counts_as_the_timeout():
    s = er.summarise([dict(rtt=1.0), dict(rtt=None), dict(rtt=2.0)])
    assert s["loss"] == 100 / 3 and s["p90"] == 15.0 and s["median"] == 2.0


def test_exp1_prints_a_table_with_the_idle_control(tmp_path, capsys):
    rows = []
    for rnd in range(3):
        for cond, base in (("idleA", 1.0), ("idleB", 1.1), ("20Hz", 8.0)):
            rows += [dict(round=rnd, cond=cond, hz=0, i=i, rtt=None if cond == "20Hz" and i % 2 else base + i * 0.05) for i in range(6)]
            rows.append(dict(round=rnd, cond=cond, hz=0, block_s=30.0, downlink_per_s=23.0, heartbeat_per_s=1.0))
    f = tmp_path / "e.jsonl"
    f.write_text("\n".join(json.dumps(r) for r in rows))
    assert er.exp1(str(f)) == 0
    out = capsys.readouterr().out
    assert "20Hz" in out and "control, idle A vs idle B" in out


def test_exp2_groups_runs_by_setpoint_rate_and_times_the_arm(tmp_path, capsys):
    def run(rid, hz, sent_t, armed_t):
        feed = [dict(n=0, t=100.0, kind="log", text="[link] x"), dict(n=1, t=sent_t, kind="sent", text="ARM")]
        if armed_t:
            feed.append(dict(n=2, t=armed_t, kind="event", data=dict(event="armed")))
        (tmp_path / f"{rid}.json").write_text(json.dumps(dict(id=rid, profile=dict(name=f"EXP2-{hz}", setpoint_hz=hz), feed=feed)))
    for i, hz in enumerate((20, 3, 20, 3, 20, 3)):
        run(f"r{i}", hz, 110.0, None if hz == 20 and i == 0 else 110.0 + (12.0 if hz == 20 else 2.0))
    assert er.exp2(str(tmp_path)) == 0
    out = capsys.readouterr().out
    assert "20 Hz" in out and "3 Hz" in out and "Mann-Whitney" in out


def test_fisher_exact_matches_a_known_table():
    assert abs(er.fisher_exact(3, 1, 1, 3) - 0.4857142857) < 1e-6          # the textbook tea-tasting table
    assert er.fisher_exact(4, 0, 0, 4) < 0.05 and abs(er.fisher_exact(2, 2, 2, 2) - 1.0) < 1e-9


def test_exp3_separates_survivors_bounces_and_void_runs(tmp_path, capsys):
    def run(rid, hz, source, outcome, armed=True):
        feed = [dict(n=0, t=1.0, kind="log", text="[link]")]
        if armed:
            feed.append(dict(n=1, t=2.0, kind="event", data=dict(event="armed")))
        (tmp_path / f"{rid}.json").write_text(json.dumps(dict(id=rid, profile=dict(name=f"EXP3-{hz}", setpoint_hz=hz), feed=feed,
                                                              result=dict(outcome=outcome, takeover_source=source, takeover_after_s=3.7 if source else None))))
    for i in range(4):
        run(f"a{i}", 3, None, "no_takeover")
    for i in range(4):
        run(f"b{i}", 20, "link_loss" if i < 3 else None, "takeover" if i < 3 else "no_takeover")
    run("v", 20, "pilot_switch", "takeover")
    run("n", 3, None, None, armed=False)
    assert er.exp3(str(tmp_path)) == 0
    out = capsys.readouterr().out
    assert "survived the hold  4" in out and "Offboard lost  3" in out and "void (pilot or kill)" in out and "Fisher exact" in out
