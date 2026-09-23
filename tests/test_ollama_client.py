import io
import json
from pathlib import Path

import pytest

from flight_planner import ollama_client
from flight_planner.agents import MultiAgentPlanner
from flight_planner.config import load_config
from flight_planner.kml_parser import parse_kml
from flight_planner.ollama_client import OllamaReviewer, build_review_prompt

ROOT = Path(__file__).parents[1]


def _planned_result(scan_type="lawnmower", number_of_drones=3):
    from dataclasses import replace

    mission = parse_kml(ROOT / "ops" / "SAFTI city.kml")
    config = replace(load_config(ROOT / "config.example.json"), scan_type=scan_type, number_of_drones=number_of_drones)
    return MultiAgentPlanner().run(mission, config), config


def test_build_review_prompt_never_asks_gemma_to_invent_coordinates():
    result, config = _planned_result()
    prompt = build_review_prompt(result, config)
    assert "already planned and guardrail-checked" in prompt
    assert "do not invent" in prompt.lower()


def test_build_review_prompt_includes_mission_summary_facts():
    result, config = _planned_result(number_of_drones=4)
    prompt = build_review_prompt(result, config)
    assert '"number_of_drones":4' in prompt
    assert '"guardrail_passed":true' in prompt
    assert f'"scan_type":"{config.scan_type}"' in prompt


class _FakeResponse:
    def __init__(self, payload: dict):
        self._data = json.dumps(payload).encode()

    def __enter__(self):
        return io.BytesIO(self._data)

    def __exit__(self, *exc_info):
        return False


def test_review_returns_the_briefing_text(monkeypatch):
    result, config = _planned_result()

    def fake_urlopen(request, timeout=None):
        return _FakeResponse({"response": "3 drones, lawnmower scan, all waypoints inside the fence."})

    monkeypatch.setattr(ollama_client.urllib.request, "urlopen", fake_urlopen)
    briefing = OllamaReviewer(config).review(result)
    assert briefing == "3 drones, lawnmower scan, all waypoints inside the fence."


def test_review_rejects_an_empty_briefing(monkeypatch):
    result, config = _planned_result()

    def fake_urlopen(request, timeout=None):
        return _FakeResponse({"response": "   "})

    monkeypatch.setattr(ollama_client.urllib.request, "urlopen", fake_urlopen)
    with pytest.raises(ValueError):
        OllamaReviewer(config).review(result)
