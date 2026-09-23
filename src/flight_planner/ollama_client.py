from __future__ import annotations

import json
import urllib.request
from typing import Any

from .models import PlannerConfig, PlanningResult
from .rag import TemplateStore


def _prompt_query(config: PlannerConfig) -> str:
    return (
        f"{config.scan_type} scan pattern field briefing review checklist "
        "guardrail warnings drone count waypoint coverage"
    )


def _mission_summary(result: PlanningResult, config: PlannerConfig) -> dict[str, Any]:
    return {
        "scan_type": config.scan_type,
        "number_of_drones": config.number_of_drones,
        "scan_altitude_m": config.scan_altitude_m,
        "guardrail_passed": result.guardrail_passed,
        "guardrail_warnings": result.warnings,
        "ops_area": [coordinate.as_qgc()[:2][::-1] for coordinate in result.mission.ops_area.coordinates],
        "geo_fence": [coordinate.as_qgc()[:2][::-1] for coordinate in result.mission.geo_fence.coordinates],
        "drones": [{"drone_id": drone.drone_id, "waypoint_count": len(drone.waypoints)} for drone in result.drones],
    }


def build_review_prompt(result: PlanningResult, config: PlannerConfig, store: TemplateStore | None = None) -> str:
    """Compose the Gemma prompt asking it to review the already-planned, already-guardrailed mission (the RAG step)."""
    store = store or TemplateStore()
    chunks = store.retrieve(_prompt_query(config), top_k=4)
    template_text = "\n\n".join(f"### {chunk.heading}\n{chunk.text}" for chunk in chunks)
    summary = _mission_summary(result, config)
    return (
        "You are a QA reviewer producing a field briefing for a drone operator. The mission below was "
        "already planned and guardrail-checked by deterministic code. Do not invent, move, or second-guess "
        "any coordinate — your job is only to summarize the mission and flag anything the operator should "
        "double check.\n\n"
        f"{template_text}\n\n"
        f"Mission summary (JSON):\n{json.dumps(summary, separators=(',', ':'))}\n\n"
        "Write the field briefing now, following the format above."
    )


class OllamaReviewer:
    """Asks Gemma (via Ollama) to write a human-readable field briefing for an already-planned mission."""

    def __init__(self, config: PlannerConfig, store: TemplateStore | None = None):
        self.config = config
        self.store = store or TemplateStore()

    def review(self, result: PlanningResult) -> str:
        prompt = build_review_prompt(result, self.config, self.store)
        payload = json.dumps({"model": self.config.ollama_model, "prompt": prompt, "stream": False}).encode()
        request = urllib.request.Request(
            self.config.ollama_base_url.rstrip("/") + "/api/generate",
            data=payload,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=self.config.ollama_timeout_seconds) as response:
            response_data = json.load(response)
        text = response_data.get("response", "").strip()
        if not text:
            raise ValueError("Ollama returned an empty field briefing")
        return text
