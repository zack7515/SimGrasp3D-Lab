"""匯出 fail-closed 控制重播與安全閘門摘要。"""

from __future__ import annotations

import json
from pathlib import Path

from simgrasp3d.models.integration import ReplayEvent, ReplayResult


def _event_payload(event: ReplayEvent) -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "sequence": event.sequence,
        "time_s": event.time_s,
        "state": event.state,
        "event": event.event,
        "payload": event.payload,
    }


def export_replay_result(
    output_dir: str | Path,
    result: ReplayResult,
) -> dict[str, Path]:
    """輸出逐行 JSON 事件與可快速檢查的 summary JSON。"""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    event_path = destination / "replay.jsonl"
    event_path.write_text(
        "".join(
            json.dumps(
                _event_payload(event),
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
            )
            + "\n"
            for event in result.events
        ),
        encoding="utf-8",
    )
    summary_path = destination / "summary.json"
    summary = {
        "schema_version": "1.0",
        "result_scope": "simulation_only",
        "execution_authorized": result.execution_authorized,
        "failure_codes": result.failure_codes,
        "selected_grasp_object": (
            None
            if result.selected_grasp is None
            else result.selected_grasp.candidate.object_name
        ),
        "metrics": result.metrics,
    }
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {"replay": event_path, "summary": summary_path}
