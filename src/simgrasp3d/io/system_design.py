"""輸出系統設計工作台的可檢查 JSON 摘要。"""

from __future__ import annotations

import json
from pathlib import Path

from simgrasp3d.models.system_design import SystemDesignLabResult, SystemDesignSnapshot


def _snapshot_dict(snapshot: SystemDesignSnapshot) -> dict:
    return {
        "parameters": snapshot.values,
        "gates": [
            {
                "key": gate.key,
                "layer": gate.layer,
                "label": gate.label,
                "value": gate.value,
                "unit": gate.unit,
                "relation": gate.relation,
                "limit": gate.limit,
                "passed": gate.passed,
                "explanation": gate.explanation,
                "action": gate.action,
            }
            for gate in snapshot.gates
        ],
        "metrics": snapshot.metrics,
        "grasp_point_m": snapshot.grasp_point.tolist(),
        "goal_point_m": snapshot.goal_point.tolist(),
        "planned_path_m": snapshot.planned_path.tolist(),
        "camera_position_m": snapshot.camera_position.tolist(),
    }


def export_system_design_result(
    output_path: str | Path,
    result: SystemDesignLabResult,
) -> Path:
    """輸出基準與 preset 結果，方便 notebook 或版本差異檢查。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "simgrasp3d.system_design_result.v1",
        "name": result.spec.name,
        "seed": result.spec.seed,
        "simulation_only": True,
        "estimator": "geometric_system_design_screening",
        "baseline": _snapshot_dict(result.baseline),
        "presets": [
            {
                "name": preset.name,
                "description": preset.description,
                "result": _snapshot_dict(snapshot),
            }
            for preset, snapshot in result.preset_snapshots
        ],
    }
    destination.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return destination

