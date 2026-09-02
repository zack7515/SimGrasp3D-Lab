"""匯出 MuJoCo baseline 軌跡與物理參數敏感度摘要。"""

from __future__ import annotations

import json
from pathlib import Path

from simgrasp3d.io.trajectory import export_trajectory
from simgrasp3d.models.physics import PhysicsSweepData


def export_physics_sweep(
    output_dir: str | Path,
    sweep: PhysicsSweepData,
) -> dict[str, Path]:
    """輸出物理軌跡 NPZ、逐幀摘要與可比較的 sweep JSON。"""

    destination = Path(output_dir)
    trajectory_paths = export_trajectory(destination, sweep.baseline)
    summary_path = destination / "sensitivity.json"
    payload = {
        "schema_version": "1.0",
        "engine": sweep.engine_version,
        "result_scope": "simulation_only",
        "contact_sampling": "trajectory_frame_rate",
        "cases": [
            {
                "name": case.name,
                "parameters": case.parameters,
                "metrics": case.metrics,
            }
            for case in sweep.cases
        ],
    }
    summary_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        **trajectory_paths,
        "sensitivity": summary_path,
    }
