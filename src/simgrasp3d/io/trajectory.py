"""匯出可供重播、分析與後續物理引擎交換的軟管軌跡。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from simgrasp3d.models.motion import TrajectoryData


MOTION_SCHEMA_VERSION = "1.0"


def write_trajectory_npz(path: str | Path, trajectory: TrajectoryData) -> Path:
    """將所有逐幀陣列寫入不使用 pickle 的壓縮 NPZ。"""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    frames = trajectory.frames
    np.savez_compressed(
        destination,
        schema_version=np.asarray(MOTION_SCHEMA_VERSION),
        scenario_name=np.asarray(trajectory.spec.name),
        time_s=np.asarray([frame.time_s for frame in frames], dtype=np.float64),
        phase=np.asarray([frame.phase for frame in frames], dtype=np.str_),
        tcp_position=np.stack([frame.tcp_position for frame in frames]),
        gripper_opening_m=np.asarray(
            [frame.gripper_opening_m for frame in frames], dtype=np.float64
        ),
        attached=np.asarray([frame.attached for frame in frames], dtype=np.bool_),
        hose_nodes=np.stack([frame.hose_nodes for frame in frames]),
        robot_joint_positions=np.stack(
            [frame.robot_joint_positions for frame in frames]
        ),
        joint_angles_deg=np.stack([frame.joint_angles_deg for frame in frames]),
        minimum_clearance_m=np.asarray(
            [frame.minimum_clearance_m for frame in frames], dtype=np.float64
        ),
        hose_clearance_m=np.asarray(
            [frame.hose_clearance_m for frame in frames], dtype=np.float64
        ),
        tool_clearance_m=np.asarray(
            [frame.tool_clearance_m for frame in frames], dtype=np.float64
        ),
        collision=np.asarray([frame.collision for frame in frames], dtype=np.bool_),
        ik_position_error_m=np.asarray(
            [frame.ik_position_error_m for frame in frames], dtype=np.float64
        ),
        hose_length_ratio=np.asarray(
            [frame.hose_length_ratio for frame in frames], dtype=np.float64
        ),
    )
    return destination


def export_trajectory(output_dir: str | Path, trajectory: TrajectoryData) -> dict[str, Path]:
    """匯出逐幀 NPZ 與人類可讀的摘要 JSON。"""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    trajectory_path = write_trajectory_npz(destination / "trajectory.npz", trajectory)
    metadata_path = destination / "metrics.json"
    metadata = {
        "schema_version": MOTION_SCHEMA_VERSION,
        "scenario_name": trajectory.spec.name,
        "length_unit": "meter",
        "time_unit": "second",
        "solver": "kinematic_position_constraints",
        "physics_engine": None,
        "safe_clearance_m": trajectory.spec.safe_clearance_m,
        "collision_tolerance_m": trajectory.spec.collision_tolerance_m,
        "metrics": trajectory.metrics,
    }
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {"trajectory": trajectory_path, "metrics": metadata_path}
