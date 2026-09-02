"""匯出 RGB-D 幾何分析、抓取候選與物件可見點雲。"""

from __future__ import annotations

import json
from pathlib import Path

from simgrasp3d.geometry.sampling import PointCloud
from simgrasp3d.io.point_cloud import write_ply
from simgrasp3d.models.perception import BoundingBox3D, GraspCandidate, PerceptionResult


def _box_payload(box: BoundingBox3D) -> dict[str, list[float] | list[list[float]]]:
    return {
        "center": box.center.tolist(),
        "extents": box.extents.tolist(),
        "rotation": box.rotation.tolist(),
    }


def _grasp_payload(candidate: GraspCandidate) -> dict[str, object]:
    return {
        "object_name": candidate.object_name,
        "tcp_position": candidate.tcp_position.tolist(),
        "pregrasp_position": candidate.pregrasp_position.tolist(),
        "tcp_rotation": candidate.tcp_rotation.tolist(),
        "approach_direction": candidate.approach_direction.tolist(),
        "closing_axis": candidate.closing_axis.tolist(),
        "required_opening_m": candidate.required_opening_m,
        "score": candidate.score,
        "geometry_feasible": candidate.geometry_feasible,
    }


def export_perception_result(
    output_dir: str | Path,
    result: PerceptionResult,
) -> dict[str, Path]:
    """輸出 JSON 結果與每個物件的 observation-only PLY。"""

    destination = Path(output_dir)
    object_directory = destination / "objects"
    object_directory.mkdir(parents=True, exist_ok=True)
    object_paths: list[Path] = []
    objects_payload = []
    for geometry in result.objects:
        color = tuple(float(value) for value in geometry.colors.mean(axis=0))
        object_paths.append(
            write_ply(
                object_directory / f"{geometry.name}.ply",
                PointCloud(
                    name=geometry.name,
                    points=geometry.points,
                    color=color,
                    category="observation_object",
                ),
            )
        )
        objects_payload.append(
            {
                "name": geometry.name,
                "visible_point_count": len(geometry.points),
                "aabb": _box_payload(geometry.aabb),
                "obb": _box_payload(geometry.obb),
                "grasp_candidates": [
                    _grasp_payload(candidate)
                    for candidate in geometry.grasp_candidates
                ],
            }
        )
    result_path = destination / "geometry.json"
    payload = {
        "schema_version": "1.0",
        "frame_id": result.frame_id,
        "segmentation_mode": result.segmentation_mode,
        "table_plane": {
            "normal": result.table_plane.normal.tolist(),
            "offset": result.table_plane.offset,
            "inlier_count": int(result.table_plane.inlier_mask.sum()),
            "rms_error_m": result.table_plane.rms_error_m,
        },
        "objects": objects_payload,
        "ranked_grasp_candidates": [
            _grasp_payload(candidate) for candidate in result.grasp_candidates
        ],
        "metrics": result.metrics,
    }
    result_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        "geometry": result_path,
        **{
            f"object_{index}": path
            for index, path in enumerate(object_paths, start=1)
        },
    }
