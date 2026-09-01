"""將模擬點雲匯出成通用 ASCII PLY。"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

from simgrasp3d.geometry.sampling import PointCloud


def _safe_name(name: str) -> str:
    result = re.sub(r"[^a-zA-Z0-9_.-]+", "_", name).strip("._")
    return result or "point_cloud"


def write_ply(
    path: str | Path,
    point_cloud: PointCloud,
    point_colors: np.ndarray | None = None,
) -> Path:
    """以公尺座標及 8-bit RGB 寫入 ASCII PLY。"""

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if point_colors is None:
        point_colors = np.repeat(
            np.asarray(point_cloud.color, dtype=np.float64)[None, :],
            point_cloud.points.shape[0],
            axis=0,
        )
    if point_colors.shape != point_cloud.points.shape:
        raise ValueError("point_colors 必須與 points 同為 N×3")
    rgb = np.clip(point_colors * 255.0, 0.0, 255.0).astype(np.uint8)

    header = "\n".join(
        [
            "ply",
            "format ascii 1.0",
            "comment SimGrasp3D Lab simulation point cloud; unit=meter",
            f"element vertex {point_cloud.points.shape[0]}",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "end_header",
        ]
    )
    with output_path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write(header)
        stream.write("\n")
        for (x_value, y_value, z_value), color in zip(point_cloud.points, rgb, strict=True):
            stream.write(
                f"{x_value:.8f} {y_value:.8f} {z_value:.8f} "
                f"{color[0]} {color[1]} {color[2]}\n"
            )
    return output_path


def export_scene_point_clouds(
    output_dir: str | Path,
    point_clouds: tuple[PointCloud, ...],
) -> tuple[Path, ...]:
    """逐一匯出場景實體，並額外建立一份合併點雲。"""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    for cloud in point_clouds:
        paths.append(write_ply(destination / f"{_safe_name(cloud.name)}.ply", cloud))

    combined_points = np.concatenate([cloud.points for cloud in point_clouds], axis=0)
    combined_colors = np.concatenate(
        [
            np.repeat(np.asarray(cloud.color)[None, :], cloud.points.shape[0], axis=0)
            for cloud in point_clouds
        ],
        axis=0,
    )
    combined = PointCloud("complete_scene", combined_points, (0.72, 0.72, 0.72), "scene")
    paths.append(write_ply(destination / "complete_scene.ply", combined, combined_colors))
    return tuple(paths)
