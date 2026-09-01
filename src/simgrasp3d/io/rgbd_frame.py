"""儲存與讀取 SimGrasp3D RGB-D frame。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from simgrasp3d.geometry.sampling import PointCloud
from simgrasp3d.io.point_cloud import write_ply
from simgrasp3d.sensors.rgbd import RGBDFrame, RGBDSimulationResult, SCHEMA_VERSION


def write_rgbd_frame(path: str | Path, frame: RGBDFrame) -> Path:
    """以不依賴 pickle 的壓縮 NPZ 儲存一張 RGB-D frame。"""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        destination,
        schema_version=np.asarray(SCHEMA_VERSION),
        frame_id=np.asarray(frame.frame_id),
        rgb=frame.rgb,
        depth_m=frame.depth_m,
        instance_mask=frame.instance_mask,
        intrinsics=frame.intrinsics,
        camera_to_world=frame.camera_to_world,
        instance_names=np.asarray(frame.instance_names, dtype=np.str_),
    )
    return destination


def read_rgbd_frame(path: str | Path) -> RGBDFrame:
    """讀取由 :func:`write_rgbd_frame` 建立的 RGB-D frame。"""

    with np.load(Path(path), allow_pickle=False) as data:
        version = str(data["schema_version"].item())
        if version != SCHEMA_VERSION:
            raise ValueError(f"不支援的 RGB-D schema version：{version}")
        return RGBDFrame(
            rgb=data["rgb"].copy(),
            depth_m=data["depth_m"].copy(),
            instance_mask=data["instance_mask"].copy(),
            intrinsics=data["intrinsics"].copy(),
            camera_to_world=data["camera_to_world"].copy(),
            instance_names=tuple(str(value) for value in data["instance_names"]),
            frame_id=str(data["frame_id"].item()),
        )


def write_rgbd_point_cloud(path: str | Path, frame: RGBDFrame) -> Path:
    """將 frame 中的有效深度匯出為世界座標彩色 PLY。"""

    cloud = PointCloud(
        name=frame.frame_id,
        points=frame.world_points(),
        color=(0.7, 0.7, 0.7),
        category="sensor_observation",
    )
    return write_ply(path, cloud, frame.point_colors())


def export_rgbd_simulation(
    output_dir: str | Path,
    result: RGBDSimulationResult,
) -> dict[str, Path]:
    """匯出理想 frame、觀測 frame、可見點雲與比較指標。"""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    paths = {
        "ground_truth_frame": write_rgbd_frame(
            destination / "ground_truth_frame.npz",
            result.ground_truth,
        ),
        "observation_frame": write_rgbd_frame(
            destination / "observation_frame.npz",
            result.observation,
        ),
        "ground_truth_cloud": write_rgbd_point_cloud(
            destination / "ground_truth_visible.ply",
            result.ground_truth,
        ),
        "observation_cloud": write_rgbd_point_cloud(
            destination / "observation_visible.ply",
            result.observation,
        ),
    }
    metadata_path = destination / "metrics.json"
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "depth_unit": "meter",
        "invalid_depth_value": 0.0,
        "camera_convention": "OpenCV optical: x-right, y-down, z-forward",
        "observation_camera_to_world_is_nominal": True,
        "translation_error_m": result.translation_error_m.tolist(),
        "rotation_error_deg": result.rotation_error_deg.tolist(),
        "actual_camera_to_world": result.actual_camera_to_world.tolist(),
        "metrics": result.metrics,
    }
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    paths["metrics"] = metadata_path
    return paths
