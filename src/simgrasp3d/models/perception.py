"""RGB-D 幾何分析、包圍盒與抓取候選資料模型。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class PerceptionSpec:
    """點雲幾何 baseline 的可重現設定。"""

    random_seed: int
    plane_ransac_iterations: int
    plane_distance_threshold_m: float
    maximum_table_tilt_deg: float
    minimum_object_points: int
    normal_neighbor_count: int
    maximum_normal_samples: int
    bounding_box_trim_quantile: float
    maximum_gripper_opening_m: float
    grasp_width_margin_m: float
    pregrasp_distance_m: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PerceptionSpec:
        spec = cls(
            random_seed=int(data.get("random_seed", 7515)),
            plane_ransac_iterations=int(data.get("plane_ransac_iterations", 300)),
            plane_distance_threshold_m=float(
                data.get("plane_distance_threshold_m", 0.008)
            ),
            maximum_table_tilt_deg=float(data.get("maximum_table_tilt_deg", 20.0)),
            minimum_object_points=int(data.get("minimum_object_points", 24)),
            normal_neighbor_count=int(data.get("normal_neighbor_count", 16)),
            maximum_normal_samples=int(data.get("maximum_normal_samples", 96)),
            bounding_box_trim_quantile=float(
                data.get("bounding_box_trim_quantile", 0.02)
            ),
            maximum_gripper_opening_m=float(
                data.get("maximum_gripper_opening_m", 0.11)
            ),
            grasp_width_margin_m=float(data.get("grasp_width_margin_m", 0.01)),
            pregrasp_distance_m=float(data.get("pregrasp_distance_m", 0.08)),
        )
        if min(
            spec.plane_ransac_iterations,
            spec.minimum_object_points,
            spec.normal_neighbor_count,
            spec.maximum_normal_samples,
        ) <= 0:
            raise ValueError("感知迭代、點數與鄰居數必須大於 0")
        if min(
            spec.plane_distance_threshold_m,
            spec.maximum_table_tilt_deg,
            spec.maximum_gripper_opening_m,
            spec.grasp_width_margin_m,
            spec.pregrasp_distance_m,
        ) <= 0.0:
            raise ValueError("感知距離、角度與夾爪尺寸必須大於 0")
        if spec.maximum_table_tilt_deg >= 90.0:
            raise ValueError("maximum_table_tilt_deg 必須小於 90 度")
        if not 0.0 <= spec.bounding_box_trim_quantile < 0.25:
            raise ValueError("bounding_box_trim_quantile 必須位於 0 到 0.25 之間")
        return spec


@dataclass(frozen=True)
class PlaneEstimate:
    """以 `normal · point + offset = 0` 表示的桌面平面。"""

    normal: np.ndarray
    offset: float
    inlier_mask: np.ndarray
    rms_error_m: float


@dataclass(frozen=True)
class BoundingBox3D:
    """AABB 或 OBB 的中心、完整邊長與局部旋轉。"""

    center: np.ndarray
    extents: np.ndarray
    rotation: np.ndarray

    def corners(self) -> np.ndarray:
        """依固定索引回傳八個世界座標角點。"""

        signs = np.asarray(
            [
                [-1, -1, -1],
                [1, -1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, -1, 1],
                [1, -1, 1],
                [1, 1, 1],
                [-1, 1, 1],
            ],
            dtype=np.float64,
        )
        local = signs * self.extents[None, :] / 2.0
        return local @ self.rotation.T + self.center


@dataclass(frozen=True)
class GraspCandidate:
    """一個可由後續 IK 與碰撞層驗證的六自由度抓取候選。"""

    object_name: str
    tcp_position: np.ndarray
    pregrasp_position: np.ndarray
    tcp_rotation: np.ndarray
    approach_direction: np.ndarray
    closing_axis: np.ndarray
    required_opening_m: float
    score: float
    geometry_feasible: bool


@dataclass(frozen=True)
class ObjectGeometry:
    """單一可見物件的點雲、法向、包圍盒與抓取候選。"""

    name: str
    points: np.ndarray
    colors: np.ndarray
    normal_points: np.ndarray
    normals: np.ndarray
    aabb: BoundingBox3D
    obb: BoundingBox3D
    grasp_candidates: tuple[GraspCandidate, ...]


@dataclass(frozen=True)
class PerceptionResult:
    """單張 RGB-D 的桌面、物件幾何與抓取分析結果。"""

    frame_id: str
    segmentation_mode: str
    table_plane: PlaneEstimate
    objects: tuple[ObjectGeometry, ...]
    grasp_candidates: tuple[GraspCandidate, ...]
    metrics: dict[str, float | int]
