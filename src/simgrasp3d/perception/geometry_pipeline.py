"""從 RGB-D observation 建立桌面平面、物件 OBB 與抓取候選。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from simgrasp3d.models.perception import (
    BoundingBox3D,
    GraspCandidate,
    ObjectGeometry,
    PerceptionResult,
    PerceptionSpec,
    PlaneEstimate,
)
from simgrasp3d.sensors.rgbd import RGBDFrame


def load_perception_spec(path: str | Path) -> PerceptionSpec:
    """讀取並驗證 RGB-D 幾何分析設定。"""

    with Path(path).open("r", encoding="utf-8") as stream:
        return PerceptionSpec.from_dict(json.load(stream))


def _fit_table_plane(points: np.ndarray, spec: PerceptionSpec) -> PlaneEstimate:
    """以限定接近水平的 RANSAC 找出主要桌面平面。"""

    if len(points) < 3:
        raise ValueError("有效點不足，無法估計桌面")
    rng = np.random.default_rng(spec.random_seed)
    minimum_vertical_component = float(
        np.cos(np.deg2rad(spec.maximum_table_tilt_deg))
    )
    best_mask = np.zeros(len(points), dtype=bool)
    best_error = float("inf")
    for _ in range(spec.plane_ransac_iterations):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-10:
            continue
        normal /= norm
        if abs(float(normal[2])) < minimum_vertical_component:
            continue
        if normal[2] < 0.0:
            normal *= -1.0
        offset = -float(np.dot(normal, sample[0]))
        distances = np.abs(points @ normal + offset)
        mask = distances <= spec.plane_distance_threshold_m
        count = int(np.count_nonzero(mask))
        error = float(np.mean(distances[mask])) if count else float("inf")
        if count > int(np.count_nonzero(best_mask)) or (
            count == int(np.count_nonzero(best_mask)) and error < best_error
        ):
            best_mask = mask
            best_error = error
    if np.count_nonzero(best_mask) < 3:
        raise RuntimeError("RANSAC 找不到符合傾角限制的桌面")

    inliers = points[best_mask]
    centroid = inliers.mean(axis=0)
    _, _, right_vectors = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = right_vectors[-1]
    if normal[2] < 0.0:
        normal *= -1.0
    offset = -float(np.dot(normal, centroid))
    distances = np.abs(points @ normal + offset)
    refined_mask = distances <= spec.plane_distance_threshold_m
    rms_error = float(
        np.sqrt(np.mean(np.square(points[refined_mask] @ normal + offset)))
    )
    return PlaneEstimate(
        normal=normal,
        offset=offset,
        inlier_mask=refined_mask,
        rms_error_m=rms_error,
    )


def _aabb(points: np.ndarray) -> BoundingBox3D:
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    return BoundingBox3D(
        center=(minimum + maximum) / 2.0,
        extents=maximum - minimum,
        rotation=np.eye(3, dtype=np.float64),
    )


def _obb(points: np.ndarray, trim_quantile: float) -> BoundingBox3D:
    """以 PCA 主軸近似可見點雲的 oriented bounding box。"""

    centroid = points.mean(axis=0)
    covariance = np.cov(points - centroid, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    rotation = eigenvectors[:, np.argsort(eigenvalues)[::-1]]
    if np.linalg.det(rotation) < 0.0:
        rotation[:, -1] *= -1.0
    local = (points - centroid) @ rotation
    minimum = np.quantile(local, trim_quantile, axis=0)
    maximum = np.quantile(local, 1.0 - trim_quantile, axis=0)
    center = centroid + ((minimum + maximum) / 2.0) @ rotation.T
    return BoundingBox3D(
        center=center,
        extents=maximum - minimum,
        rotation=rotation,
    )


def _surface_normals(
    points: np.ndarray,
    camera_position: np.ndarray,
    spec: PerceptionSpec,
) -> tuple[np.ndarray, np.ndarray]:
    """以局部 PCA 計算法向，僅抽樣視覺化所需數量。"""

    sample_count = min(len(points), spec.maximum_normal_samples)
    sample_indices = np.linspace(0, len(points) - 1, sample_count, dtype=np.int64)
    normal_points = points[sample_indices]
    normals: list[np.ndarray] = []
    neighbor_count = min(spec.normal_neighbor_count, len(points))
    for point in normal_points:
        squared_distances = np.sum(np.square(points - point), axis=1)
        neighbor_indices = np.argpartition(
            squared_distances,
            neighbor_count - 1,
        )[:neighbor_count]
        neighbors = points[neighbor_indices]
        covariance = np.cov(neighbors - neighbors.mean(axis=0), rowvar=False)
        _, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, 0]
        if float(np.dot(normal, camera_position - point)) < 0.0:
            normal *= -1.0
        normals.append(normal)
    return normal_points, np.asarray(normals, dtype=np.float64)


def _grasp_candidates(
    name: str,
    box: BoundingBox3D,
    point_count: int,
    spec: PerceptionSpec,
) -> tuple[GraspCandidate, ...]:
    """由 OBB 水平軸建立兩個可解釋的 top-down 平行夾爪候選。"""

    world_up = np.asarray([0.0, 0.0, 1.0])
    vertical_index = int(np.argmax(np.abs(box.rotation.T @ world_up)))
    horizontal_indices = [index for index in range(3) if index != vertical_index]
    approach = np.asarray([0.0, 0.0, -1.0])
    candidates: list[GraspCandidate] = []
    for axis_index in horizontal_indices:
        closing_axis = box.rotation[:, axis_index].copy()
        closing_axis[2] = 0.0
        closing_norm = float(np.linalg.norm(closing_axis))
        if closing_norm <= 1e-10:
            continue
        closing_axis /= closing_norm
        side_axis = np.cross(approach, closing_axis)
        side_axis /= np.linalg.norm(side_axis)
        rotation = np.column_stack((approach, closing_axis, side_axis))
        required_opening = float(box.extents[axis_index] + spec.grasp_width_margin_m)
        feasible = required_opening <= spec.maximum_gripper_opening_m
        width_score = max(0.0, 1.0 - required_opening / spec.maximum_gripper_opening_m)
        support_score = min(1.0, point_count / 200.0)
        score = 0.7 * width_score + 0.3 * support_score
        candidates.append(
            GraspCandidate(
                object_name=name,
                tcp_position=box.center.copy(),
                pregrasp_position=(
                    box.center - approach * spec.pregrasp_distance_m
                ),
                tcp_rotation=rotation,
                approach_direction=approach.copy(),
                closing_axis=closing_axis,
                required_opening_m=required_opening,
                score=score,
                geometry_feasible=feasible,
            )
        )
    return tuple(sorted(candidates, key=lambda item: item.score, reverse=True))


def analyze_rgbd_geometry(
    frame: RGBDFrame,
    object_names: tuple[str, ...],
    spec: PerceptionSpec,
    expected_table_top_z: float,
) -> PerceptionResult:
    """分析 observation；instance mask 僅作為第一版 oracle 物件分割。"""

    points = frame.world_points()
    colors = frame.point_colors()
    instance_ids = frame.instance_mask[frame.valid_mask]
    table_plane = _fit_table_plane(points, spec)
    camera_position = frame.camera_to_world[:3, 3]
    geometries: list[ObjectGeometry] = []
    all_candidates: list[GraspCandidate] = []
    for name in object_names:
        if name not in frame.instance_names:
            continue
        instance_id = frame.instance_names.index(name)
        mask = instance_ids == instance_id
        object_points = points[mask]
        if len(object_points) < spec.minimum_object_points:
            continue
        object_colors = colors[mask]
        normal_points, normals = _surface_normals(
            object_points,
            camera_position,
            spec,
        )
        oriented_box = _obb(object_points, spec.bounding_box_trim_quantile)
        candidates = _grasp_candidates(
            name,
            oriented_box,
            len(object_points),
            spec,
        )
        geometries.append(
            ObjectGeometry(
                name=name,
                points=object_points,
                colors=object_colors,
                normal_points=normal_points,
                normals=normals,
                aabb=_aabb(object_points),
                obb=oriented_box,
                grasp_candidates=candidates,
            )
        )
        all_candidates.extend(candidates)

    all_candidates.sort(key=lambda item: item.score, reverse=True)
    table_height = -table_plane.offset / table_plane.normal[2]
    tilt_deg = float(
        np.rad2deg(
            np.arccos(np.clip(table_plane.normal[2], -1.0, 1.0))
        )
    )
    metrics: dict[str, float | int] = {
        "valid_point_count": len(points),
        "table_inlier_count": int(np.count_nonzero(table_plane.inlier_mask)),
        "table_inlier_ratio": float(np.mean(table_plane.inlier_mask)),
        "table_plane_rms_error_m": table_plane.rms_error_m,
        "table_tilt_error_deg": tilt_deg,
        "table_height_error_m": float(table_height - expected_table_top_z),
        "detected_object_count": len(geometries),
        "grasp_candidate_count": len(all_candidates),
        "feasible_grasp_candidate_count": sum(
            candidate.geometry_feasible for candidate in all_candidates
        ),
    }
    return PerceptionResult(
        frame_id=frame.frame_id,
        segmentation_mode="oracle_instance_mask+geometric_ransac",
        table_plane=table_plane,
        objects=tuple(geometries),
        grasp_candidates=tuple(all_candidates),
        metrics=metrics,
    )
