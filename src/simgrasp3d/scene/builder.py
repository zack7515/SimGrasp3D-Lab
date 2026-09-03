"""從 JSON 設定建立完整點雲、機械手與相機視錐。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simgrasp3d.geometry.sampling import (
    PointCloud,
    sample_box,
    sample_cylinder,
    sample_sphere,
)
from simgrasp3d.geometry.transforms import pose_matrix, transform_points
from simgrasp3d.io import load_spec
from simgrasp3d.models.specs import CameraSpec, ObjectSpec, SceneSpec
from simgrasp3d.robot.kinematics import RobotState, build_robot_state


@dataclass(frozen=True)
class SceneData:
    """可供匯出與視覺化的完整場景資料。"""

    spec: SceneSpec
    point_clouds: tuple[PointCloud, ...]
    robot_state: RobotState
    frames: dict[str, np.ndarray]
    camera_segments: np.ndarray


def load_scene_spec(path: str | Path) -> SceneSpec:
    """讀取並驗證 UTF-8 JSON 場景設定。"""

    return load_spec(path, SceneSpec)


def _sample_object(spec: ObjectSpec, rng: np.random.Generator) -> PointCloud:
    if spec.shape == "box":
        box_size = (spec.dimensions[0], spec.dimensions[1], spec.dimensions[2])
        local_points = sample_box(box_size, spec.point_count, rng)
    elif spec.shape == "cylinder":
        local_points = sample_cylinder(spec.dimensions[0], spec.dimensions[1], spec.point_count, rng)
    elif spec.shape == "sphere":
        local_points = sample_sphere(spec.dimensions[0], spec.point_count, rng)
    else:
        raise ValueError(f"不支援的物件形狀：{spec.shape}")

    world_points = transform_points(local_points, pose_matrix(spec.pose.xyz, spec.pose.rpy_deg))
    return PointCloud(spec.name, world_points, spec.color, "object")


def _camera_basis(camera: CameraSpec) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = np.asarray(camera.position, dtype=np.float64)
    target = np.asarray(camera.look_at, dtype=np.float64)
    up_hint = np.asarray(camera.up, dtype=np.float64)
    forward = target - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up_hint)
    if np.linalg.norm(right) <= 1e-9:
        raise ValueError("camera.up 不可與視線方向平行")
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)
    return right, up, forward


def camera_frame(camera: CameraSpec) -> np.ndarray:
    """建立以 x 向右、y 向上、z 向前的相機視覺座標系。"""

    right, up, forward = _camera_basis(camera)
    frame = np.eye(4, dtype=np.float64)
    frame[:3, :3] = np.column_stack((right, up, forward))
    frame[:3, 3] = np.asarray(camera.position, dtype=np.float64)
    return frame


def camera_frustum_segments(camera: CameraSpec) -> np.ndarray:
    """計算相機近遠平面及視錐邊線。"""

    position = np.asarray(camera.position, dtype=np.float64)
    right, up, forward = _camera_basis(camera)
    half_angle = np.deg2rad(camera.vertical_fov_deg) / 2.0

    planes: list[np.ndarray] = []
    for distance in (camera.near, camera.far):
        center = position + forward * distance
        half_height = np.tan(half_angle) * distance
        half_width = half_height * camera.aspect_ratio
        planes.append(
            np.asarray(
                [
                    center - right * half_width - up * half_height,
                    center + right * half_width - up * half_height,
                    center + right * half_width + up * half_height,
                    center - right * half_width + up * half_height,
                ],
                dtype=np.float64,
            )
        )

    near_corners, far_corners = planes
    segments: list[np.ndarray] = []
    for corners in (near_corners, far_corners):
        for index in range(4):
            segments.append(np.asarray([corners[index], corners[(index + 1) % 4]]))
    for index in range(4):
        segments.append(np.asarray([position, far_corners[index]]))
    return np.asarray(segments, dtype=np.float64)


def build_scene(spec: SceneSpec) -> SceneData:
    """依固定 seed 產生可重現的場景點雲。"""

    rng = np.random.default_rng(spec.seed)
    table_transform = pose_matrix(spec.table.pose.xyz, spec.table.pose.rpy_deg)
    table_points = transform_points(
        sample_box(spec.table.size, spec.table.point_count, rng),
        table_transform,
    )
    clouds: list[PointCloud] = [
        PointCloud("table", table_points, spec.table.color, "environment")
    ]
    clouds.extend(_sample_object(item, rng) for item in spec.objects)

    robot_state = build_robot_state(spec.robot, rng)
    clouds.extend(robot_state.point_clouds)
    frames: dict[str, np.ndarray] = {
        "world": np.eye(4, dtype=np.float64),
        "table": table_transform,
        "camera": camera_frame(spec.camera),
        **robot_state.joint_frames,
    }
    for item in spec.objects:
        frames[item.name] = pose_matrix(item.pose.xyz, item.pose.rpy_deg)

    return SceneData(
        spec=spec,
        point_clouds=tuple(clouds),
        robot_state=robot_state,
        frames=frames,
        camera_segments=camera_frustum_segments(spec.camera),
    )
