"""以 pinhole 相機、z-buffer 與簡化雜訊產生 RGB-D 觀測。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simgrasp3d.geometry.sampling import PointCloud
from simgrasp3d.geometry.transforms import pose_matrix
from simgrasp3d.models.specs import CameraSpec, SensorNoiseSpec
from simgrasp3d.scene.builder import SceneData, camera_frame


SCHEMA_VERSION = "1.0"


@dataclass(frozen=True)
class RGBDFrame:
    """可由模擬與真實資料轉接器共用的單張 RGB-D 資料。"""

    rgb: np.ndarray
    depth_m: np.ndarray
    instance_mask: np.ndarray
    intrinsics: np.ndarray
    camera_to_world: np.ndarray
    instance_names: tuple[str, ...]
    frame_id: str

    def __post_init__(self) -> None:
        height, width = self.depth_m.shape
        if self.rgb.dtype != np.uint8:
            raise ValueError("rgb dtype 必須是 uint8")
        if self.depth_m.dtype != np.float32:
            raise ValueError("depth_m dtype 必須是 float32")
        if self.instance_mask.dtype != np.uint16:
            raise ValueError("instance_mask dtype 必須是 uint16")
        if self.rgb.shape != (height, width, 3):
            raise ValueError("rgb 必須為 H×W×3")
        if self.instance_mask.shape != (height, width):
            raise ValueError("instance_mask 必須與 depth_m 尺寸一致")
        if self.intrinsics.shape != (3, 3):
            raise ValueError("intrinsics 必須為 3×3")
        if self.camera_to_world.shape != (4, 4):
            raise ValueError("camera_to_world 必須為 4×4")
        if not self.instance_names or self.instance_names[0] != "background":
            raise ValueError("instance_names 的第 0 項必須是 background")
        if np.any(self.depth_m < 0.0):
            raise ValueError("depth_m 不可包含負值")
        if int(self.instance_mask.max(initial=0)) >= len(self.instance_names):
            raise ValueError("instance_mask 含有未定義的 instance id")

    @property
    def valid_mask(self) -> np.ndarray:
        """回傳深度大於零且為有限值的像素遮罩。"""

        return np.isfinite(self.depth_m) & (self.depth_m > 0.0)

    @property
    def shape(self) -> tuple[int, int]:
        """以高度、寬度回傳影像尺寸。"""

        return self.depth_m.shape

    def camera_points(self) -> np.ndarray:
        """將有效深度反投影至 OpenCV 光學相機座標。"""

        rows, columns = np.nonzero(self.valid_mask)
        depth = self.depth_m[rows, columns].astype(np.float64)
        fx = float(self.intrinsics[0, 0])
        fy = float(self.intrinsics[1, 1])
        cx = float(self.intrinsics[0, 2])
        cy = float(self.intrinsics[1, 2])
        x_values = (columns.astype(np.float64) - cx) * depth / fx
        y_values = (rows.astype(np.float64) - cy) * depth / fy
        return np.column_stack((x_values, y_values, depth))

    def world_points(self) -> np.ndarray:
        """將有效深度反投影至公尺制世界座標。"""

        camera_points = self.camera_points()
        rotation = self.camera_to_world[:3, :3]
        translation = self.camera_to_world[:3, 3]
        return camera_points @ rotation.T + translation

    def point_colors(self) -> np.ndarray:
        """依有效深度順序回傳 0 到 1 的 RGB 顏色。"""

        return self.rgb[self.valid_mask].astype(np.float64) / 255.0


@dataclass(frozen=True)
class RGBDSimulationResult:
    """ground truth、感測觀測及兩者比較結果。"""

    ground_truth: RGBDFrame
    observation: RGBDFrame
    actual_camera_to_world: np.ndarray
    translation_error_m: np.ndarray
    rotation_error_deg: np.ndarray
    metrics: dict[str, float | int]


def camera_intrinsics(camera: CameraSpec) -> np.ndarray:
    """由垂直視角與解析度建立 pinhole 相機內參。"""

    focal_length = (camera.height / 2.0) / np.tan(
        np.deg2rad(camera.vertical_fov_deg) / 2.0
    )
    return np.asarray(
        [
            [focal_length, 0.0, (camera.width - 1.0) / 2.0],
            [0.0, focal_length, (camera.height - 1.0) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def camera_optical_pose(camera: CameraSpec) -> np.ndarray:
    """建立 OpenCV 光學座標至世界座標的剛體轉換。"""

    visualization_pose = camera_frame(camera)
    optical_pose = visualization_pose.copy()
    optical_pose[:3, 1] *= -1.0
    return optical_pose


def _stack_clouds(
    point_clouds: tuple[PointCloud, ...],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[str, ...]]:
    points = np.concatenate([cloud.points for cloud in point_clouds], axis=0)
    colors = np.concatenate(
        [
            np.repeat(
                np.asarray(cloud.color, dtype=np.float64)[None, :],
                cloud.points.shape[0],
                axis=0,
            )
            for cloud in point_clouds
        ],
        axis=0,
    )
    instance_ids = np.concatenate(
        [
            np.full(cloud.points.shape[0], index, dtype=np.uint16)
            for index, cloud in enumerate(point_clouds, start=1)
        ]
    )
    names = ("background", *(cloud.name for cloud in point_clouds))
    return points, colors, instance_ids, names


def project_point_clouds(
    point_clouds: tuple[PointCloud, ...],
    camera: CameraSpec,
    projection_camera_to_world: np.ndarray | None = None,
    stored_camera_to_world: np.ndarray | None = None,
    frame_id: str = "frame",
) -> RGBDFrame:
    """投影世界點並以每個像素最近深度完成 z-buffer。"""

    intrinsics = camera_intrinsics(camera)
    projection_pose = (
        camera_optical_pose(camera)
        if projection_camera_to_world is None
        else np.asarray(projection_camera_to_world, dtype=np.float64)
    )
    stored_pose = (
        projection_pose
        if stored_camera_to_world is None
        else np.asarray(stored_camera_to_world, dtype=np.float64)
    )
    points, colors, instance_ids, names = _stack_clouds(point_clouds)
    rotation = projection_pose[:3, :3]
    translation = projection_pose[:3, 3]
    camera_points = (points - translation) @ rotation

    depth = camera_points[:, 2]
    finite = np.all(np.isfinite(camera_points), axis=1)
    in_depth_range = (depth >= camera.near) & (depth <= camera.far)
    candidates = finite & in_depth_range
    candidate_indices = np.flatnonzero(candidates)
    candidate_points = camera_points[candidates]
    candidate_depth = candidate_points[:, 2]

    columns = np.rint(
        intrinsics[0, 0] * candidate_points[:, 0] / candidate_depth
        + intrinsics[0, 2]
    ).astype(np.int64)
    rows = np.rint(
        intrinsics[1, 1] * candidate_points[:, 1] / candidate_depth
        + intrinsics[1, 2]
    ).astype(np.int64)
    inside = (
        (columns >= 0)
        & (columns < camera.width)
        & (rows >= 0)
        & (rows < camera.height)
    )
    columns = columns[inside]
    rows = rows[inside]
    candidate_depth = candidate_depth[inside]
    candidate_indices = candidate_indices[inside]

    depth_image = np.zeros((camera.height, camera.width), dtype=np.float32)
    rgb_image = np.zeros((camera.height, camera.width, 3), dtype=np.uint8)
    instance_mask = np.zeros((camera.height, camera.width), dtype=np.uint16)
    if candidate_indices.size:
        flat_pixels = rows * camera.width + columns
        order = np.lexsort((candidate_indices, candidate_depth))
        sorted_pixels = flat_pixels[order]
        _, first_occurrences = np.unique(sorted_pixels, return_index=True)
        selected = order[first_occurrences]
        selected_rows = rows[selected]
        selected_columns = columns[selected]
        source_indices = candidate_indices[selected]
        depth_image[selected_rows, selected_columns] = candidate_depth[selected]
        rgb_image[selected_rows, selected_columns] = np.clip(
            colors[source_indices] * 255.0,
            0.0,
            255.0,
        ).astype(np.uint8)
        instance_mask[selected_rows, selected_columns] = instance_ids[source_indices]

    return RGBDFrame(
        rgb=rgb_image,
        depth_m=depth_image,
        instance_mask=instance_mask,
        intrinsics=intrinsics,
        camera_to_world=stored_pose,
        instance_names=names,
        frame_id=frame_id,
    )


def _perturb_camera_pose(
    nominal_pose: np.ndarray,
    noise: SensorNoiseSpec,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    translation_error = rng.normal(0.0, noise.extrinsic_translation_std_m, size=3)
    rotation_error = rng.normal(0.0, noise.extrinsic_rotation_std_deg, size=3)
    local_rotation = pose_matrix((0.0, 0.0, 0.0), tuple(rotation_error))[:3, :3]
    actual_pose = nominal_pose.copy()
    actual_pose[:3, :3] = nominal_pose[:3, :3] @ local_rotation
    actual_pose[:3, 3] = nominal_pose[:3, 3] + nominal_pose[:3, :3] @ translation_error
    return actual_pose, translation_error, rotation_error


def _apply_depth_noise(
    frame: RGBDFrame,
    camera: CameraSpec,
    rng: np.random.Generator,
) -> tuple[RGBDFrame, int]:
    depth = frame.depth_m.astype(np.float64, copy=True)
    valid = frame.valid_mask
    valid_depth = depth[valid]
    noise = camera.noise
    standard_deviation = (
        noise.axial_noise_std_base_m
        + noise.axial_noise_std_per_m2 * np.square(valid_depth)
    )
    valid_depth += rng.normal(0.0, standard_deviation)
    if noise.depth_quantization_m > 0.0:
        valid_depth = (
            np.rint(valid_depth / noise.depth_quantization_m)
            * noise.depth_quantization_m
        )
    valid_depth[
        (valid_depth < camera.near)
        | (valid_depth > camera.far)
        | ~np.isfinite(valid_depth)
    ] = 0.0
    depth[valid] = valid_depth

    dropout = valid & (rng.random(depth.shape) < noise.dropout_probability)
    depth[dropout] = 0.0
    noisy_frame = RGBDFrame(
        rgb=frame.rgb.copy(),
        depth_m=depth.astype(np.float32),
        instance_mask=frame.instance_mask.copy(),
        intrinsics=frame.intrinsics.copy(),
        camera_to_world=frame.camera_to_world.copy(),
        instance_names=frame.instance_names,
        frame_id=frame.frame_id,
    )
    return noisy_frame, int(np.count_nonzero(dropout))


def compare_depth_frames(
    ground_truth: RGBDFrame,
    observation: RGBDFrame,
) -> dict[str, float | int]:
    """以共同有效像素計算深度誤差與覆蓋率。"""

    if ground_truth.shape != observation.shape:
        raise ValueError("比較的 RGB-D frame 尺寸必須一致")
    total_pixels = int(np.prod(ground_truth.shape))
    ground_truth_valid = ground_truth.valid_mask
    observation_valid = observation.valid_mask
    common = ground_truth_valid & observation_valid
    errors = observation.depth_m[common] - ground_truth.depth_m[common]
    absolute_errors = np.abs(errors)
    if not errors.size:
        raise ValueError("ground truth 與 observation 沒有共同有效深度像素")
    mae = float(np.mean(absolute_errors))
    rmse = float(np.sqrt(np.mean(np.square(errors))))
    bias = float(np.mean(errors))
    p95 = float(np.percentile(absolute_errors, 95.0))
    ground_truth_count = int(np.count_nonzero(ground_truth_valid))
    observation_count = int(np.count_nonzero(observation_valid))
    common_count = int(np.count_nonzero(common))
    return {
        "total_pixels": total_pixels,
        "ground_truth_valid_pixels": ground_truth_count,
        "observation_valid_pixels": observation_count,
        "common_valid_pixels": common_count,
        "ground_truth_fill_ratio": ground_truth_count / total_pixels,
        "observation_fill_ratio": observation_count / total_pixels,
        "common_retention_ratio": common_count / max(ground_truth_count, 1),
        "depth_mae_m": mae,
        "depth_rmse_m": rmse,
        "depth_bias_m": bias,
        "depth_p95_abs_error_m": p95,
    }


def simulate_rgbd(scene_data: SceneData) -> RGBDSimulationResult:
    """產生理想影像與含深度／外參誤差的可重現觀測。"""

    camera = scene_data.spec.camera
    nominal_pose = camera_optical_pose(camera)
    ground_truth = project_point_clouds(
        scene_data.point_clouds,
        camera,
        projection_camera_to_world=nominal_pose,
        stored_camera_to_world=nominal_pose,
        frame_id="ground_truth",
    )
    rng = np.random.default_rng(scene_data.spec.seed + 1009)
    actual_pose, translation_error, rotation_error = _perturb_camera_pose(
        nominal_pose,
        camera.noise,
        rng,
    )
    clean_observation = project_point_clouds(
        scene_data.point_clouds,
        camera,
        projection_camera_to_world=actual_pose,
        stored_camera_to_world=nominal_pose,
        frame_id="observation",
    )
    observation, dropout_count = _apply_depth_noise(clean_observation, camera, rng)
    metrics = compare_depth_frames(ground_truth, observation)
    metrics.update(
        {
            "injected_depth_dropouts": dropout_count,
            "extrinsic_translation_error_norm_m": float(
                np.linalg.norm(translation_error)
            ),
            "extrinsic_rotation_error_norm_deg": float(
                np.linalg.norm(rotation_error)
            ),
        }
    )
    return RGBDSimulationResult(
        ground_truth=ground_truth,
        observation=observation,
        actual_camera_to_world=actual_pose,
        translation_error_m=translation_error,
        rotation_error_deg=rotation_error,
        metrics=metrics,
    )
