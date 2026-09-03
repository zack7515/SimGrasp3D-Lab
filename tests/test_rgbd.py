"""RGB-D 投影、z-buffer、雜訊與資料交換格式測試。"""

from pathlib import Path

import numpy as np

from simgrasp3d.geometry.sampling import PointCloud
from simgrasp3d.io.rgbd_frame import read_rgbd_frame, write_rgbd_frame
from simgrasp3d.models.specs import CameraSpec
from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.sensors.rgbd import project_point_clouds, simulate_rgbd

CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")


def _forward_camera() -> CameraSpec:
    return CameraSpec.from_dict(
        {
            "name": "test_camera",
            "position": [0.0, 0.0, 0.0],
            "look_at": [0.0, 0.0, 1.0],
            "up": [0.0, -1.0, 0.0],
            "vertical_fov_deg": 90.0,
            "aspect_ratio": 1.0,
            "width": 5,
            "height": 5,
            "near": 0.1,
            "far": 3.0,
        }
    )


def test_z_buffer_keeps_nearest_point() -> None:
    camera = _forward_camera()
    far_cloud = PointCloud(
        "far",
        np.asarray([[0.0, 0.0, 2.0]]),
        (0.0, 0.0, 1.0),
        "object",
    )
    near_cloud = PointCloud(
        "near",
        np.asarray([[0.0, 0.0, 1.0]]),
        (1.0, 0.0, 0.0),
        "object",
    )

    frame = project_point_clouds((far_cloud, near_cloud), camera)

    assert frame.depth_m[2, 2] == 1.0
    np.testing.assert_array_equal(frame.rgb[2, 2], [255, 0, 0])
    assert frame.instance_mask[2, 2] == 2


def test_center_depth_backprojects_to_original_world_point() -> None:
    camera = _forward_camera()
    cloud = PointCloud(
        "center",
        np.asarray([[0.0, 0.0, 1.25]]),
        (1.0, 1.0, 1.0),
        "object",
    )

    frame = project_point_clouds((cloud,), camera)

    np.testing.assert_allclose(frame.world_points(), [[0.0, 0.0, 1.25]], atol=1e-12)


def test_sensor_simulation_is_reproducible_and_quantized() -> None:
    scene = build_scene(load_scene_spec(CONFIG_PATH))
    first = simulate_rgbd(scene)
    second = simulate_rgbd(scene)

    np.testing.assert_array_equal(first.observation.depth_m, second.observation.depth_m)
    assert first.metrics == second.metrics
    assert first.metrics["injected_depth_dropouts"] > 0
    valid_depth = first.observation.depth_m[first.observation.valid_mask]
    quantization_m = scene.spec.camera.noise.depth_quantization_m
    np.testing.assert_allclose(
        valid_depth / quantization_m,
        np.rint(valid_depth / quantization_m),
        atol=1e-3,
    )
    np.testing.assert_allclose(
        first.observation.camera_to_world,
        first.ground_truth.camera_to_world,
    )
    assert not np.allclose(
        first.actual_camera_to_world,
        first.ground_truth.camera_to_world,
    )


def test_default_tabletop_sensor_metrics_match_baseline() -> None:
    result = simulate_rgbd(build_scene(load_scene_spec(CONFIG_PATH)))

    assert result.metrics["total_pixels"] == 19200
    assert result.metrics["ground_truth_valid_pixels"] == 4180
    assert result.metrics["observation_valid_pixels"] == 4080
    assert result.metrics["common_valid_pixels"] == 3536
    assert result.metrics["injected_depth_dropouts"] == 68
    np.testing.assert_allclose(result.metrics["depth_mae_m"], 0.0077073672, atol=1e-9)
    np.testing.assert_allclose(result.metrics["depth_rmse_m"], 0.0198405068, atol=1e-9)


def test_rgbd_npz_round_trip(tmp_path: Path) -> None:
    result = simulate_rgbd(build_scene(load_scene_spec(CONFIG_PATH)))
    path = write_rgbd_frame(tmp_path / "frame.npz", result.observation)

    restored = read_rgbd_frame(path)

    assert restored.frame_id == result.observation.frame_id
    assert restored.instance_names == result.observation.instance_names
    np.testing.assert_array_equal(restored.rgb, result.observation.rgb)
    np.testing.assert_array_equal(restored.depth_m, result.observation.depth_m)
    np.testing.assert_array_equal(
        restored.instance_mask,
        result.observation.instance_mask,
    )
    np.testing.assert_allclose(
        restored.camera_to_world,
        result.observation.camera_to_world,
    )
