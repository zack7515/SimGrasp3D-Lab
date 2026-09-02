"""RGB-D 桌面、OBB、法向與抓取候選測試。"""

import json
from pathlib import Path

import numpy as np

from simgrasp3d.io.perception import export_perception_result
from simgrasp3d.models.perception import PerceptionResult
from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.sensors.rgbd import simulate_rgbd
from simgrasp3d.visualization.perception_viewer import build_perception_figure


SCENE_CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")


def test_perception_finds_table_objects_and_feasible_grasp(
    perception_result: PerceptionResult,
) -> None:
    metrics = perception_result.metrics
    assert metrics["detected_object_count"] == 3
    assert metrics["grasp_candidate_count"] == 6
    assert metrics["feasible_grasp_candidate_count"] >= 1
    assert metrics["table_plane_rms_error_m"] < 0.005
    assert abs(metrics["table_height_error_m"]) < 0.003
    assert metrics["table_tilt_error_deg"] < 0.2
    for geometry in perception_result.objects:
        assert geometry.points.shape[1] == 3
        assert geometry.normals.shape == geometry.normal_points.shape
        np.testing.assert_allclose(
            np.linalg.det(geometry.obb.rotation),
            1.0,
            atol=1e-10,
        )


def test_perception_export_is_inspectable_and_pickle_free(
    tmp_path: Path,
    perception_result: PerceptionResult,
) -> None:
    paths = export_perception_result(tmp_path, perception_result)
    payload = json.loads(paths["geometry"].read_text(encoding="utf-8"))
    assert payload["segmentation_mode"] == "oracle_instance_mask+geometric_ransac"
    assert len(payload["objects"]) == 3
    assert len(payload["ranked_grasp_candidates"]) == 6
    assert all(path.exists() for path in paths.values())


def test_perception_figure_contains_obb_normals_and_grasps(
    perception_result: PerceptionResult,
) -> None:
    scene = build_scene(load_scene_spec(SCENE_CONFIG_PATH))
    sensor_result = simulate_rgbd(scene)
    figure = build_perception_figure(sensor_result.observation, perception_result)
    names = {trace.name for trace in figure.data}
    assert "blue_box OBB" in names
    assert "orange_cylinder normals" in names
    assert any(name.startswith("G1 ") for name in names)
