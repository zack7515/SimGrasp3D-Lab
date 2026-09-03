"""設定、正向運動學與完整場景測試。"""

from pathlib import Path

import numpy as np

from simgrasp3d.io.point_cloud import export_scene_point_clouds
from simgrasp3d.scene.builder import build_scene, load_scene_spec

CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")


def test_scene_is_deterministic_for_fixed_seed() -> None:
    spec = load_scene_spec(CONFIG_PATH)
    first = build_scene(spec)
    second = build_scene(spec)
    assert len(first.point_clouds) == len(second.point_clouds)
    for first_cloud, second_cloud in zip(first.point_clouds, second.point_clouds, strict=True):
        np.testing.assert_array_equal(first_cloud.points, second_cloud.points)


def test_robot_has_one_more_joint_position_than_links() -> None:
    scene = build_scene(load_scene_spec(CONFIG_PATH))
    assert scene.robot_state.joint_positions.shape == (len(scene.spec.robot.links) + 1, 3)
    np.testing.assert_allclose(
        scene.robot_state.joint_positions[-1],
        scene.robot_state.joint_frames["flange"][:3, 3],
    )
    tcp_offset = np.linalg.norm(scene.spec.robot.gripper.tcp_offset)
    flange_to_tcp = np.linalg.norm(
        scene.robot_state.tool_frame[:3, 3]
        - scene.robot_state.joint_frames["flange"][:3, 3]
    )
    assert abs(flange_to_tcp - tcp_offset) < 1e-12


def test_objects_are_above_table_top() -> None:
    scene = build_scene(load_scene_spec(CONFIG_PATH))
    table = next(cloud for cloud in scene.point_clouds if cloud.name == "table")
    table_top = table.points[:, 2].max()
    for cloud in scene.point_clouds:
        if cloud.category == "object":
            assert cloud.points[:, 2].min() >= table_top - 1e-12


def test_point_cloud_export_contains_ply_header(tmp_path: Path) -> None:
    scene = build_scene(load_scene_spec(CONFIG_PATH))
    paths = export_scene_point_clouds(tmp_path, scene.point_clouds[:2])
    assert len(paths) == 3
    assert paths[0].read_text(encoding="ascii").startswith("ply\nformat ascii 1.0")
