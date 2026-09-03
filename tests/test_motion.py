"""軟管時間序列、IK、碰撞與匯出格式測試。"""

import json
from pathlib import Path

import numpy as np

from simgrasp3d.io.trajectory import MOTION_SCHEMA_VERSION, export_trajectory
from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.scene.builder import load_scene_spec
from simgrasp3d.visualization.motion_viewer import build_motion_figure

SCENE_CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")


def test_motion_has_continuous_frames_and_pinned_grasp(
    motion_trajectory: TrajectoryData,
) -> None:
    frames = motion_trajectory.frames
    times = np.asarray([frame.time_s for frame in frames])
    assert len(frames) == 116
    assert times[0] == 0.0
    assert np.all(np.diff(times) > 0.0)
    assert abs(times[-1] - 9.6) < 1e-9

    grasp_index = motion_trajectory.spec.hose.grasp_node_index
    for frame in frames:
        assert frame.hose_nodes.shape == (motion_trajectory.spec.hose.node_count, 3)
        assert frame.robot_joint_positions.shape == (7, 3)
        if frame.attached:
            np.testing.assert_allclose(
                frame.hose_nodes[grasp_index],
                frame.tcp_position,
                atol=1e-12,
            )


def test_motion_reports_ik_collision_and_constraint_quality(
    motion_trajectory: TrajectoryData,
) -> None:
    metrics = motion_trajectory.metrics
    assert metrics["failed_ik_frame_count"] == 0
    assert metrics["maximum_ik_error_m"] <= 0.002
    assert metrics["maximum_ik_orientation_error_deg"] <= 1.0
    assert metrics["collision_frame_count"] == 0
    assert metrics["minimum_robot_clearance_m"] >= 0.005
    assert metrics["unsafe_clearance_frame_count"] == 0
    assert metrics["inserted_waypoint_count"] == 1
    assert metrics["unresolved_path_segment_count"] == 0
    assert metrics["hose_contact_frame_count"] > 0
    assert metrics["hose_penetration_frame_count"] == 0
    assert metrics["maximum_hose_length_error_ratio"] < 0.01


def test_trajectory_export_is_pickle_free_and_shape_stable(
    tmp_path: Path,
    motion_trajectory: TrajectoryData,
) -> None:
    paths = export_trajectory(tmp_path, motion_trajectory)
    with np.load(paths["trajectory"], allow_pickle=False) as data:
        assert str(data["schema_version"].item()) == MOTION_SCHEMA_VERSION
        assert data["hose_nodes"].shape == (116, 49, 3)
        assert data["joint_angles_deg"].shape == (116, 6)
        assert data["tcp_rotation"].shape == (116, 3, 3)
        assert data["tool_frame"].shape == (116, 4, 4)
        assert data["ik_orientation_error_deg"].shape == (116,)
        assert data["phase"].dtype.kind == "U"

    metadata = json.loads(paths["metrics"].read_text(encoding="utf-8"))
    assert metadata["physics_engine"] is None
    assert metadata["solver"] == "kinematic_pose_and_geometric_constraints"
    assert metadata["metrics"]["collision_frame_count"] == 0
    assert len(metadata["planned_keyframes"]) == 12
    assert sum(item["generated"] for item in metadata["planned_keyframes"]) == 1


def test_planner_inserts_visible_generated_waypoint(
    motion_trajectory: TrajectoryData,
) -> None:
    generated = [
        keyframe
        for keyframe in motion_trajectory.planned_keyframes
        if keyframe.generated
    ]
    assert len(generated) == 1
    assert generated[0].phase == "自動安全繞行"
    assert any(frame.phase == "自動安全繞行" for frame in motion_trajectory.frames)


def test_motion_figure_contains_all_animation_frames(
    motion_trajectory: TrajectoryData,
) -> None:
    scene_spec = load_scene_spec(SCENE_CONFIG_PATH)
    figure = build_motion_figure(
        motion_trajectory,
        scene_spec.robot,
        scene_spec.table,
    )
    assert len(figure.frames) == len(motion_trajectory.frames)
    assert figure.layout.updatemenus[0].buttons[0].label == "▶ 播放"
    assert figure.layout.updatemenus[0].buttons[2].label == "↺ 從頭"
    assert figure.frames[-1].data[2].text[0].startswith("安全退回")
    assert figure.frames[-1].data[4].name == "軟管夾取節點"
    assert figure.frames[-1].data[5].name == "TCP 已走路徑"
    assert len(figure.frames[-1].data[5].x) == len(motion_trajectory.frames)
