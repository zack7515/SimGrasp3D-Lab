"""URDF/SRDF、抓取驗證與 fail-closed 控制重播測試。"""

import json
from dataclasses import replace
from pathlib import Path
from xml.etree import ElementTree as ET

from simgrasp3d.integration import build_fail_closed_replay, load_integration_spec
from simgrasp3d.io.integration import export_replay_result
from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.models.perception import PerceptionResult
from simgrasp3d.robot.description import build_srdf, build_urdf
from simgrasp3d.scene.builder import load_scene_spec


SCENE_CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")
INTEGRATION_CONFIG_PATH = Path("configs/integration/fail_closed_baseline.json")


def test_robot_description_contains_arm_chain_collision_and_tcp() -> None:
    robot = load_scene_spec(SCENE_CONFIG_PATH).robot
    urdf_root = ET.fromstring(build_urdf(robot))
    srdf_root = ET.fromstring(build_srdf(robot))
    revolute_joints = [
        joint for joint in urdf_root.findall("joint") if joint.get("type") == "revolute"
    ]
    assert len(revolute_joints) == 6
    assert urdf_root.find("link[@name='tool0']") is not None
    assert urdf_root.find("link[@name='gripper_left_finger']/collision") is not None
    assert srdf_root.find("group[@name='arm']/chain") is not None
    assert len(srdf_root.findall("disable_collisions")) >= 6


def test_fail_closed_replay_authorizes_validated_simulation(
    physics_trajectory: TrajectoryData,
    perception_result: PerceptionResult,
) -> None:
    robot = load_scene_spec(SCENE_CONFIG_PATH).robot
    result = build_fail_closed_replay(
        physics_trajectory,
        perception_result,
        robot,
        load_integration_spec(INTEGRATION_CONFIG_PATH),
    )
    assert result.execution_authorized
    assert result.failure_codes == ()
    assert result.selected_grasp is not None
    assert result.selected_grasp.candidate.object_name == "green_sphere"
    assert result.metrics["command_frame_count"] == 116
    assert result.events[-1].event == "TRAJECTORY_COMPLETED"


def test_fail_closed_replay_emits_no_commands_when_force_gate_fails(
    physics_trajectory: TrajectoryData,
    perception_result: PerceptionResult,
) -> None:
    metrics = dict(physics_trajectory.metrics)
    metrics["maximum_contact_force_n"] = 31.0
    unsafe = replace(physics_trajectory, metrics=metrics)
    result = build_fail_closed_replay(
        unsafe,
        perception_result,
        load_scene_spec(SCENE_CONFIG_PATH).robot,
        load_integration_spec(INTEGRATION_CONFIG_PATH),
    )
    assert not result.execution_authorized
    assert "CONTACT_FORCE_LIMIT" in result.failure_codes
    assert result.metrics["command_frame_count"] == 0
    assert all(event.event != "TRAJECTORY_FRAME_COMMAND" for event in result.events)
    assert result.events[-1].event == "SAFETY_GATE_REJECTED"


def test_replay_export_is_jsonl_and_summary_is_explicit(
    tmp_path: Path,
    physics_trajectory: TrajectoryData,
    perception_result: PerceptionResult,
) -> None:
    result = build_fail_closed_replay(
        physics_trajectory,
        perception_result,
        load_scene_spec(SCENE_CONFIG_PATH).robot,
        load_integration_spec(INTEGRATION_CONFIG_PATH),
    )
    paths = export_replay_result(tmp_path, result)
    events = [
        json.loads(line)
        for line in paths["replay"].read_text(encoding="utf-8").splitlines()
    ]
    summary = json.loads(paths["summary"].read_text(encoding="utf-8"))
    assert [event["sequence"] for event in events] == list(range(len(events)))
    assert summary["result_scope"] == "simulation_only"
    assert summary["execution_authorized"] is True
    assert summary["failure_codes"] == []
