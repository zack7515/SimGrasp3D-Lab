"""將感知、IK、碰撞與物理指標組成 fail-closed 控制重播。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from simgrasp3d.io import load_spec
from simgrasp3d.models.integration import (
    IntegrationSpec,
    ReplayEvent,
    ReplayResult,
    ValidatedGrasp,
)
from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.models.perception import PerceptionResult
from simgrasp3d.models.specs import RobotSpec
from simgrasp3d.robot.collision import evaluate_robot_clearance
from simgrasp3d.robot.kinematics import solve_pose_ik


def load_integration_spec(path: str | Path) -> IntegrationSpec:
    """讀取並驗證 fail-closed 安全門檻。"""

    return load_spec(path, IntegrationSpec)


def _validated_grasp(
    perception: PerceptionResult,
    trajectory: TrajectoryData,
    robot: RobotSpec,
    spec: IntegrationSpec,
) -> ValidatedGrasp | None:
    """依分數排序，選出同時通過 pregrasp／grasp IK 與碰撞的候選。"""

    initial_angles = np.asarray(
        [link.joint_angle_deg for link in robot.links],
        dtype=np.float64,
    )
    for candidate in perception.grasp_candidates:
        if (
            not candidate.geometry_feasible
            or candidate.required_opening_m > robot.gripper.opening
        ):
            continue
        pregrasp = solve_pose_ik(
            robot,
            candidate.pregrasp_position,
            candidate.tcp_rotation,
            initial_angles,
        )
        grasp = solve_pose_ik(
            robot,
            candidate.tcp_position,
            candidate.tcp_rotation,
            pregrasp.joint_angles_deg,
        )
        opening = min(candidate.required_opening_m, robot.gripper.opening)
        pregrasp_clearance = evaluate_robot_clearance(
            robot,
            pregrasp.joint_positions,
            pregrasp.tool_frame,
            opening,
            trajectory.spec.obstacles,
            trajectory.spec.table_top_z,
        )
        grasp_clearance = evaluate_robot_clearance(
            robot,
            grasp.joint_positions,
            grasp.tool_frame,
            opening,
            trajectory.spec.obstacles,
            trajectory.spec.table_top_z,
        )
        minimum_clearance = min(
            pregrasp_clearance.minimum_clearance_m,
            grasp_clearance.minimum_clearance_m,
        )
        maximum_position_error = max(
            pregrasp.position_error_m,
            grasp.position_error_m,
        )
        maximum_orientation_error = max(
            pregrasp.orientation_error_deg,
            grasp.orientation_error_deg,
        )
        if (
            pregrasp.converged
            and grasp.converged
            and minimum_clearance >= spec.minimum_robot_clearance_m
            and maximum_position_error <= spec.maximum_ik_position_error_m
            and maximum_orientation_error <= spec.maximum_ik_orientation_error_deg
        ):
            return ValidatedGrasp(
                candidate=candidate,
                pregrasp_joint_angles_deg=pregrasp.joint_angles_deg,
                grasp_joint_angles_deg=grasp.joint_angles_deg,
                minimum_clearance_m=minimum_clearance,
                maximum_ik_position_error_m=maximum_position_error,
                maximum_ik_orientation_error_deg=maximum_orientation_error,
            )
    return None


def _failure_codes(
    trajectory: TrajectoryData,
    perception: PerceptionResult,
    selected_grasp: ValidatedGrasp | None,
    spec: IntegrationSpec,
) -> tuple[str, ...]:
    """以固定順序產生可統計的失敗分類。"""

    metrics = trajectory.metrics
    failures: list[str] = []
    if spec.require_feasible_grasp and selected_grasp is None:
        failures.append("PERCEPTION_GRASP_UNAVAILABLE")
    if abs(float(perception.metrics["table_height_error_m"])) > (
        spec.maximum_table_height_error_m
    ):
        failures.append("TABLE_MODEL_OUT_OF_TOLERANCE")
    if int(metrics["unresolved_path_segment_count"]) > (
        spec.maximum_unresolved_path_segments
    ):
        failures.append("PATH_UNRESOLVED")
    if (
        float(metrics["maximum_ik_error_m"])
        > spec.maximum_ik_position_error_m
        or float(metrics["maximum_ik_orientation_error_deg"])
        > spec.maximum_ik_orientation_error_deg
        or int(metrics["failed_ik_frame_count"]) > 0
    ):
        failures.append("IK_TOLERANCE_EXCEEDED")
    if int(metrics["collision_frame_count"]) > 0:
        failures.append("ROBOT_COLLISION")
    if float(metrics["minimum_robot_clearance_m"]) < spec.minimum_robot_clearance_m:
        failures.append("ROBOT_CLEARANCE_LOW")
    if int(metrics["hose_penetration_frame_count"]) > (
        spec.maximum_hose_penetration_frames
    ):
        failures.append("HOSE_PENETRATION_LIMIT")
    if float(metrics.get("maximum_contact_force_n", 0.0)) > (
        spec.maximum_sampled_contact_force_n
    ):
        failures.append("CONTACT_FORCE_LIMIT")
    if float(metrics.get("maximum_grasp_constraint_error_m", 0.0)) > (
        spec.maximum_grasp_constraint_error_m
    ):
        failures.append("GRASP_CONSTRAINT_LIMIT")
    if int(metrics.get("physics_nonfinite_frame_count", 0)) > 0:
        failures.append("PHYSICS_NONFINITE")
    return tuple(failures)


def _grasp_payload(grasp: ValidatedGrasp) -> dict[str, object]:
    return {
        "object_name": grasp.candidate.object_name,
        "score": grasp.candidate.score,
        "required_opening_m": grasp.candidate.required_opening_m,
        "pregrasp_position": grasp.candidate.pregrasp_position.tolist(),
        "grasp_position": grasp.candidate.tcp_position.tolist(),
        "pregrasp_joint_angles_deg": grasp.pregrasp_joint_angles_deg.tolist(),
        "grasp_joint_angles_deg": grasp.grasp_joint_angles_deg.tolist(),
        "minimum_clearance_m": grasp.minimum_clearance_m,
        "maximum_ik_position_error_m": grasp.maximum_ik_position_error_m,
        "maximum_ik_orientation_error_deg": grasp.maximum_ik_orientation_error_deg,
    }


def build_fail_closed_replay(
    trajectory: TrajectoryData,
    perception: PerceptionResult,
    robot: RobotSpec,
    spec: IntegrationSpec,
) -> ReplayResult:
    """只有全部安全閘門通過時才產生逐幀控制命令。"""

    selected_grasp = _validated_grasp(perception, trajectory, robot, spec)
    failure_codes = _failure_codes(
        trajectory,
        perception,
        selected_grasp,
        spec,
    )
    events: list[ReplayEvent] = [
        ReplayEvent(
            sequence=0,
            time_s=0.0,
            state="INITIALIZING",
            event="SYSTEM_INITIALIZED",
            payload={
                "physics_engine": trajectory.physics_engine,
                "solver": trajectory.solver_name,
            },
        ),
        ReplayEvent(
            sequence=1,
            time_s=0.0,
            state="PERCEPTION_READY",
            event="PERCEPTION_ANALYZED",
            payload={
                "detected_object_count": perception.metrics[
                    "detected_object_count"
                ],
                "feasible_grasp_candidate_count": perception.metrics[
                    "feasible_grasp_candidate_count"
                ],
            },
        ),
    ]
    if selected_grasp is not None:
        events.append(
            ReplayEvent(
                sequence=len(events),
                time_s=0.0,
                state="GRASP_VALIDATED",
                event="PERCEPTION_GRASP_IK_VALIDATED",
                payload=_grasp_payload(selected_grasp),
            )
        )
    if failure_codes:
        events.append(
            ReplayEvent(
                sequence=len(events),
                time_s=0.0,
                state="ABORTED",
                event="SAFETY_GATE_REJECTED",
                payload={"failure_codes": list(failure_codes)},
            )
        )
    else:
        events.append(
            ReplayEvent(
                sequence=len(events),
                time_s=0.0,
                state="PLAN_VALIDATED",
                event="SAFETY_GATE_ACCEPTED",
                payload={"failure_codes": []},
            )
        )
        previous_attached = False
        for frame in trajectory.frames:
            if frame.attached and not previous_attached:
                state = "GRASPING"
            elif frame.attached:
                state = "TRANSPORTING"
            elif previous_attached and not frame.attached:
                state = "RELEASING"
            else:
                state = "EXECUTING"
            events.append(
                ReplayEvent(
                    sequence=len(events),
                    time_s=frame.time_s,
                    state=state,
                    event="TRAJECTORY_FRAME_COMMAND",
                    payload={
                        "phase": frame.phase,
                        "joint_angles_deg": frame.joint_angles_deg.tolist(),
                        "tcp_position": frame.tcp_position.tolist(),
                        "tcp_rpy_deg": frame.tcp_rpy_deg.tolist(),
                        "gripper_opening_m": frame.gripper_opening_m,
                        "attached": frame.attached,
                        "robot_clearance_m": frame.minimum_clearance_m,
                        "sampled_contact_force_n": frame.maximum_contact_force_n,
                    },
                )
            )
            previous_attached = frame.attached
        events.append(
            ReplayEvent(
                sequence=len(events),
                time_s=trajectory.frames[-1].time_s,
                state="COMPLETE",
                event="TRAJECTORY_COMPLETED",
                payload={"frame_count": len(trajectory.frames)},
            )
        )

    authorized = not failure_codes
    metrics: dict[str, float | int] = {
        "execution_authorized": int(authorized),
        "failure_count": len(failure_codes),
        "event_count": len(events),
        "command_frame_count": len(trajectory.frames) if authorized else 0,
        "replay_duration_s": trajectory.frames[-1].time_s if authorized else 0.0,
        "validated_grasp_count": int(selected_grasp is not None),
        "selected_grasp_score": (
            selected_grasp.candidate.score if selected_grasp is not None else 0.0
        ),
        "selected_grasp_clearance_m": (
            selected_grasp.minimum_clearance_m if selected_grasp is not None else 0.0
        ),
    }
    return ReplayResult(
        execution_authorized=authorized,
        failure_codes=failure_codes,
        selected_grasp=selected_grasp,
        events=tuple(events),
        metrics=metrics,
    )
