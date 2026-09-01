"""以六自由度運動學與幾何約束模擬軟管抽取連續動作。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simgrasp3d.geometry.collision import closest_point_on_segment, segment_distance
from simgrasp3d.geometry.transforms import (
    matrix_from_quaternion,
    pose_matrix,
    quaternion_from_matrix,
    quaternion_slerp,
    rpy_deg_from_matrix,
)
from simgrasp3d.models.motion import (
    HoseMotionSpec,
    MotionKeyframeSpec,
    PipeObstacleSpec,
    TrajectoryData,
    TrajectoryFrame,
)
from simgrasp3d.models.specs import RobotSpec
from simgrasp3d.robot.collision import evaluate_robot_clearance
from simgrasp3d.robot.kinematics import solve_pose_ik
from simgrasp3d.simulation.waypoint_planner import plan_safe_waypoints


@dataclass(frozen=True)
class _MotionState:
    """關鍵幀插值後的一個 TCP 位置、姿態與夾取狀態。"""

    time_s: float
    phase: str
    tcp_position: np.ndarray
    tcp_rpy_deg: np.ndarray
    tcp_rotation: np.ndarray
    gripper_opening_m: float
    attached: bool


def load_hose_motion_spec(path: str | Path) -> HoseMotionSpec:
    """讀取並驗證 UTF-8 JSON 軟管動作情境。"""

    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    return HoseMotionSpec.from_dict(data)


def _resample_polyline(
    control_points: tuple[tuple[float, float, float], ...],
    count: int,
) -> np.ndarray:
    """依弧長等距重取樣軟管中心線。"""

    points = np.asarray(control_points, dtype=np.float64)
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    if cumulative[-1] <= 1e-9:
        raise ValueError("軟管控制點不可全部重疊")
    samples = np.linspace(0.0, cumulative[-1], count)
    return np.column_stack(
        [np.interp(samples, cumulative, points[:, axis]) for axis in range(3)]
    )


def _project_outside_obstacles(
    nodes: np.ndarray,
    obstacles: tuple[PipeObstacleSpec, ...],
    hose_radius: float,
    pinned_index: int | None,
) -> None:
    """將穿入固定管路的節點投影回圓柱外側。"""

    for obstacle in obstacles:
        start = np.asarray(obstacle.start, dtype=np.float64)
        end = np.asarray(obstacle.end, dtype=np.float64)
        required_distance = obstacle.radius + hose_radius
        for index, node in enumerate(nodes):
            if index == pinned_index:
                continue
            closest = closest_point_on_segment(node, start, end)
            radial = node - closest
            distance = float(np.linalg.norm(radial))
            if distance >= required_distance:
                continue
            if distance <= 1e-9:
                axis = end - start
                fallback = np.cross(axis, np.asarray([0.0, 0.0, 1.0]))
                if np.linalg.norm(fallback) <= 1e-9:
                    fallback = np.asarray([1.0, 0.0, 0.0])
                radial = fallback / np.linalg.norm(fallback)
            else:
                radial /= distance
            nodes[index] = closest + radial * required_distance

def _project_segment_samples(
    nodes: np.ndarray,
    obstacles: tuple[PipeObstacleSpec, ...],
    hose_radius: float,
    pinned_index: int | None,
) -> None:
    """補查節點間內插位置，降低線段切入障礙的離散化誤差。"""

    for obstacle in obstacles:
        start = np.asarray(obstacle.start, dtype=np.float64)
        end = np.asarray(obstacle.end, dtype=np.float64)
        required_distance = obstacle.radius + hose_radius
        for index in range(len(nodes) - 1):
            for fraction in (0.25, 0.5, 0.75):
                sample = (1.0 - fraction) * nodes[index] + fraction * nodes[index + 1]
                closest = closest_point_on_segment(sample, start, end)
                radial = sample - closest
                distance = float(np.linalg.norm(radial))
                if distance >= required_distance or distance <= 1e-9:
                    continue
                correction = radial / distance * (required_distance - distance)
                if index == pinned_index:
                    nodes[index + 1] += correction / fraction
                elif index + 1 == pinned_index:
                    nodes[index] += correction / (1.0 - fraction)
                else:
                    nodes[index] += correction
                    nodes[index + 1] += correction


def _solve_hose_step(
    previous_nodes: np.ndarray,
    spec: HoseMotionSpec,
    rest_lengths: np.ndarray,
    tcp_position: np.ndarray,
    attached: bool,
) -> np.ndarray:
    """以長度約束、重力與障礙投影更新一幀軟管中心線。"""

    nodes = previous_nodes.copy()
    hose = spec.hose
    # 幾何階段固定一個內部錨點以維持管長；夾取時錨點才跟隨 TCP。
    pinned_index = hose.grasp_node_index
    anchor_position = (
        tcp_position.copy() if attached else previous_nodes[pinned_index].copy()
    )
    if attached:
        distance_from_grasp = np.abs(np.arange(len(nodes)) - pinned_index)
        follow_weights = np.exp(-distance_from_grasp / hose.follow_decay_nodes)
        displacement = tcp_position - nodes[pinned_index]
        nodes += follow_weights[:, None] * displacement
    nodes[pinned_index] = anchor_position

    for _ in range(hose.constraint_iterations):
        if pinned_index is None:
            nodes[:, 2] -= hose.gravity_step_m / hose.constraint_iterations
        else:
            free_mask = np.ones(len(nodes), dtype=bool)
            free_mask[pinned_index] = False
            nodes[free_mask, 2] -= hose.gravity_step_m / hose.constraint_iterations

        if pinned_index is None:
            for index, rest_length in enumerate(rest_lengths):
                difference = nodes[index + 1] - nodes[index]
                length = float(np.linalg.norm(difference))
                if length <= 1e-12:
                    continue
                correction = difference * ((length - rest_length) / length)
                nodes[index] += correction * 0.5
                nodes[index + 1] -= correction * 0.5
        else:
            # 由夾取點向兩端傳遞固定節長，比逐段平均修正更快收斂。
            for index in range(pinned_index, len(nodes) - 1):
                difference = nodes[index + 1] - nodes[index]
                length = float(np.linalg.norm(difference))
                if length > 1e-12:
                    nodes[index + 1] = (
                        nodes[index] + difference / length * rest_lengths[index]
                    )
            for index in range(pinned_index - 1, -1, -1):
                difference = nodes[index] - nodes[index + 1]
                length = float(np.linalg.norm(difference))
                if length > 1e-12:
                    nodes[index] = (
                        nodes[index + 1] + difference / length * rest_lengths[index]
                    )

        minimum_z = spec.table_top_z + hose.radius
        if pinned_index is None:
            nodes[:, 2] = np.maximum(nodes[:, 2], minimum_z)
        else:
            free_mask = np.arange(len(nodes)) != pinned_index
            nodes[free_mask, 2] = np.maximum(nodes[free_mask, 2], minimum_z)
            nodes[pinned_index] = anchor_position
        _project_outside_obstacles(nodes, spec.obstacles, hose.radius, pinned_index)

    if pinned_index is not None:
        nodes[pinned_index] = anchor_position
    for _ in range(3):
        _project_segment_samples(nodes, spec.obstacles, hose.radius, pinned_index)
        free_mask = (
            np.arange(len(nodes)) != pinned_index
            if pinned_index is not None
            else slice(None)
        )
        nodes[free_mask, 2] = np.maximum(
            nodes[free_mask, 2],
            spec.table_top_z + hose.radius,
        )
        if pinned_index is not None:
            nodes[pinned_index] = anchor_position
    return nodes


def _hose_clearance(nodes: np.ndarray, spec: HoseMotionSpec) -> float:
    """計算軟管外表面到所有固定管路外表面的最短距離。"""

    clearances: list[float] = []
    for obstacle in spec.obstacles:
        start = np.asarray(obstacle.start, dtype=np.float64)
        end = np.asarray(obstacle.end, dtype=np.float64)
        for index in range(len(nodes) - 1):
            centerline_distance = segment_distance(
                nodes[index],
                nodes[index + 1],
                start,
                end,
            )
            clearances.append(centerline_distance - spec.hose.radius - obstacle.radius)
    return min(clearances, default=float("inf"))


def _smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def _interpolated_states(
    keyframes: tuple[MotionKeyframeSpec, ...],
    frame_rate_hz: int,
) -> list[_MotionState]:
    """以 smoothstep 與四元數 SLERP 產生固定幀率 TCP 狀態。"""

    first = keyframes[0]
    first_rotation = pose_matrix((0.0, 0.0, 0.0), first.tcp_rpy_deg)[:3, :3]
    states = [
        _MotionState(
            time_s=0.0,
            phase=first.phase,
            tcp_position=np.asarray(first.tcp_position, dtype=np.float64),
            tcp_rpy_deg=np.asarray(first.tcp_rpy_deg, dtype=np.float64),
            tcp_rotation=first_rotation,
            gripper_opening_m=first.gripper_opening_m,
            attached=first.attached,
        )
    ]
    time_s = 0.0
    for previous, target in zip(keyframes[:-1], keyframes[1:], strict=True):
        steps = max(1, int(round(target.duration_s * frame_rate_hz)))
        previous_rotation = pose_matrix(
            (0.0, 0.0, 0.0), previous.tcp_rpy_deg
        )[:3, :3]
        target_rotation = pose_matrix((0.0, 0.0, 0.0), target.tcp_rpy_deg)[:3, :3]
        previous_quaternion = quaternion_from_matrix(previous_rotation)
        target_quaternion = quaternion_from_matrix(target_rotation)
        for step in range(1, steps + 1):
            fraction = step / steps
            blend = _smoothstep(fraction)
            previous_position = np.asarray(previous.tcp_position, dtype=np.float64)
            target_position = np.asarray(target.tcp_position, dtype=np.float64)
            tcp_position = previous_position + blend * (target_position - previous_position)
            tcp_rotation = matrix_from_quaternion(
                quaternion_slerp(
                    previous_quaternion,
                    target_quaternion,
                    blend,
                )
            )
            tcp_rpy_deg = rpy_deg_from_matrix(tcp_rotation)
            opening = previous.gripper_opening_m + blend * (
                target.gripper_opening_m - previous.gripper_opening_m
            )
            # 夾取在閉爪階段末端生效，釋放則在開爪階段末端生效。
            attached = previous.attached if step < steps else target.attached
            time_s += target.duration_s / steps
            states.append(
                _MotionState(
                    time_s=time_s,
                    phase=target.phase,
                    tcp_position=tcp_position,
                    tcp_rpy_deg=tcp_rpy_deg,
                    tcp_rotation=tcp_rotation,
                    gripper_opening_m=opening,
                    attached=attached,
                )
            )
    return states


def simulate_hose_motion(spec: HoseMotionSpec, robot: RobotSpec) -> TrajectoryData:
    """產生可重現的軟管夾取、避障、搬運與放置時間序列。"""

    if any(
        keyframe.gripper_opening_m > robot.gripper.opening + 1e-12
        for keyframe in spec.keyframes
    ):
        raise ValueError("動作 keyframe 的夾爪開口超過機械手規格")

    waypoint_plan = plan_safe_waypoints(spec)
    initial_nodes = _resample_polyline(spec.hose.control_points, spec.hose.node_count)
    rest_lengths = np.linalg.norm(np.diff(initial_nodes, axis=0), axis=1)
    rest_total_length = float(rest_lengths.sum())
    states = _interpolated_states(
        waypoint_plan.keyframes,
        spec.frame_rate_hz,
    )
    previous_nodes = _solve_hose_step(
        initial_nodes,
        spec,
        rest_lengths,
        states[0].tcp_position,
        attached=False,
    )
    previous_angles = np.asarray(
        [link.joint_angle_deg for link in robot.links],
        dtype=np.float64,
    )
    frames: list[TrajectoryFrame] = []

    for state in states:
        ik_result = solve_pose_ik(
            robot,
            state.tcp_position,
            state.tcp_rotation,
            previous_angles,
        )
        previous_angles = ik_result.joint_angles_deg
        previous_nodes = _solve_hose_step(
            previous_nodes,
            spec,
            rest_lengths,
            state.tcp_position,
            state.attached,
        )
        hose_clearance = _hose_clearance(previous_nodes, spec)
        robot_clearance = evaluate_robot_clearance(
            robot,
            ik_result.joint_positions,
            ik_result.tool_frame,
            state.gripper_opening_m,
            spec.obstacles,
            spec.table_top_z,
        )
        current_length = float(np.linalg.norm(np.diff(previous_nodes, axis=0), axis=1).sum())
        frames.append(
            TrajectoryFrame(
                time_s=state.time_s,
                phase=state.phase,
                tcp_position=state.tcp_position.copy(),
                tcp_rpy_deg=state.tcp_rpy_deg.copy(),
                tcp_rotation=state.tcp_rotation.copy(),
                tool_frame=ik_result.tool_frame.copy(),
                gripper_opening_m=state.gripper_opening_m,
                attached=state.attached,
                hose_nodes=previous_nodes.copy(),
                robot_joint_positions=ik_result.joint_positions.copy(),
                joint_angles_deg=ik_result.joint_angles_deg.copy(),
                minimum_clearance_m=robot_clearance.minimum_clearance_m,
                hose_clearance_m=hose_clearance,
                link_clearance_m=robot_clearance.link_clearance_m,
                gripper_clearance_m=robot_clearance.gripper_clearance_m,
                closest_collision_pair=robot_clearance.closest_pair,
                collision=(
                    robot_clearance.minimum_clearance_m
                    < -spec.collision_tolerance_m
                ),
                ik_position_error_m=ik_result.position_error_m,
                ik_orientation_error_deg=ik_result.orientation_error_deg,
                hose_length_ratio=current_length / rest_total_length,
            )
        )

    clearances = np.asarray([frame.minimum_clearance_m for frame in frames])
    ik_errors = np.asarray([frame.ik_position_error_m for frame in frames])
    orientation_errors = np.asarray(
        [frame.ik_orientation_error_deg for frame in frames]
    )
    length_ratios = np.asarray([frame.hose_length_ratio for frame in frames])
    hose_clearances = np.asarray([frame.hose_clearance_m for frame in frames])
    metrics: dict[str, float | int] = {
        "frame_count": len(frames),
        "duration_s": frames[-1].time_s,
        "minimum_clearance_m": float(clearances.min()),
        "minimum_robot_clearance_m": float(clearances.min()),
        "minimum_link_clearance_m": float(
            min(frame.link_clearance_m for frame in frames)
        ),
        "minimum_gripper_clearance_m": float(
            min(frame.gripper_clearance_m for frame in frames)
        ),
        "minimum_hose_clearance_m": float(hose_clearances.min()),
        "collision_frame_count": int(
            np.count_nonzero(clearances < -spec.collision_tolerance_m)
        ),
        "unsafe_clearance_frame_count": int(
            np.count_nonzero(clearances < spec.safe_clearance_m)
        ),
        "hose_contact_frame_count": int(np.count_nonzero(hose_clearances <= 0.001)),
        "hose_penetration_frame_count": int(
            np.count_nonzero(hose_clearances < -spec.collision_tolerance_m)
        ),
        "maximum_ik_error_m": float(ik_errors.max()),
        "maximum_ik_orientation_error_deg": float(orientation_errors.max()),
        "failed_ik_frame_count": int(
            np.count_nonzero((ik_errors > 0.002) | (orientation_errors > 1.0))
        ),
        "maximum_hose_length_error_ratio": float(np.max(np.abs(length_ratios - 1.0))),
        "attached_frame_count": int(sum(frame.attached for frame in frames)),
        "planned_keyframe_count": len(waypoint_plan.keyframes),
        "inserted_waypoint_count": waypoint_plan.inserted_waypoint_count,
        "unresolved_path_segment_count": waypoint_plan.unresolved_segment_count,
    }
    return TrajectoryData(
        spec=spec,
        planned_keyframes=waypoint_plan.keyframes,
        frames=tuple(frames),
        metrics=metrics,
    )
