"""為直線 TCP 動作搜尋可解釋的單一安全繞行 waypoint。"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from simgrasp3d.geometry.collision import capsule_clearance
from simgrasp3d.geometry.transforms import (
    matrix_from_quaternion,
    pose_matrix,
    quaternion_from_matrix,
    quaternion_slerp,
    rpy_deg_from_matrix,
)
from simgrasp3d.models.motion import HoseMotionSpec, MotionKeyframeSpec


@dataclass(frozen=True)
class WaypointPlanResult:
    """自動規劃後的關鍵幀及未解決線段數。"""

    keyframes: tuple[MotionKeyframeSpec, ...]
    inserted_waypoint_count: int
    unresolved_segment_count: int


def _segment_environment_clearance(
    start: np.ndarray,
    end: np.ndarray,
    spec: HoseMotionSpec,
) -> float:
    radius = spec.waypoint_planner.tool_envelope_radius_m
    return min(
        capsule_clearance(
            start,
            end,
            radius,
            np.asarray(obstacle.start),
            np.asarray(obstacle.end),
            obstacle.radius,
        )
        for obstacle in spec.obstacles
    )


def _intermediate_rpy(
    first: MotionKeyframeSpec,
    second: MotionKeyframeSpec,
) -> tuple[float, float, float]:
    first_rotation = pose_matrix((0.0, 0.0, 0.0), first.tcp_rpy_deg)[:3, :3]
    second_rotation = pose_matrix((0.0, 0.0, 0.0), second.tcp_rpy_deg)[:3, :3]
    quaternion = quaternion_slerp(
        quaternion_from_matrix(first_rotation),
        quaternion_from_matrix(second_rotation),
        0.5,
    )
    rpy = rpy_deg_from_matrix(matrix_from_quaternion(quaternion))
    return (float(rpy[0]), float(rpy[1]), float(rpy[2]))


def _candidate_offsets(step_m: float, maximum_m: float) -> list[np.ndarray]:
    """依序產生優先向上，再向工作區側邊偏移的搜尋位置。"""

    offsets: list[np.ndarray] = []
    distance = step_m
    while distance <= maximum_m + 1e-12:
        offsets.extend(
            (
                np.asarray([0.0, 0.0, distance]),
                np.asarray([0.0, distance, distance * 0.5]),
                np.asarray([0.0, -distance, distance * 0.5]),
                np.asarray([distance, 0.0, distance * 0.5]),
                np.asarray([-distance, 0.0, distance * 0.5]),
            )
        )
        distance += step_m
    return offsets


def _find_detour(
    first: MotionKeyframeSpec,
    second: MotionKeyframeSpec,
    spec: HoseMotionSpec,
) -> np.ndarray | None:
    start = np.asarray(first.tcp_position, dtype=np.float64)
    end = np.asarray(second.tcp_position, dtype=np.float64)
    midpoint = (start + end) / 2.0
    candidates: list[tuple[float, np.ndarray]] = []
    for offset in _candidate_offsets(
        spec.waypoint_planner.detour_step_m,
        spec.waypoint_planner.maximum_detour_m,
    ):
        candidate = midpoint + offset
        first_clearance = _segment_environment_clearance(start, candidate, spec)
        second_clearance = _segment_environment_clearance(candidate, end, spec)
        minimum_clearance = min(first_clearance, second_clearance)
        if minimum_clearance < spec.safe_clearance_m:
            continue
        path_length = float(
            np.linalg.norm(candidate - start) + np.linalg.norm(end - candidate)
        )
        candidates.append((path_length, candidate))
    if not candidates:
        return None
    return min(candidates, key=lambda item: item[0])[1]


def plan_safe_waypoints(spec: HoseMotionSpec) -> WaypointPlanResult:
    """檢查每段 TCP 直線，必要時插入一個安全繞行點。"""

    if not spec.waypoint_planner.enabled:
        unresolved = sum(
            _segment_environment_clearance(
                np.asarray(first.tcp_position, dtype=np.float64),
                np.asarray(second.tcp_position, dtype=np.float64),
                spec,
            )
            < spec.safe_clearance_m
            for first, second in zip(
                spec.keyframes[:-1],
                spec.keyframes[1:],
                strict=True,
            )
        )
        return WaypointPlanResult(spec.keyframes, 0, unresolved)
    planned: list[MotionKeyframeSpec] = [spec.keyframes[0]]
    inserted = 0
    unresolved = 0
    for target in spec.keyframes[1:]:
        previous = planned[-1]
        start = np.asarray(previous.tcp_position, dtype=np.float64)
        end = np.asarray(target.tcp_position, dtype=np.float64)
        direct_clearance = _segment_environment_clearance(start, end, spec)
        if direct_clearance >= spec.safe_clearance_m:
            planned.append(target)
            continue
        detour = _find_detour(previous, target, spec)
        if detour is None:
            unresolved += 1
            planned.append(target)
            continue

        first_distance = float(np.linalg.norm(detour - start))
        second_distance = float(np.linalg.norm(end - detour))
        total_distance = max(first_distance + second_distance, 1e-12)
        first_duration = target.duration_s * first_distance / total_distance
        second_duration = target.duration_s - first_duration
        opening = (previous.gripper_opening_m + target.gripper_opening_m) / 2.0
        planned.append(
            MotionKeyframeSpec(
                phase="自動安全繞行",
                duration_s=first_duration,
                tcp_position=(float(detour[0]), float(detour[1]), float(detour[2])),
                tcp_rpy_deg=_intermediate_rpy(previous, target),
                gripper_opening_m=opening,
                attached=previous.attached,
                generated=True,
            )
        )
        planned.append(replace(target, duration_s=second_duration))
        inserted += 1
    return WaypointPlanResult(tuple(planned), inserted, unresolved)
