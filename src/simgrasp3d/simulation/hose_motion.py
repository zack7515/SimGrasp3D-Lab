"""以運動學與位置約束模擬軟管抽取連續動作。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from simgrasp3d.models.motion import (
    HoseMotionSpec,
    MotionKeyframeSpec,
    PipeObstacleSpec,
    TrajectoryData,
    TrajectoryFrame,
)
from simgrasp3d.models.specs import RobotSpec
from simgrasp3d.robot.kinematics import solve_position_ik


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


def _closest_point_on_segment(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    direction = end - start
    parameter = float(np.dot(point - start, direction) / np.dot(direction, direction))
    return start + np.clip(parameter, 0.0, 1.0) * direction


def _segment_distance(
    first_start: np.ndarray,
    first_end: np.ndarray,
    second_start: np.ndarray,
    second_end: np.ndarray,
) -> float:
    """計算兩個有限線段的最短距離。"""

    first_direction = first_end - first_start
    second_direction = second_end - second_start
    offset = first_start - second_start
    first_squared = float(np.dot(first_direction, first_direction))
    second_squared = float(np.dot(second_direction, second_direction))
    cross = float(np.dot(first_direction, second_direction))
    first_offset = float(np.dot(first_direction, offset))
    second_offset = float(np.dot(second_direction, offset))
    denominator = first_squared * second_squared - cross * cross

    if denominator <= 1e-12:
        first_parameter = 0.0
    else:
        first_parameter = np.clip(
            (cross * second_offset - first_offset * second_squared) / denominator,
            0.0,
            1.0,
        )
    second_parameter = np.clip(
        (cross * first_parameter + second_offset) / second_squared,
        0.0,
        1.0,
    )
    first_parameter = np.clip(
        (cross * second_parameter - first_offset) / first_squared,
        0.0,
        1.0,
    )
    first_point = first_start + first_parameter * first_direction
    second_point = second_start + second_parameter * second_direction
    return float(np.linalg.norm(first_point - second_point))


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
            closest = _closest_point_on_segment(node, start, end)
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
                closest = _closest_point_on_segment(sample, start, end)
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
    pinned_index = hose.grasp_node_index if attached else None
    if pinned_index is not None:
        distance_from_grasp = np.abs(np.arange(len(nodes)) - pinned_index)
        follow_weights = np.exp(-distance_from_grasp / hose.follow_decay_nodes)
        displacement = tcp_position - nodes[pinned_index]
        nodes += follow_weights[:, None] * displacement
        nodes[pinned_index] = tcp_position

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
            nodes[pinned_index] = tcp_position
        _project_outside_obstacles(nodes, spec.obstacles, hose.radius, pinned_index)

    if pinned_index is not None:
        nodes[pinned_index] = tcp_position
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
            nodes[pinned_index] = tcp_position
    return nodes


def _hose_clearance(nodes: np.ndarray, spec: HoseMotionSpec) -> float:
    """計算軟管外表面到所有固定管路外表面的最短距離。"""

    clearances: list[float] = []
    for obstacle in spec.obstacles:
        start = np.asarray(obstacle.start, dtype=np.float64)
        end = np.asarray(obstacle.end, dtype=np.float64)
        for index in range(len(nodes) - 1):
            centerline_distance = _segment_distance(nodes[index], nodes[index + 1], start, end)
            clearances.append(centerline_distance - spec.hose.radius - obstacle.radius)
    return min(clearances, default=float("inf"))


def _tool_clearance(
    tcp_position: np.ndarray,
    gripper_opening_m: float,
    robot: RobotSpec,
    spec: HoseMotionSpec,
) -> float:
    """以保守包覆球近似夾爪到固定管路的最短距離。"""

    gripper = robot.gripper
    tool_radius = max(
        gripper.palm_size[2] / 2.0,
        gripper_opening_m / 2.0 + gripper.finger_size[1],
    )
    clearances = []
    for obstacle in spec.obstacles:
        closest = _closest_point_on_segment(
            tcp_position,
            np.asarray(obstacle.start, dtype=np.float64),
            np.asarray(obstacle.end, dtype=np.float64),
        )
        clearances.append(
            float(np.linalg.norm(tcp_position - closest)) - tool_radius - obstacle.radius
        )
    return min(clearances, default=float("inf"))


def _smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def _interpolated_states(
    spec: HoseMotionSpec,
) -> list[tuple[float, str, np.ndarray, float, bool]]:
    """將關鍵幀轉為固定幀率且速度連續的 TCP 狀態。"""

    first = spec.keyframes[0]
    states = [
        (
            0.0,
            first.phase,
            np.asarray(first.tcp_position, dtype=np.float64),
            first.gripper_opening_m,
            first.attached,
        )
    ]
    time_s = 0.0
    for previous, target in zip(spec.keyframes[:-1], spec.keyframes[1:], strict=True):
        steps = max(1, int(round(target.duration_s * spec.frame_rate_hz)))
        for step in range(1, steps + 1):
            fraction = step / steps
            blend = _smoothstep(fraction)
            previous_position = np.asarray(previous.tcp_position, dtype=np.float64)
            target_position = np.asarray(target.tcp_position, dtype=np.float64)
            tcp_position = previous_position + blend * (target_position - previous_position)
            opening = previous.gripper_opening_m + blend * (
                target.gripper_opening_m - previous.gripper_opening_m
            )
            # 夾取在閉爪階段末端生效，釋放則在開爪階段末端生效。
            attached = previous.attached if step < steps else target.attached
            time_s += target.duration_s / steps
            states.append((time_s, target.phase, tcp_position, opening, attached))
    return states


def simulate_hose_motion(spec: HoseMotionSpec, robot: RobotSpec) -> TrajectoryData:
    """產生可重現的軟管夾取、避障、搬運與放置時間序列。"""

    if any(
        keyframe.gripper_opening_m > robot.gripper.opening + 1e-12
        for keyframe in spec.keyframes
    ):
        raise ValueError("動作 keyframe 的夾爪開口超過機械手規格")

    initial_nodes = _resample_polyline(spec.hose.control_points, spec.hose.node_count)
    rest_lengths = np.linalg.norm(np.diff(initial_nodes, axis=0), axis=1)
    rest_total_length = float(rest_lengths.sum())
    states = _interpolated_states(spec)
    previous_nodes = _solve_hose_step(
        initial_nodes,
        spec,
        rest_lengths,
        states[0][2],
        attached=False,
    )
    previous_angles = np.asarray(
        [link.joint_angle_deg for link in robot.links],
        dtype=np.float64,
    )
    frames: list[TrajectoryFrame] = []

    for time_s, phase, tcp_position, opening, attached in states:
        ik_result = solve_position_ik(robot, tcp_position, previous_angles)
        previous_angles = ik_result.joint_angles_deg
        previous_nodes = _solve_hose_step(
            previous_nodes,
            spec,
            rest_lengths,
            tcp_position,
            attached,
        )
        hose_clearance = _hose_clearance(previous_nodes, spec)
        tool_clearance = _tool_clearance(tcp_position, opening, robot, spec)
        minimum_clearance = min(hose_clearance, tool_clearance)
        current_length = float(np.linalg.norm(np.diff(previous_nodes, axis=0), axis=1).sum())
        frames.append(
            TrajectoryFrame(
                time_s=time_s,
                phase=phase,
                tcp_position=tcp_position.copy(),
                gripper_opening_m=opening,
                attached=attached,
                hose_nodes=previous_nodes.copy(),
                robot_joint_positions=ik_result.joint_positions.copy(),
                joint_angles_deg=ik_result.joint_angles_deg.copy(),
                minimum_clearance_m=minimum_clearance,
                hose_clearance_m=hose_clearance,
                tool_clearance_m=tool_clearance,
                collision=minimum_clearance < -spec.collision_tolerance_m,
                ik_position_error_m=ik_result.position_error_m,
                hose_length_ratio=current_length / rest_total_length,
            )
        )

    clearances = np.asarray([frame.minimum_clearance_m for frame in frames])
    ik_errors = np.asarray([frame.ik_position_error_m for frame in frames])
    length_ratios = np.asarray([frame.hose_length_ratio for frame in frames])
    metrics: dict[str, float | int] = {
        "frame_count": len(frames),
        "duration_s": frames[-1].time_s,
        "minimum_clearance_m": float(clearances.min()),
        "minimum_hose_clearance_m": float(min(frame.hose_clearance_m for frame in frames)),
        "minimum_tool_clearance_m": float(min(frame.tool_clearance_m for frame in frames)),
        "collision_frame_count": int(
            np.count_nonzero(clearances < -spec.collision_tolerance_m)
        ),
        "unsafe_clearance_frame_count": int(
            np.count_nonzero(clearances < spec.safe_clearance_m)
        ),
        "maximum_ik_error_m": float(ik_errors.max()),
        "failed_ik_frame_count": int(np.count_nonzero(ik_errors > 0.002)),
        "maximum_hose_length_error_ratio": float(np.max(np.abs(length_ratios - 1.0))),
        "attached_frame_count": int(sum(frame.attached for frame in frames)),
    }
    return TrajectoryData(spec=spec, frames=tuple(frames), metrics=metrics)
