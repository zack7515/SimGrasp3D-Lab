"""軟管夾取情境與逐幀運動資料模型。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


Vector3 = tuple[float, float, float]
Color = tuple[float, float, float]


def _vector3(value: list[float] | tuple[float, ...], field_name: str) -> Vector3:
    if len(value) != 3:
        raise ValueError(f"{field_name} 必須包含 3 個數值")
    return (float(value[0]), float(value[1]), float(value[2]))


def _color(value: list[float] | tuple[float, ...], field_name: str) -> Color:
    result = _vector3(value, field_name)
    if any(channel < 0.0 or channel > 1.0 for channel in result):
        raise ValueError(f"{field_name} 必須位於 0 到 1 之間")
    return result


@dataclass(frozen=True)
class HoseSpec:
    """以中心線節點近似的軟管規格。"""

    name: str
    radius: float
    node_count: int
    grasp_node_index: int
    control_points: tuple[Vector3, ...]
    color: Color
    constraint_iterations: int
    follow_decay_nodes: float
    gravity_step_m: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> HoseSpec:
        node_count = int(data.get("node_count", 31))
        grasp_node_index = int(data["grasp_node_index"])
        control_points = tuple(
            _vector3(value, "hose.control_points") for value in data["control_points"]
        )
        if node_count < 3:
            raise ValueError("hose.node_count 至少為 3")
        if not 0 <= grasp_node_index < node_count:
            raise ValueError("hose.grasp_node_index 超出節點範圍")
        if len(control_points) < 2:
            raise ValueError("hose.control_points 至少需要兩點")
        radius = float(data["radius"])
        if radius <= 0.0:
            raise ValueError("hose.radius 必須大於 0")
        constraint_iterations = int(data.get("constraint_iterations", 12))
        follow_decay_nodes = float(data.get("follow_decay_nodes", 7.0))
        gravity_step_m = float(data.get("gravity_step_m", 0.002))
        if constraint_iterations <= 0 or follow_decay_nodes <= 0.0 or gravity_step_m < 0.0:
            raise ValueError("軟管求解參數不合法")
        return cls(
            name=str(data.get("name", "hose")),
            radius=radius,
            node_count=node_count,
            grasp_node_index=grasp_node_index,
            control_points=control_points,
            color=_color(data.get("color", [0.08, 0.62, 0.67]), "hose.color"),
            constraint_iterations=constraint_iterations,
            follow_decay_nodes=follow_decay_nodes,
            gravity_step_m=gravity_step_m,
        )


@dataclass(frozen=True)
class PipeObstacleSpec:
    """以有限長圓柱表示的固定管路障礙。"""

    name: str
    start: Vector3
    end: Vector3
    radius: float
    color: Color

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PipeObstacleSpec:
        start = _vector3(data["start"], "obstacle.start")
        end = _vector3(data["end"], "obstacle.end")
        if np.linalg.norm(np.asarray(end) - np.asarray(start)) <= 1e-9:
            raise ValueError("固定管路的 start 與 end 不可相同")
        radius = float(data["radius"])
        if radius <= 0.0:
            raise ValueError("固定管路半徑必須大於 0")
        return cls(
            name=str(data["name"]),
            start=start,
            end=end,
            radius=radius,
            color=_color(data.get("color", [0.31, 0.40, 0.47]), "obstacle.color"),
        )


@dataclass(frozen=True)
class MotionKeyframeSpec:
    """一個動作階段結束時的 TCP、夾爪與附著狀態。"""

    phase: str
    duration_s: float
    tcp_position: Vector3
    gripper_opening_m: float
    attached: bool

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> MotionKeyframeSpec:
        duration_s = float(data.get("duration_s", 0.0))
        gripper_opening_m = float(data["gripper_opening_m"])
        if duration_s < 0.0:
            raise ValueError("keyframe.duration_s 不可小於 0")
        if gripper_opening_m <= 0.0:
            raise ValueError("keyframe.gripper_opening_m 必須大於 0")
        return cls(
            phase=str(data["phase"]),
            duration_s=duration_s,
            tcp_position=_vector3(data["tcp_position"], "keyframe.tcp_position"),
            gripper_opening_m=gripper_opening_m,
            attached=bool(data.get("attached", False)),
        )


@dataclass(frozen=True)
class HoseMotionSpec:
    """完整軟管抽取與搬運示範設定。"""

    name: str
    frame_rate_hz: int
    table_top_z: float
    safe_clearance_m: float
    collision_tolerance_m: float
    target_position: Vector3
    target_radius_m: float
    hose: HoseSpec
    obstacles: tuple[PipeObstacleSpec, ...]
    keyframes: tuple[MotionKeyframeSpec, ...]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> HoseMotionSpec:
        frame_rate_hz = int(data.get("frame_rate_hz", 12))
        safe_clearance_m = float(data.get("safe_clearance_m", 0.015))
        collision_tolerance_m = float(data.get("collision_tolerance_m", 0.00025))
        target_radius_m = float(data.get("target_radius_m", 0.06))
        keyframes = tuple(MotionKeyframeSpec.from_dict(item) for item in data["keyframes"])
        obstacles = tuple(PipeObstacleSpec.from_dict(item) for item in data["obstacles"])
        if frame_rate_hz <= 0:
            raise ValueError("frame_rate_hz 必須大於 0")
        if (
            safe_clearance_m < 0.0
            or collision_tolerance_m < 0.0
            or target_radius_m <= 0.0
        ):
            raise ValueError("安全距離與碰撞容差不可為負，目標半徑必須大於 0")
        if not obstacles:
            raise ValueError("obstacles 至少需要一個固定管路")
        if len(keyframes) < 2 or keyframes[0].duration_s != 0.0:
            raise ValueError("至少需要兩個 keyframe，且第一個 duration_s 必須為 0")
        return cls(
            name=str(data["name"]),
            frame_rate_hz=frame_rate_hz,
            table_top_z=float(data["table_top_z"]),
            safe_clearance_m=safe_clearance_m,
            collision_tolerance_m=collision_tolerance_m,
            target_position=_vector3(data["target_position"], "target_position"),
            target_radius_m=target_radius_m,
            hose=HoseSpec.from_dict(data["hose"]),
            obstacles=obstacles,
            keyframes=keyframes,
        )


@dataclass(frozen=True)
class TrajectoryFrame:
    """動畫中一個時間點的可檢查狀態。"""

    time_s: float
    phase: str
    tcp_position: np.ndarray
    gripper_opening_m: float
    attached: bool
    hose_nodes: np.ndarray
    robot_joint_positions: np.ndarray
    joint_angles_deg: np.ndarray
    minimum_clearance_m: float
    hose_clearance_m: float
    tool_clearance_m: float
    collision: bool
    ik_position_error_m: float
    hose_length_ratio: float


@dataclass(frozen=True)
class TrajectoryData:
    """軟管連續動作的規格、逐幀結果與摘要指標。"""

    spec: HoseMotionSpec
    frames: tuple[TrajectoryFrame, ...]
    metrics: dict[str, float | int]
