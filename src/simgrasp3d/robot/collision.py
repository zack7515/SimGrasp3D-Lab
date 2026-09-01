"""依機械臂與夾爪實際尺寸計算環境碰撞距離。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simgrasp3d.geometry.collision import capsule_clearance, capsule_table_clearance
from simgrasp3d.models.motion import PipeObstacleSpec
from simgrasp3d.models.specs import RobotSpec


@dataclass(frozen=True)
class CapsulePart:
    """以中心線與半徑保守包覆一個機器人零件。"""

    name: str
    category: str
    start: np.ndarray
    end: np.ndarray
    radius: float


@dataclass(frozen=True)
class RobotClearanceResult:
    """機械臂、夾爪到固定管路與桌面的最小距離。"""

    minimum_clearance_m: float
    link_clearance_m: float
    gripper_clearance_m: float
    closest_pair: str
    parts: tuple[CapsulePart, ...]


def _world_point(tool_frame: np.ndarray, local_point: tuple[float, float, float]) -> np.ndarray:
    return tool_frame[:3, :3] @ np.asarray(local_point) + tool_frame[:3, 3]


def build_robot_capsules(
    robot: RobotSpec,
    joint_positions: np.ndarray,
    tool_frame: np.ndarray,
    gripper_opening_m: float,
) -> tuple[CapsulePart, ...]:
    """建立連桿、關節、手掌與兩根手指的保守膠囊體。"""

    parts: list[CapsulePart] = []
    for index, link in enumerate(robot.links):
        parts.append(
            CapsulePart(
                name=link.name,
                category="link",
                start=joint_positions[index].copy(),
                end=joint_positions[index + 1].copy(),
                radius=link.radius,
            )
        )
        joint_radius = max(link.radius * 1.18, 0.022)
        parts.append(
            CapsulePart(
                name=f"{link.name}_joint",
                category="link",
                start=joint_positions[index].copy(),
                end=joint_positions[index].copy(),
                radius=joint_radius,
            )
        )

    gripper = robot.gripper
    finger_length = gripper.finger_size[0]
    palm_length = gripper.palm_size[0]
    palm_radius = float(np.hypot(gripper.palm_size[1], gripper.palm_size[2]) / 2.0)
    finger_radius = float(
        np.hypot(gripper.finger_size[1], gripper.finger_size[2]) / 2.0
    )
    parts.append(
        CapsulePart(
            name="gripper_palm",
            category="gripper",
            start=_world_point(tool_frame, (-finger_length - palm_length, 0.0, 0.0)),
            end=_world_point(tool_frame, (-finger_length, 0.0, 0.0)),
            radius=palm_radius,
        )
    )
    finger_y = gripper_opening_m / 2.0 + gripper.finger_size[1] / 2.0
    for side, y_offset in (("left", finger_y), ("right", -finger_y)):
        parts.append(
            CapsulePart(
                name=f"gripper_{side}_finger",
                category="gripper",
                start=_world_point(tool_frame, (-finger_length, y_offset, 0.0)),
                end=_world_point(tool_frame, (0.0, y_offset, 0.0)),
                radius=finger_radius,
            )
        )
    return tuple(parts)


def evaluate_robot_clearance(
    robot: RobotSpec,
    joint_positions: np.ndarray,
    tool_frame: np.ndarray,
    gripper_opening_m: float,
    obstacles: tuple[PipeObstacleSpec, ...],
    table_top_z: float,
) -> RobotClearanceResult:
    """計算所有機器人膠囊體到環境的最小有號距離。"""

    parts = build_robot_capsules(
        robot,
        joint_positions,
        tool_frame,
        gripper_opening_m,
    )
    link_clearance = float("inf")
    gripper_clearance = float("inf")
    minimum_clearance = float("inf")
    closest_pair = ""
    for part in parts:
        candidates = [
            (
                "table",
                capsule_table_clearance(
                    part.start,
                    part.end,
                    part.radius,
                    table_top_z,
                ),
            )
        ]
        for obstacle in obstacles:
            candidates.append(
                (
                    obstacle.name,
                    capsule_clearance(
                        part.start,
                        part.end,
                        part.radius,
                        np.asarray(obstacle.start),
                        np.asarray(obstacle.end),
                        obstacle.radius,
                    ),
                )
            )
        environment_name, part_clearance = min(candidates, key=lambda item: item[1])
        if part.category == "gripper":
            gripper_clearance = min(gripper_clearance, part_clearance)
        else:
            link_clearance = min(link_clearance, part_clearance)
        if part_clearance < minimum_clearance:
            minimum_clearance = part_clearance
            closest_pair = f"{part.name} ↔ {environment_name}"
    return RobotClearanceResult(
        minimum_clearance_m=minimum_clearance,
        link_clearance_m=link_clearance,
        gripper_clearance_m=gripper_clearance,
        closest_pair=closest_pair,
        parts=parts,
    )
