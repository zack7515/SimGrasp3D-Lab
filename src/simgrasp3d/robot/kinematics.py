"""序列式機械手的正向運動學與點雲模型。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simgrasp3d.geometry.sampling import PointCloud, sample_box, sample_cylinder_between, sample_sphere
from simgrasp3d.geometry.transforms import pose_matrix, rotation_matrix, transform_points, translation_matrix
from simgrasp3d.models.specs import RobotSpec


@dataclass(frozen=True)
class RobotState:
    """特定關節角下的機械手幾何與座標系。"""

    point_clouds: tuple[PointCloud, ...]
    joint_positions: np.ndarray
    joint_frames: dict[str, np.ndarray]
    tool_frame: np.ndarray


def forward_kinematics(robot: RobotSpec) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray]:
    """計算每個關節位置、關節座標系與末端工具姿態。"""

    base_transform = pose_matrix(robot.base_pose.xyz, robot.base_pose.rpy_deg)
    current = base_transform @ translation_matrix((0.0, 0.0, robot.base_size[2]))
    positions = [current[:3, 3].copy()]
    frames: dict[str, np.ndarray] = {"robot_base": base_transform.copy()}

    for link in robot.links:
        current = current @ rotation_matrix(link.joint_axis, link.joint_angle_deg)
        frames[link.name] = current.copy()
        current = current @ translation_matrix(link.translation)
        positions.append(current[:3, 3].copy())

    return np.asarray(positions, dtype=np.float64), frames, current.copy()


def _gripper_clouds(
    robot: RobotSpec,
    tool_frame: np.ndarray,
    rng: np.random.Generator,
) -> list[PointCloud]:
    gripper = robot.gripper
    palm_count = max(500, robot.points_per_link // 2)
    finger_count = max(700, robot.points_per_link // 2)

    palm_local = translation_matrix((gripper.palm_size[0] / 2.0, 0.0, 0.0))
    palm_points = transform_points(
        sample_box(gripper.palm_size, palm_count, rng),
        tool_frame @ palm_local,
    )
    clouds = [PointCloud("gripper_palm", palm_points, gripper.color, "gripper")]

    finger_x = gripper.palm_size[0] + gripper.finger_size[0] / 2.0
    finger_y = gripper.opening / 2.0 + gripper.finger_size[1] / 2.0
    for side, y_offset in (("left", finger_y), ("right", -finger_y)):
        finger_local = translation_matrix((finger_x, y_offset, 0.0))
        finger_points = transform_points(
            sample_box(gripper.finger_size, finger_count, rng),
            tool_frame @ finger_local,
        )
        clouds.append(PointCloud(f"gripper_{side}_finger", finger_points, gripper.color, "gripper"))

    return clouds


def build_robot_state(robot: RobotSpec, rng: np.random.Generator) -> RobotState:
    """由規格建立機械手本體、關節與夾爪點雲。"""

    joint_positions, frames, tool_frame = forward_kinematics(robot)
    base_transform = pose_matrix(robot.base_pose.xyz, robot.base_pose.rpy_deg)
    base_center = translation_matrix((0.0, 0.0, robot.base_size[2] / 2.0))
    base_points = transform_points(
        sample_box(robot.base_size, robot.points_per_link * 2, rng),
        base_transform @ base_center,
    )
    clouds: list[PointCloud] = [
        PointCloud(f"{robot.name}_base", base_points, robot.base_color, "robot")
    ]

    link_color = (0.72, 0.75, 0.80)
    joint_color = (0.94, 0.75, 0.16)
    for index, link in enumerate(robot.links):
        start = joint_positions[index]
        end = joint_positions[index + 1]
        segment_length = float(np.linalg.norm(end - start))
        if segment_length > 1e-9:
            link_points = sample_cylinder_between(
                start,
                end,
                link.radius,
                robot.points_per_link,
                rng,
            )
            clouds.append(PointCloud(link.name, link_points, link_color, "robot_link"))

        joint_radius = max(link.radius * 1.18, 0.022)
        joint_points = sample_sphere(joint_radius, max(350, robot.points_per_link // 3), rng)
        joint_points += start
        clouds.append(PointCloud(f"{link.name}_joint", joint_points, joint_color, "robot_joint"))

    clouds.extend(_gripper_clouds(robot, tool_frame, rng))
    frames["tool"] = tool_frame.copy()
    return RobotState(tuple(clouds), joint_positions, frames, tool_frame)

