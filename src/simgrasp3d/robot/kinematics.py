"""序列式機械手的正向運動學與點雲模型。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simgrasp3d.geometry.sampling import (
    PointCloud,
    sample_box,
    sample_cylinder_between,
    sample_sphere,
)
from simgrasp3d.geometry.transforms import (
    pose_matrix,
    rotation_matrix,
    rotation_vector_from_matrix,
    transform_points,
    translation_matrix,
)
from simgrasp3d.models.specs import RobotSpec


@dataclass(frozen=True)
class RobotState:
    """特定關節角下的機械手幾何與座標系。"""

    point_clouds: tuple[PointCloud, ...]
    joint_positions: np.ndarray
    joint_frames: dict[str, np.ndarray]
    tool_frame: np.ndarray


@dataclass(frozen=True)
class IKResult:
    """逆向運動學的關節解、末端誤差與收斂資訊。"""

    joint_angles_deg: np.ndarray
    joint_positions: np.ndarray
    tool_frame: np.ndarray
    position_error_m: float
    orientation_error_deg: float
    converged: bool
    iterations: int


def forward_kinematics(
    robot: RobotSpec,
    joint_angles_deg: np.ndarray | tuple[float, ...] | None = None,
) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray]:
    """計算每個關節位置、關節座標系與末端工具姿態。"""

    if joint_angles_deg is None:
        angles = np.asarray(
            [link.joint_angle_deg for link in robot.links],
            dtype=np.float64,
        )
    else:
        angles = np.asarray(joint_angles_deg, dtype=np.float64)
    if angles.shape != (len(robot.links),):
        raise ValueError("joint_angles_deg 數量必須等於 robot.links 數量")

    base_transform = pose_matrix(robot.base_pose.xyz, robot.base_pose.rpy_deg)
    current = base_transform @ translation_matrix((0.0, 0.0, robot.base_size[2]))
    positions = [current[:3, 3].copy()]
    frames: dict[str, np.ndarray] = {"robot_base": base_transform.copy()}

    for link, angle_deg in zip(robot.links, angles, strict=True):
        current = current @ rotation_matrix(link.joint_axis, float(angle_deg))
        frames[link.name] = current.copy()
        current = current @ translation_matrix(link.translation)
        positions.append(current[:3, 3].copy())

    frames["flange"] = current.copy()
    tool_frame = current @ translation_matrix(robot.gripper.tcp_offset)
    return np.asarray(positions, dtype=np.float64), frames, tool_frame


def solve_position_ik(
    robot: RobotSpec,
    target_position: np.ndarray | tuple[float, float, float],
    initial_angles_deg: np.ndarray | tuple[float, ...] | None = None,
    *,
    tolerance_m: float = 0.002,
    max_iterations: int = 120,
    damping: float = 0.035,
    maximum_step_deg: float = 7.0,
) -> IKResult:
    """以阻尼最小平方求解 TCP 位置，姿態暫不納入第一階段。"""

    target = np.asarray(target_position, dtype=np.float64)
    if target.shape != (3,):
        raise ValueError("target_position 必須是三維座標")
    if initial_angles_deg is None:
        angles = np.asarray(
            [link.joint_angle_deg for link in robot.links],
            dtype=np.float64,
        )
    else:
        angles = np.asarray(initial_angles_deg, dtype=np.float64).copy()
    if angles.shape != (len(robot.links),):
        raise ValueError("initial_angles_deg 數量必須等於 robot.links 數量")

    lower = np.asarray([link.joint_limits_deg[0] for link in robot.links])
    upper = np.asarray([link.joint_limits_deg[1] for link in robot.links])
    angles = np.clip(angles, lower, upper)
    epsilon_deg = 0.08
    iteration = 0

    for iteration in range(1, max_iterations + 1):
        positions, _, tool_frame = forward_kinematics(robot, angles)
        error = target - tool_frame[:3, 3]
        if float(np.linalg.norm(error)) <= tolerance_m:
            break

        jacobian = np.empty((3, len(angles)), dtype=np.float64)
        for index in range(len(angles)):
            plus = angles.copy()
            minus = angles.copy()
            plus[index] += epsilon_deg
            minus[index] -= epsilon_deg
            _, _, plus_tool = forward_kinematics(robot, plus)
            _, _, minus_tool = forward_kinematics(robot, minus)
            # 導數以弧度為單位，方便限制每輪關節更新幅度。
            denominator = np.deg2rad(2.0 * epsilon_deg)
            jacobian[:, index] = (
                plus_tool[:3, 3] - minus_tool[:3, 3]
            ) / denominator

        regularized = jacobian @ jacobian.T + (damping**2) * np.eye(3)
        delta_rad = jacobian.T @ np.linalg.solve(regularized, error)
        maximum_step_rad = np.deg2rad(maximum_step_deg)
        delta_rad = np.clip(delta_rad, -maximum_step_rad, maximum_step_rad)
        angles = np.clip(angles + np.rad2deg(delta_rad), lower, upper)

    positions, _, tool_frame = forward_kinematics(robot, angles)
    position_error_m = float(np.linalg.norm(target - tool_frame[:3, 3]))
    return IKResult(
        joint_angles_deg=angles,
        joint_positions=positions,
        tool_frame=tool_frame,
        position_error_m=position_error_m,
        orientation_error_deg=0.0,
        converged=position_error_m <= tolerance_m,
        iterations=iteration,
    )


def solve_pose_ik(
    robot: RobotSpec,
    target_position: np.ndarray | tuple[float, float, float],
    target_rotation: np.ndarray,
    initial_angles_deg: np.ndarray | tuple[float, ...] | None = None,
    *,
    position_tolerance_m: float = 0.002,
    orientation_tolerance_deg: float = 1.0,
    max_iterations: int = 160,
    damping: float = 0.02,
    orientation_weight_m_per_rad: float = 0.15,
    maximum_step_deg: float = 5.0,
) -> IKResult:
    """以阻尼最小平方同步求解 TCP 三維位置與旋轉姿態。"""

    target = np.asarray(target_position, dtype=np.float64)
    target_rotation = np.asarray(target_rotation, dtype=np.float64)
    if target.shape != (3,) or target_rotation.shape != (3, 3):
        raise ValueError("target_position 與 target_rotation 尺寸不正確")
    if initial_angles_deg is None:
        angles = np.asarray(
            [link.joint_angle_deg for link in robot.links],
            dtype=np.float64,
        )
    else:
        angles = np.asarray(initial_angles_deg, dtype=np.float64).copy()
    if angles.shape != (len(robot.links),):
        raise ValueError("initial_angles_deg 數量必須等於 robot.links 數量")

    lower = np.asarray([link.joint_limits_deg[0] for link in robot.links])
    upper = np.asarray([link.joint_limits_deg[1] for link in robot.links])
    angles = np.clip(angles, lower, upper)
    epsilon_deg = 0.05
    orientation_tolerance_rad = np.deg2rad(orientation_tolerance_deg)
    iteration = 0

    for iteration in range(1, max_iterations + 1):
        _, _, tool_frame = forward_kinematics(robot, angles)
        position_error = target - tool_frame[:3, 3]
        orientation_error = rotation_vector_from_matrix(
            target_rotation @ tool_frame[:3, :3].T
        )
        if (
            float(np.linalg.norm(position_error)) <= position_tolerance_m
            and float(np.linalg.norm(orientation_error)) <= orientation_tolerance_rad
        ):
            break

        jacobian = np.empty((6, len(angles)), dtype=np.float64)
        denominator = np.deg2rad(2.0 * epsilon_deg)
        for index in range(len(angles)):
            plus = angles.copy()
            minus = angles.copy()
            plus[index] += epsilon_deg
            minus[index] -= epsilon_deg
            _, _, plus_tool = forward_kinematics(robot, plus)
            _, _, minus_tool = forward_kinematics(robot, minus)
            jacobian[:3, index] = (
                plus_tool[:3, 3] - minus_tool[:3, 3]
            ) / denominator
            jacobian[3:, index] = (
                orientation_weight_m_per_rad
                * rotation_vector_from_matrix(
                    plus_tool[:3, :3] @ minus_tool[:3, :3].T
                )
                / denominator
            )

        weighted_error = np.concatenate(
            (position_error, orientation_weight_m_per_rad * orientation_error)
        )
        regularized = jacobian @ jacobian.T + (damping**2) * np.eye(6)
        delta_rad = jacobian.T @ np.linalg.solve(regularized, weighted_error)
        maximum_step_rad = np.deg2rad(maximum_step_deg)
        delta_rad = np.clip(delta_rad, -maximum_step_rad, maximum_step_rad)
        angles = np.clip(angles + np.rad2deg(delta_rad), lower, upper)

    positions, _, tool_frame = forward_kinematics(robot, angles)
    position_error_m = float(np.linalg.norm(target - tool_frame[:3, 3]))
    orientation_error_deg = float(
        np.rad2deg(
            np.linalg.norm(
                rotation_vector_from_matrix(
                    target_rotation @ tool_frame[:3, :3].T
                )
            )
        )
    )
    return IKResult(
        joint_angles_deg=angles,
        joint_positions=positions,
        tool_frame=tool_frame,
        position_error_m=position_error_m,
        orientation_error_deg=orientation_error_deg,
        converged=(
            position_error_m <= position_tolerance_m
            and orientation_error_deg <= orientation_tolerance_deg
        ),
        iterations=iteration,
    )


def _gripper_clouds(
    robot: RobotSpec,
    tool_frame: np.ndarray,
    rng: np.random.Generator,
) -> list[PointCloud]:
    gripper = robot.gripper
    palm_count = max(500, robot.points_per_link // 2)
    finger_count = max(700, robot.points_per_link // 2)

    # TCP 位於兩指尖中央，夾爪本體沿工具座標 -x 往手腕方向延伸。
    palm_x = -gripper.finger_size[0] - gripper.palm_size[0] / 2.0
    palm_local = translation_matrix((palm_x, 0.0, 0.0))
    palm_points = transform_points(
        sample_box(gripper.palm_size, palm_count, rng),
        tool_frame @ palm_local,
    )
    clouds = [PointCloud("gripper_palm", palm_points, gripper.color, "gripper")]

    finger_x = -gripper.finger_size[0] / 2.0
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
