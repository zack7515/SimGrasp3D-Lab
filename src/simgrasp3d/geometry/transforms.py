"""齊次轉換、旋轉與點雲座標運算。"""

from __future__ import annotations

import numpy as np


def translation_matrix(xyz: tuple[float, float, float] | np.ndarray) -> np.ndarray:
    """建立 4×4 平移矩陣。"""

    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return transform


def rotation_matrix(axis: str, angle_deg: float) -> np.ndarray:
    """建立繞局部 x、y 或 z 軸旋轉的 4×4 矩陣。"""

    angle = np.deg2rad(angle_deg)
    cosine = float(np.cos(angle))
    sine = float(np.sin(angle))
    result = np.eye(4, dtype=np.float64)

    if axis == "x":
        result[:3, :3] = np.array(
            [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
            dtype=np.float64,
        )
    elif axis == "y":
        result[:3, :3] = np.array(
            [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
            dtype=np.float64,
        )
    elif axis == "z":
        result[:3, :3] = np.array(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    else:
        raise ValueError(f"不支援的旋轉軸：{axis}")

    return result


def pose_matrix(
    xyz: tuple[float, float, float] | np.ndarray,
    rpy_deg: tuple[float, float, float] | np.ndarray,
) -> np.ndarray:
    """由 XYZ 與固定軸 RPY 角度建立世界姿態矩陣。"""

    roll, pitch, yaw = np.asarray(rpy_deg, dtype=np.float64)
    rotation = (
        rotation_matrix("z", float(yaw))
        @ rotation_matrix("y", float(pitch))
        @ rotation_matrix("x", float(roll))
    )
    rotation[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return rotation


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """將 N×3 點雲套用 4×4 齊次轉換。"""

    points = np.asarray(points, dtype=np.float64)
    transform = np.asarray(transform, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points 必須是 N×3 陣列")
    if transform.shape != (4, 4):
        raise ValueError("transform 必須是 4×4 陣列")

    homogeneous = np.column_stack((points, np.ones(points.shape[0], dtype=np.float64)))
    return (transform @ homogeneous.T).T[:, :3]


def align_z_axis(direction: np.ndarray) -> np.ndarray:
    """建立將局部 z 軸對齊指定方向的 3×3 旋轉矩陣。"""

    direction = np.asarray(direction, dtype=np.float64)
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-12:
        raise ValueError("方向向量長度不可為 0")
    z_axis = direction / norm

    helper = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(helper, z_axis))) > 0.95:
        helper = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    x_axis = np.cross(helper, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))

