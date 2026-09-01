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


def rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """將 3×3 旋轉矩陣轉成軸角旋轉向量，向量長度為弧度。"""

    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation 必須是 3×3 陣列")
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle <= 1e-10:
        return np.zeros(3, dtype=np.float64)
    if np.pi - angle <= 1e-5:
        axis = np.sqrt(np.maximum((np.diag(rotation) + 1.0) / 2.0, 0.0))
        axis[1] = np.copysign(axis[1], rotation[0, 1] + rotation[1, 0])
        axis[2] = np.copysign(axis[2], rotation[0, 2] + rotation[2, 0])
        norm = float(np.linalg.norm(axis))
        axis = np.asarray([1.0, 0.0, 0.0]) if norm <= 1e-10 else axis / norm
        return axis * angle
    skew = np.asarray(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    return skew * (angle / (2.0 * np.sin(angle)))


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """將 3×3 旋轉矩陣轉為 `[w, x, y, z]` 單位四元數。"""

    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation 必須是 3×3 陣列")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        following = (index + 1) % 3
        remaining = (index + 2) % 3
        scale = np.sqrt(
            1.0
            + rotation[index, index]
            - rotation[following, following]
            - rotation[remaining, remaining]
        ) * 2.0
        vector = np.zeros(3, dtype=np.float64)
        vector[index] = 0.25 * scale
        vector[following] = (
            rotation[following, index] + rotation[index, following]
        ) / scale
        vector[remaining] = (
            rotation[remaining, index] + rotation[index, remaining]
        ) / scale
        quaternion = np.asarray(
            [
                (rotation[remaining, following] - rotation[following, remaining])
                / scale,
                vector[0],
                vector[1],
                vector[2],
            ]
        )
    return quaternion / np.linalg.norm(quaternion)


def matrix_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    """將 `[w, x, y, z]` 單位四元數轉為 3×3 旋轉矩陣。"""

    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,):
        raise ValueError("quaternion 必須包含 4 個數值")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("quaternion 長度不可為 0")
    w, x, y, z = quaternion / norm
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def quaternion_slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    """沿最短旋轉路徑進行單位四元數球面線性插值。"""

    first = np.asarray(first, dtype=np.float64).copy()
    second = np.asarray(second, dtype=np.float64).copy()
    if first.shape != (4,) or second.shape != (4,):
        raise ValueError("四元數必須包含 4 個數值")
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if first_norm <= 1e-12 or second_norm <= 1e-12:
        raise ValueError("四元數長度不可為 0")
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction 必須位於 0 到 1 之間")
    first /= first_norm
    second /= second_norm
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = first + fraction * (second - first)
        return result / np.linalg.norm(result)
    angle = float(np.arccos(dot))
    sine = float(np.sin(angle))
    return (
        np.sin((1.0 - fraction) * angle) / sine * first
        + np.sin(fraction * angle) / sine * second
    )


def rpy_deg_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """將旋轉矩陣轉為固定軸 roll、pitch、yaw 角度。"""

    rotation = np.asarray(rotation, dtype=np.float64)
    pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(float(np.cos(pitch))) > 1e-8:
        roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    else:
        roll = 0.0
        yaw = float(np.arctan2(-rotation[0, 1], rotation[1, 1]))
    return np.rad2deg(np.asarray([roll, pitch, yaw]))
