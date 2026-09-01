"""供軟管、機械臂與路徑規劃共用的解析式距離運算。"""

from __future__ import annotations

import numpy as np


def closest_point_on_segment(
    point: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
) -> np.ndarray:
    """取得有限線段上最靠近指定點的位置。"""

    direction = np.asarray(end, dtype=np.float64) - np.asarray(start, dtype=np.float64)
    squared_length = float(np.dot(direction, direction))
    if squared_length <= 1e-12:
        return np.asarray(start, dtype=np.float64).copy()
    parameter = float(
        np.dot(np.asarray(point, dtype=np.float64) - start, direction) / squared_length
    )
    return np.asarray(start, dtype=np.float64) + np.clip(parameter, 0.0, 1.0) * direction


def segment_distance(
    first_start: np.ndarray,
    first_end: np.ndarray,
    second_start: np.ndarray,
    second_end: np.ndarray,
) -> float:
    """計算兩個有限線段的最短距離，並支援退化成點的線段。"""

    first_start = np.asarray(first_start, dtype=np.float64)
    first_end = np.asarray(first_end, dtype=np.float64)
    second_start = np.asarray(second_start, dtype=np.float64)
    second_end = np.asarray(second_end, dtype=np.float64)
    first_direction = first_end - first_start
    second_direction = second_end - second_start
    first_squared = float(np.dot(first_direction, first_direction))
    second_squared = float(np.dot(second_direction, second_direction))
    if first_squared <= 1e-12:
        return float(
            np.linalg.norm(
                first_start
                - closest_point_on_segment(first_start, second_start, second_end)
            )
        )
    if second_squared <= 1e-12:
        return float(
            np.linalg.norm(
                second_start
                - closest_point_on_segment(second_start, first_start, first_end)
            )
        )

    offset = first_start - second_start
    cross = float(np.dot(first_direction, second_direction))
    first_offset = float(np.dot(first_direction, offset))
    second_offset = float(np.dot(second_direction, offset))
    denominator = first_squared * second_squared - cross * cross
    first_parameter = (
        0.0
        if denominator <= 1e-12
        else float(
            np.clip(
                (cross * second_offset - first_offset * second_squared)
                / denominator,
                0.0,
                1.0,
            )
        )
    )
    second_parameter = float(
        np.clip(
            (cross * first_parameter + second_offset) / second_squared,
            0.0,
            1.0,
        )
    )
    first_parameter = float(
        np.clip(
            (cross * second_parameter - first_offset) / first_squared,
            0.0,
            1.0,
        )
    )
    first_point = first_start + first_parameter * first_direction
    second_point = second_start + second_parameter * second_direction
    return float(np.linalg.norm(first_point - second_point))


def capsule_clearance(
    first_start: np.ndarray,
    first_end: np.ndarray,
    first_radius: float,
    second_start: np.ndarray,
    second_end: np.ndarray,
    second_radius: float,
) -> float:
    """計算兩個膠囊體外表面的有號距離，負值表示穿透。"""

    return (
        segment_distance(first_start, first_end, second_start, second_end)
        - first_radius
        - second_radius
    )


def capsule_table_clearance(
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    table_top_z: float,
) -> float:
    """計算膠囊體最低點到水平桌面的有號距離。"""

    return float(min(start[2], end[2]) - radius - table_top_z)
