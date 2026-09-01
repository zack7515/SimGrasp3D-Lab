"""基礎幾何表面的可重現點雲取樣。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .transforms import align_z_axis, transform_points


@dataclass(frozen=True)
class PointCloud:
    """帶有單一語意名稱與 RGB 顏色的點雲。"""

    name: str
    points: np.ndarray
    color: tuple[float, float, float]
    category: str

    def transformed(self, transform: np.ndarray) -> PointCloud:
        """回傳套用新姿態後的點雲，不修改原始資料。"""

        return PointCloud(
            name=self.name,
            points=transform_points(self.points, transform),
            color=self.color,
            category=self.category,
        )

    @property
    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """回傳世界座標中的最小與最大 XYZ。"""

        return self.points.min(axis=0), self.points.max(axis=0)


def sample_box(size: tuple[float, float, float], count: int, rng: np.random.Generator) -> np.ndarray:
    """依各表面面積比例取樣盒體表面。"""

    if count <= 0:
        raise ValueError("count 必須大於 0")
    half = np.asarray(size, dtype=np.float64) / 2.0
    x_size, y_size, z_size = np.asarray(size, dtype=np.float64)
    face_areas = np.array(
        [y_size * z_size, y_size * z_size, x_size * z_size, x_size * z_size, x_size * y_size, x_size * y_size],
        dtype=np.float64,
    )
    faces = rng.choice(6, size=count, p=face_areas / face_areas.sum())
    points = rng.uniform(-1.0, 1.0, size=(count, 3)) * half

    points[faces == 0, 0] = half[0]
    points[faces == 1, 0] = -half[0]
    points[faces == 2, 1] = half[1]
    points[faces == 3, 1] = -half[1]
    points[faces == 4, 2] = half[2]
    points[faces == 5, 2] = -half[2]
    return points


def sample_cylinder(radius: float, height: float, count: int, rng: np.random.Generator) -> np.ndarray:
    """取樣以原點為中心、局部 z 軸為中心軸的圓柱表面。"""

    if count <= 0:
        raise ValueError("count 必須大於 0")
    side_area = 2.0 * np.pi * radius * height
    cap_area = 2.0 * np.pi * radius**2
    on_side = rng.random(count) < side_area / (side_area + cap_area)
    points = np.empty((count, 3), dtype=np.float64)

    side_count = int(on_side.sum())
    side_angle = rng.uniform(0.0, 2.0 * np.pi, size=side_count)
    points[on_side, 0] = radius * np.cos(side_angle)
    points[on_side, 1] = radius * np.sin(side_angle)
    points[on_side, 2] = rng.uniform(-height / 2.0, height / 2.0, size=side_count)

    cap_mask = ~on_side
    cap_count = int(cap_mask.sum())
    cap_angle = rng.uniform(0.0, 2.0 * np.pi, size=cap_count)
    cap_radius = radius * np.sqrt(rng.random(cap_count))
    points[cap_mask, 0] = cap_radius * np.cos(cap_angle)
    points[cap_mask, 1] = cap_radius * np.sin(cap_angle)
    points[cap_mask, 2] = rng.choice((-height / 2.0, height / 2.0), size=cap_count)
    return points


def sample_sphere(radius: float, count: int, rng: np.random.Generator) -> np.ndarray:
    """以均勻球面分布取樣球體。"""

    if count <= 0:
        raise ValueError("count 必須大於 0")
    z_unit = rng.uniform(-1.0, 1.0, size=count)
    angle = rng.uniform(0.0, 2.0 * np.pi, size=count)
    radial = np.sqrt(1.0 - z_unit**2)
    return radius * np.column_stack((radial * np.cos(angle), radial * np.sin(angle), z_unit))


def sample_cylinder_between(
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    count: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """在兩個世界座標點之間建立圓柱表面點雲。"""

    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length <= 1e-12:
        raise ValueError("連桿起點與終點不可相同")

    local_points = sample_cylinder(radius, length, count, rng)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = align_z_axis(direction)
    transform[:3, 3] = (start + end) / 2.0
    return transform_points(local_points, transform)
