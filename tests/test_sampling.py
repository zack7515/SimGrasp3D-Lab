"""幾何表面點雲取樣測試。"""

import numpy as np

from simgrasp3d.geometry.sampling import sample_box, sample_cylinder, sample_sphere


def test_box_points_lie_on_surface() -> None:
    rng = np.random.default_rng(42)
    size = np.array([0.4, 0.2, 0.1])
    points = sample_box(tuple(size), 3000, rng)
    normalized = np.abs(points) / (size / 2.0)
    assert np.all(normalized <= 1.0 + 1e-12)
    assert np.all(np.any(np.isclose(normalized, 1.0, atol=1e-12), axis=1))


def test_sphere_points_have_requested_radius() -> None:
    rng = np.random.default_rng(7)
    points = sample_sphere(0.075, 2000, rng)
    np.testing.assert_allclose(np.linalg.norm(points, axis=1), 0.075, atol=1e-12)


def test_cylinder_points_respect_bounds() -> None:
    rng = np.random.default_rng(9)
    points = sample_cylinder(0.08, 0.20, 2500, rng)
    radial = np.linalg.norm(points[:, :2], axis=1)
    assert np.all(radial <= 0.08 + 1e-12)
    assert np.all(np.abs(points[:, 2]) <= 0.10 + 1e-12)

