"""齊次轉換與座標運算測試。"""

import numpy as np

from simgrasp3d.geometry.transforms import pose_matrix, rotation_matrix, transform_points


def test_translation_is_applied_to_points() -> None:
    transform = pose_matrix((1.0, 2.0, 3.0), (0.0, 0.0, 0.0))
    result = transform_points(np.array([[0.2, -0.1, 0.5]]), transform)
    np.testing.assert_allclose(result, [[1.2, 1.9, 3.5]], atol=1e-12)


def test_z_rotation_maps_x_to_y() -> None:
    result = transform_points(np.array([[1.0, 0.0, 0.0]]), rotation_matrix("z", 90.0))
    np.testing.assert_allclose(result, [[0.0, 1.0, 0.0]], atol=1e-12)


def test_pose_rotation_uses_roll_pitch_yaw_order() -> None:
    transform = pose_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 90.0))
    np.testing.assert_allclose(transform[:3, :3], rotation_matrix("z", 90.0)[:3, :3])

