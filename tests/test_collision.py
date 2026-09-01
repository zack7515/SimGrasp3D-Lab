"""線段與膠囊體解析式距離測試。"""

import numpy as np

from simgrasp3d.geometry.collision import (
    capsule_clearance,
    capsule_table_clearance,
    segment_distance,
)


def test_segment_distance_supports_crossing_parallel_and_degenerate_cases() -> None:
    assert segment_distance(
        np.asarray([-1.0, 0.0, 0.0]),
        np.asarray([1.0, 0.0, 0.0]),
        np.asarray([0.0, -1.0, 0.0]),
        np.asarray([0.0, 1.0, 0.0]),
    ) == 0.0
    assert abs(
        segment_distance(
            np.asarray([0.0, 0.0, 0.0]),
            np.asarray([1.0, 0.0, 0.0]),
            np.asarray([0.0, 2.0, 0.0]),
            np.asarray([1.0, 2.0, 0.0]),
        )
        - 2.0
    ) < 1e-12
    assert abs(
        segment_distance(
            np.zeros(3),
            np.zeros(3),
            np.asarray([3.0, 0.0, 0.0]),
            np.asarray([3.0, 1.0, 0.0]),
        )
        - 3.0
    ) < 1e-12


def test_capsule_clearance_is_signed_and_table_uses_lowest_endpoint() -> None:
    clearance = capsule_clearance(
        np.asarray([0.0, 0.0, 0.0]),
        np.asarray([1.0, 0.0, 0.0]),
        0.6,
        np.asarray([0.0, 1.0, 0.0]),
        np.asarray([1.0, 1.0, 0.0]),
        0.5,
    )
    assert abs(clearance + 0.1) < 1e-12
    assert abs(
        capsule_table_clearance(
            np.asarray([0.0, 0.0, 0.4]),
            np.asarray([0.0, 0.0, 0.8]),
            0.1,
            0.2,
        )
        - 0.1
    ) < 1e-12
