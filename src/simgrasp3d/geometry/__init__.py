"""3D 轉換與表面點雲取樣。"""

from .collision import capsule_clearance, capsule_table_clearance, segment_distance
from .sampling import (
    PointCloud,
    sample_box,
    sample_cylinder,
    sample_cylinder_between,
    sample_sphere,
)
from .transforms import (
    matrix_from_quaternion,
    pose_matrix,
    quaternion_from_matrix,
    quaternion_slerp,
    rotation_matrix,
    rotation_vector_from_matrix,
    rpy_deg_from_matrix,
    transform_points,
    translation_matrix,
)

__all__ = [
    "PointCloud",
    "capsule_clearance",
    "capsule_table_clearance",
    "matrix_from_quaternion",
    "pose_matrix",
    "quaternion_from_matrix",
    "quaternion_slerp",
    "rotation_matrix",
    "rotation_vector_from_matrix",
    "rpy_deg_from_matrix",
    "sample_box",
    "sample_cylinder",
    "sample_cylinder_between",
    "sample_sphere",
    "segment_distance",
    "transform_points",
    "translation_matrix",
]
