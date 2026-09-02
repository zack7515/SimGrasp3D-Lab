"""Fail-closed 規劃驗證與控制重播資料模型。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from simgrasp3d.models.perception import GraspCandidate


@dataclass(frozen=True)
class IntegrationSpec:
    """在產生控制命令前必須通過的安全門檻。"""

    maximum_ik_position_error_m: float
    maximum_ik_orientation_error_deg: float
    minimum_robot_clearance_m: float
    maximum_unresolved_path_segments: int
    maximum_hose_penetration_frames: int
    maximum_sampled_contact_force_n: float
    maximum_grasp_constraint_error_m: float
    maximum_table_height_error_m: float
    require_feasible_grasp: bool

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> IntegrationSpec:
        spec = cls(
            maximum_ik_position_error_m=float(
                data.get("maximum_ik_position_error_m", 0.002)
            ),
            maximum_ik_orientation_error_deg=float(
                data.get("maximum_ik_orientation_error_deg", 1.0)
            ),
            minimum_robot_clearance_m=float(
                data.get("minimum_robot_clearance_m", 0.005)
            ),
            maximum_unresolved_path_segments=int(
                data.get("maximum_unresolved_path_segments", 0)
            ),
            maximum_hose_penetration_frames=int(
                data.get("maximum_hose_penetration_frames", 0)
            ),
            maximum_sampled_contact_force_n=float(
                data.get("maximum_sampled_contact_force_n", 30.0)
            ),
            maximum_grasp_constraint_error_m=float(
                data.get("maximum_grasp_constraint_error_m", 0.015)
            ),
            maximum_table_height_error_m=float(
                data.get("maximum_table_height_error_m", 0.005)
            ),
            require_feasible_grasp=bool(data.get("require_feasible_grasp", True)),
        )
        floats = (
            spec.maximum_ik_position_error_m,
            spec.maximum_ik_orientation_error_deg,
            spec.minimum_robot_clearance_m,
            spec.maximum_sampled_contact_force_n,
            spec.maximum_grasp_constraint_error_m,
            spec.maximum_table_height_error_m,
        )
        if min(floats) <= 0.0:
            raise ValueError("整合安全門檻必須大於 0")
        if min(
            spec.maximum_unresolved_path_segments,
            spec.maximum_hose_penetration_frames,
        ) < 0:
            raise ValueError("允許的失敗計數不可為負")
        return spec


@dataclass(frozen=True)
class ValidatedGrasp:
    """已通過幾何、IK 與環境距離檢查的抓取候選。"""

    candidate: GraspCandidate
    pregrasp_joint_angles_deg: np.ndarray
    grasp_joint_angles_deg: np.ndarray
    minimum_clearance_m: float
    maximum_ik_position_error_m: float
    maximum_ik_orientation_error_deg: float


@dataclass(frozen=True)
class ReplayEvent:
    """一筆可序列化、可依時間重播的規劃或控制事件。"""

    sequence: int
    time_s: float
    state: str
    event: str
    payload: dict[str, object]


@dataclass(frozen=True)
class ReplayResult:
    """安全閘門、抓取驗證、事件 log 與執行摘要。"""

    execution_authorized: bool
    failure_codes: tuple[str, ...]
    selected_grasp: ValidatedGrasp | None
    events: tuple[ReplayEvent, ...]
    metrics: dict[str, float | int]
