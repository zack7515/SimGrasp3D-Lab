"""MuJoCo 軟管物理模擬與參數敏感度資料模型。"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

from simgrasp3d.models.motion import TrajectoryData


@dataclass(frozen=True)
class PhysicsVariantSpec:
    """一組用於敏感度比較的物理參數覆寫。"""

    name: str
    timestep_s: float | None = None
    bend_pa: float | None = None
    twist_pa: float | None = None
    friction: float | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PhysicsVariantSpec:
        return cls(
            name=str(data["name"]),
            timestep_s=(
                None if "timestep_s" not in data else float(data["timestep_s"])
            ),
            bend_pa=None if "bend_pa" not in data else float(data["bend_pa"]),
            twist_pa=None if "twist_pa" not in data else float(data["twist_pa"]),
            friction=None if "friction" not in data else float(data["friction"]),
        )


@dataclass(frozen=True)
class MujocoHoseSpec:
    """無視窗 MuJoCo cable baseline 的可重現參數。"""

    timestep_s: float
    settling_s: float
    bend_pa: float
    twist_pa: float
    maximum_velocity_m_s: float
    joint_damping: float
    armature: float
    density_kg_m3: float
    friction: float
    torsional_friction: float
    rolling_friction: float
    attachment_time_constant_s: float
    solver_iterations: int
    sensitivity_variants: tuple[PhysicsVariantSpec, ...]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> MujocoHoseSpec:
        spec = cls(
            timestep_s=float(data.get("timestep_s", 0.002)),
            settling_s=float(data.get("settling_s", 0.4)),
            bend_pa=float(data.get("bend_pa", 4.0e6)),
            twist_pa=float(data.get("twist_pa", 1.0e7)),
            maximum_velocity_m_s=float(data.get("maximum_velocity_m_s", 0.5)),
            joint_damping=float(data.get("joint_damping", 0.03)),
            armature=float(data.get("armature", 0.002)),
            density_kg_m3=float(data.get("density_kg_m3", 850.0)),
            friction=float(data.get("friction", 0.8)),
            torsional_friction=float(data.get("torsional_friction", 0.02)),
            rolling_friction=float(data.get("rolling_friction", 0.001)),
            attachment_time_constant_s=float(
                data.get("attachment_time_constant_s", 0.012)
            ),
            solver_iterations=int(data.get("solver_iterations", 80)),
            sensitivity_variants=tuple(
                PhysicsVariantSpec.from_dict(item)
                for item in data.get("sensitivity_variants", [])
            ),
        )
        positive = (
            spec.timestep_s,
            spec.bend_pa,
            spec.twist_pa,
            spec.maximum_velocity_m_s,
            spec.joint_damping,
            spec.armature,
            spec.density_kg_m3,
            spec.attachment_time_constant_s,
        )
        if min(positive) <= 0.0 or spec.settling_s < 0.0:
            raise ValueError("MuJoCo 時間、材料、密度與求解參數不合法")
        if min(spec.friction, spec.torsional_friction, spec.rolling_friction) < 0.0:
            raise ValueError("MuJoCo 摩擦係數不可為負")
        if spec.solver_iterations <= 0:
            raise ValueError("solver_iterations 必須大於 0")
        for variant in spec.sensitivity_variants:
            values = (
                variant.timestep_s,
                variant.bend_pa,
                variant.twist_pa,
                variant.friction,
            )
            if any(value is not None and value <= 0.0 for value in values):
                raise ValueError(f"敏感度案例 {variant.name} 的覆寫值必須大於 0")
        return spec

    def with_variant(self, variant: PhysicsVariantSpec) -> MujocoHoseSpec:
        """套用一組敏感度覆寫並保留其餘 baseline 參數。"""

        return replace(
            self,
            timestep_s=(
                self.timestep_s
                if variant.timestep_s is None
                else variant.timestep_s
            ),
            bend_pa=self.bend_pa if variant.bend_pa is None else variant.bend_pa,
            twist_pa=(
                self.twist_pa if variant.twist_pa is None else variant.twist_pa
            ),
            friction=(
                self.friction if variant.friction is None else variant.friction
            ),
            sensitivity_variants=(),
        )


@dataclass(frozen=True)
class PhysicsSweepCase:
    """單一物理參數案例的摘要。"""

    name: str
    parameters: dict[str, float]
    metrics: dict[str, float | int]


@dataclass(frozen=True)
class PhysicsSweepData:
    """baseline 物理軌跡與所有敏感度案例。"""

    baseline: TrajectoryData
    cases: tuple[PhysicsSweepCase, ...]
    engine_version: str
