"""系統設計教學工作台的參數、閘門與幾何結果。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


_REQUIRED_PARAMETER_KEYS = {
    "camera_height_m",
    "camera_lateral_m",
    "camera_fov_deg",
    "depth_noise_scale",
    "arm_reach_scale",
    "gripper_command_m",
    "hose_radius_m",
    "hose_min_bend_radius_m",
    "grasp_fraction",
    "obstacle_radius_scale",
    "safety_margin_m",
    "lift_height_m",
}


@dataclass(frozen=True)
class DesignParameterSpec:
    """一個可在瀏覽器與 JSON 中調整的設計變數。"""

    key: str
    group: str
    label: str
    value: float
    minimum: float
    maximum: float
    step: float
    unit: str
    description: str

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> DesignParameterSpec:
        minimum = float(data["minimum"])
        maximum = float(data["maximum"])
        value = float(data["value"])
        step = float(data["step"])
        if minimum >= maximum or step <= 0.0 or not minimum <= value <= maximum:
            raise ValueError(f"設計參數 {data.get('key')} 的範圍、步距或預設值不合法")
        return cls(
            key=str(data["key"]),
            group=str(data["group"]),
            label=str(data["label"]),
            value=value,
            minimum=minimum,
            maximum=maximum,
            step=step,
            unit=str(data["unit"]),
            description=str(data["description"]),
        )


@dataclass(frozen=True)
class DesignPresetSpec:
    """用來暴露單一失敗原因的教學參數組。"""

    name: str
    description: str
    values: dict[str, float]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> DesignPresetSpec:
        return cls(
            name=str(data["name"]),
            description=str(data["description"]),
            values={str(key): float(value) for key, value in data.get("values", {}).items()},
        )


@dataclass(frozen=True)
class SystemDesignSpec:
    """一套情境、調參範圍、教學門檻與失敗案例。"""

    name: str
    seed: int
    scenario_summary: str
    parameters: tuple[DesignParameterSpec, ...]
    thresholds: dict[str, float]
    presets: tuple[DesignPresetSpec, ...]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SystemDesignSpec:
        parameters = tuple(DesignParameterSpec.from_dict(item) for item in data["parameters"])
        keys = [item.key for item in parameters]
        if len(keys) != len(set(keys)):
            raise ValueError("設計參數 key 不可重複")
        missing = _REQUIRED_PARAMETER_KEYS - set(keys)
        if missing:
            raise ValueError(f"缺少必要設計參數：{', '.join(sorted(missing))}")
        ranges = {item.key: item for item in parameters}
        presets = tuple(DesignPresetSpec.from_dict(item) for item in data.get("presets", []))
        for preset in presets:
            unknown = set(preset.values) - set(keys)
            if unknown:
                raise ValueError(f"preset {preset.name} 含未知參數：{', '.join(sorted(unknown))}")
            for key, value in preset.values.items():
                parameter = ranges[key]
                if not parameter.minimum <= value <= parameter.maximum:
                    raise ValueError(f"preset {preset.name} 的 {key} 超出調整範圍")
        thresholds = {str(key): float(value) for key, value in data["thresholds"].items()}
        if any(value < 0.0 for value in thresholds.values()):
            raise ValueError("教學門檻不可為負")
        return cls(
            name=str(data["name"]),
            seed=int(data.get("seed", 0)),
            scenario_summary=str(data["scenario_summary"]),
            parameters=parameters,
            thresholds=thresholds,
            presets=presets,
        )

    @property
    def default_values(self) -> dict[str, float]:
        return {item.key: item.value for item in self.parameters}


@dataclass(frozen=True)
class DesignGate:
    """一個可解釋、可調參且不冒充認證的設計檢查。"""

    key: str
    layer: str
    label: str
    value: float
    unit: str
    relation: str
    limit: float
    passed: bool
    explanation: str
    action: str


@dataclass(frozen=True)
class SystemDesignSnapshot:
    """某一組參數的幾何估算、規劃路徑與閘門結果。"""

    values: dict[str, float]
    gates: tuple[DesignGate, ...]
    hose_points: np.ndarray
    grasp_point: np.ndarray
    goal_point: np.ndarray
    planned_path: np.ndarray
    camera_position: np.ndarray
    camera_look_at: np.ndarray
    obstacle_segments: np.ndarray
    obstacle_radii: np.ndarray
    metrics: dict[str, float | int]

    def __post_init__(self) -> None:
        if self.hose_points.ndim != 2 or self.hose_points.shape[1] != 3:
            raise ValueError("hose_points 必須是 [N,3]")
        if self.planned_path.ndim != 2 or self.planned_path.shape[1] != 3:
            raise ValueError("planned_path 必須是 [N,3]")
        arrays = (
            self.hose_points,
            self.grasp_point,
            self.goal_point,
            self.planned_path,
            self.camera_position,
            self.camera_look_at,
            self.obstacle_segments,
            self.obstacle_radii,
        )
        if any(not np.all(np.isfinite(array)) for array in arrays):
            raise ValueError("系統設計結果不可包含 NaN 或 Inf")


@dataclass(frozen=True)
class SystemDesignLabResult:
    """基準設計與所有教學 preset 的可重現結果。"""

    spec: SystemDesignSpec
    baseline: SystemDesignSnapshot
    preset_snapshots: tuple[tuple[DesignPresetSpec, SystemDesignSnapshot], ...]

