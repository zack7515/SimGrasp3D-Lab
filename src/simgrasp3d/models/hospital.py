"""醫院情境模擬的設定、時間序列與驗證結果資料模型。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

import numpy as np


MetricDirection = Literal["maximum", "minimum", "exact", "informational"]
AssetKind = Literal["box", "zone", "polyline"]
TrackStyle = Literal["markers", "lines", "lines+markers"]


def _vector3(value: list[float] | tuple[float, ...], field_name: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{field_name} 必須包含 3 個數值")
    return (float(value[0]), float(value[1]), float(value[2]))


@dataclass(frozen=True)
class HospitalCaseSpec:
    """一個醫院學習案例及其可調參數。"""

    case_id: str
    order: int
    title: str
    short_title: str
    domain: str
    risk_level: str
    maturity: str
    enabled: bool
    parameters: dict[str, Any]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> HospitalCaseSpec:
        case_id = str(data["id"])
        if not case_id.replace("_", "").isalnum():
            raise ValueError("hospital case id 只可包含英數字與底線")
        order = int(data["order"])
        if order <= 0:
            raise ValueError("hospital case order 必須大於 0")
        risk_level = str(data["risk_level"])
        if risk_level not in {"low", "medium", "high", "very_high"}:
            raise ValueError(f"不支援的醫療風險等級：{risk_level}")
        return cls(
            case_id=case_id,
            order=order,
            title=str(data["title"]),
            short_title=str(data["short_title"]),
            domain=str(data["domain"]),
            risk_level=risk_level,
            maturity=str(data["maturity"]),
            enabled=bool(data.get("enabled", True)),
            parameters=dict(data.get("parameters", {})),
        )


@dataclass(frozen=True)
class HospitalSuiteSpec:
    """醫院多案例模擬套件設定。"""

    name: str
    seed: int
    frame_rate_hz: int
    cases: tuple[HospitalCaseSpec, ...]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> HospitalSuiteSpec:
        frame_rate_hz = int(data.get("frame_rate_hz", 10))
        if frame_rate_hz <= 0:
            raise ValueError("hospital frame_rate_hz 必須大於 0")
        cases = tuple(
            sorted(
                (HospitalCaseSpec.from_dict(item) for item in data["cases"]),
                key=lambda item: item.order,
            )
        )
        case_ids = [item.case_id for item in cases]
        enabled_ids = [item.case_id for item in cases if item.enabled]
        if not enabled_ids:
            raise ValueError("hospital suite 至少需要一個啟用案例")
        if len(case_ids) != len(set(case_ids)):
            raise ValueError("hospital case id 不可重複")
        orders = [item.order for item in cases]
        if len(orders) != len(set(orders)):
            raise ValueError("hospital case order 不可重複")
        return cls(
            name=str(data["name"]),
            seed=int(data.get("seed", 0)),
            frame_rate_hz=frame_rate_hz,
            cases=cases,
        )


@dataclass(frozen=True)
class HospitalAsset:
    """醫院世界中的靜態盒體、語意區域或折線。"""

    name: str
    kind: AssetKind
    color: str
    opacity: float
    center: tuple[float, float, float] | None = None
    size: tuple[float, float, float] | None = None
    points: np.ndarray | None = None
    analysis_only: bool = False

    @classmethod
    def box(
        cls,
        name: str,
        center: list[float] | tuple[float, ...],
        size: list[float] | tuple[float, ...],
        color: str,
        *,
        opacity: float = 0.75,
        analysis_only: bool = False,
        zone: bool = False,
    ) -> HospitalAsset:
        size_value = _vector3(size, f"asset[{name}].size")
        if min(size_value) <= 0.0:
            raise ValueError(f"asset[{name}].size 必須大於 0")
        return cls(
            name=name,
            kind="zone" if zone else "box",
            center=_vector3(center, f"asset[{name}].center"),
            size=size_value,
            color=color,
            opacity=opacity,
            analysis_only=analysis_only,
        )

    @classmethod
    def polyline(
        cls,
        name: str,
        points: np.ndarray,
        color: str,
        *,
        opacity: float = 0.85,
        analysis_only: bool = False,
    ) -> HospitalAsset:
        values = np.asarray(points, dtype=np.float64)
        if values.ndim != 2 or values.shape[1] != 3 or len(values) < 2:
            raise ValueError(f"asset[{name}].points 必須是 N×3 折線")
        return cls(
            name=name,
            kind="polyline",
            points=values,
            color=color,
            opacity=opacity,
            analysis_only=analysis_only,
        )


@dataclass(frozen=True)
class HospitalTrack:
    """同時保存真值與含誤差觀測的可動畫 3D 軌跡。"""

    name: str
    world_positions: np.ndarray
    observed_positions: np.ndarray
    color: str
    style: TrackStyle
    width: float = 6.0
    marker_size: float = 5.0

    def __post_init__(self) -> None:
        world = np.asarray(self.world_positions, dtype=np.float64)
        observed = np.asarray(self.observed_positions, dtype=np.float64)
        if world.ndim != 3 or world.shape[-1] != 3:
            raise ValueError(f"track[{self.name}] 必須是 frame×point×3")
        if observed.shape != world.shape:
            raise ValueError(f"track[{self.name}] 的真值與觀測尺寸必須一致")
        object.__setattr__(self, "world_positions", world)
        object.__setattr__(self, "observed_positions", observed)


@dataclass(frozen=True)
class HospitalMetric:
    """具方向、門檻與校正狀態的案例評估指標。"""

    key: str
    label: str
    value: float
    unit: str
    direction: MetricDirection = "informational"
    limit: float | None = None
    calibrated: bool = False

    @property
    def passed(self) -> bool | None:
        if self.direction == "informational" or self.limit is None:
            return None
        if self.direction == "maximum":
            return self.value <= self.limit
        if self.direction == "minimum":
            return self.value >= self.limit
        return bool(np.isclose(self.value, self.limit))


@dataclass(frozen=True)
class HospitalEvent:
    """動畫時間線中的階段切換或安全事件。"""

    time_s: float
    phase: str
    message: str
    severity: Literal["info", "warning", "stop"] = "info"


@dataclass(frozen=True)
class HospitalCaseResult:
    """一個案例的可重播世界、觀測、訊號、指標與假設。"""

    spec: HospitalCaseSpec
    frame_rate_hz: int
    time_s: np.ndarray
    phases: tuple[str, ...]
    engine: str
    safety_scope: str
    summary: str
    assets: tuple[HospitalAsset, ...]
    tracks: tuple[HospitalTrack, ...]
    signals: dict[str, np.ndarray]
    signal_units: dict[str, str]
    metrics: tuple[HospitalMetric, ...]
    assumptions: tuple[str, ...]
    events: tuple[HospitalEvent, ...]

    def __post_init__(self) -> None:
        time_values = np.asarray(self.time_s, dtype=np.float64)
        if time_values.ndim != 1 or len(time_values) < 2:
            raise ValueError("hospital case 至少需要兩個時間點")
        if len(self.phases) != len(time_values):
            raise ValueError("hospital phases 與 time_s 長度必須一致")
        for track in self.tracks:
            if track.world_positions.shape[0] != len(time_values):
                raise ValueError(f"track[{track.name}] 幀數與 time_s 不一致")
        for name, values in self.signals.items():
            array = np.asarray(values, dtype=np.float64)
            if array.shape != time_values.shape:
                raise ValueError(f"signal[{name}] 長度與 time_s 不一致")
            if not np.all(np.isfinite(array)):
                raise ValueError(f"signal[{name}] 含非有限值")
        object.__setattr__(self, "time_s", time_values)


@dataclass(frozen=True)
class HospitalSuiteResult:
    """依學習順序排列的全部醫院案例結果。"""

    spec: HospitalSuiteSpec
    cases: tuple[HospitalCaseResult, ...]
