"""建立可重現的醫院機器人教學案例與量化結果。"""

from __future__ import annotations

import itertools
from collections.abc import Callable
from pathlib import Path

import numpy as np

from simgrasp3d.io import load_spec
from simgrasp3d.models.hospital import (
    HospitalAsset,
    HospitalCaseResult,
    HospitalCaseSpec,
    HospitalEvent,
    HospitalMetric,
    HospitalSuiteResult,
    HospitalSuiteSpec,
    HospitalTrack,
)

GRAPHITE = "#23323A"
STEEL = "#71828A"
SURGICAL_BLUE = "#227C9D"
MONITOR_TEAL = "#0A9B83"
PULSE_CORAL = "#D95D50"
IODINE = "#D89A35"
STERILE = "#DDE9E7"
VIOLET = "#745B9E"


def load_hospital_suite_spec(path: str | Path) -> HospitalSuiteSpec:
    """讀取並驗證 UTF-8 醫院案例設定。"""

    return load_spec(path, HospitalSuiteSpec)


def _smoothstep(value: np.ndarray) -> np.ndarray:
    return value * value * (3.0 - 2.0 * value)


def _timeline(
    keyframes: tuple[tuple[str, float, tuple[float, float, float]], ...],
    frame_rate_hz: int,
) -> tuple[np.ndarray, tuple[str, ...], np.ndarray]:
    """把階段關鍵幀轉為固定頻率且端點不重複的平滑軌跡。"""

    if len(keyframes) < 2 or keyframes[0][1] != 0.0:
        raise ValueError("案例至少需要兩個 keyframe，第一幀 duration 必須為 0")
    times = [0.0]
    phases = [keyframes[0][0]]
    positions = [np.asarray(keyframes[0][2], dtype=np.float64)]
    elapsed = 0.0
    for previous, target in itertools.pairwise(keyframes):
        duration = float(target[1])
        steps = max(1, int(round(duration * frame_rate_hz)))
        fractions = np.arange(1, steps + 1, dtype=np.float64) / steps
        start = np.asarray(previous[2], dtype=np.float64)
        end = np.asarray(target[2], dtype=np.float64)
        for step, blend in enumerate(_smoothstep(fractions), start=1):
            positions.append(start + blend * (end - start))
            times.append(elapsed + step / frame_rate_hz)
            phases.append(target[0])
        elapsed += duration
    return np.asarray(times), tuple(phases), np.asarray(positions)


def _observed(world: np.ndarray, rng: np.random.Generator, noise_m: float) -> np.ndarray:
    return world + rng.normal(0.0, noise_m, size=world.shape)


def _track(
    name: str,
    positions: np.ndarray,
    rng: np.random.Generator,
    noise_m: float,
    color: str,
    *,
    style: str = "lines+markers",
    width: float = 7.0,
    marker_size: float = 5.0,
) -> HospitalTrack:
    values = np.asarray(positions, dtype=np.float64)
    if values.ndim == 2:
        values = values[:, None, :]
    return HospitalTrack(
        name=name,
        world_positions=values,
        observed_positions=_observed(values, rng, noise_m),
        color=color,
        style=style,  # type: ignore[arg-type]
        width=width,
        marker_size=marker_size,
    )


def _events(time_s: np.ndarray, phases: tuple[str, ...]) -> tuple[HospitalEvent, ...]:
    result = [HospitalEvent(float(time_s[0]), phases[0], f"進入「{phases[0]}」階段")]
    previous = phases[0]
    for index, phase in enumerate(phases[1:], start=1):
        if phase != previous:
            result.append(HospitalEvent(float(time_s[index]), phase, f"進入「{phase}」階段"))
            previous = phase
    return tuple(result)


def _parameter(spec: HospitalCaseSpec, name: str, default: float) -> float:
    return float(spec.parameters.get(name, default))


def _resample(points: np.ndarray, count: int) -> np.ndarray:
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    samples = np.linspace(0.0, cumulative[-1], count)
    return np.column_stack(
        [np.interp(samples, cumulative, points[:, axis]) for axis in range(3)]
    )


def _bend_radius(nodes: np.ndarray) -> float:
    radii: list[float] = []
    for first, middle, last in zip(nodes[:-2], nodes[1:-1], nodes[2:], strict=True):
        a = float(np.linalg.norm(middle - first))
        b = float(np.linalg.norm(last - middle))
        c = float(np.linalg.norm(last - first))
        area_twice = float(np.linalg.norm(np.cross(middle - first, last - first)))
        if area_twice > 1e-10:
            radii.append(a * b * c / (2.0 * area_twice))
    return min(radii, default=10.0)


def _specimen_transfer(
    spec: HospitalCaseSpec, frame_rate_hz: int, rng: np.random.Generator
) -> HospitalCaseResult:
    noise_m = _parameter(spec, "observation_noise_m", 0.0012)
    target = np.asarray([0.48, 0.12, 0.16])
    time_s, phases, tcp = _timeline(
        (
            ("待命與身分確認", 0.0, (-0.62, -0.35, 0.62)),
            ("移至指定檢體", 1.0, (-0.42, -0.15, 0.30)),
            ("垂直取管", 0.6, (-0.42, -0.15, 0.18)),
            ("夾取與條碼確認", 0.5, (-0.42, -0.15, 0.18)),
            ("保持直立抬升", 0.8, (-0.42, -0.15, 0.46)),
            ("避開相鄰試管", 1.0, (0.02, -0.30, 0.52)),
            ("移至分析盤", 1.1, (0.48, 0.12, 0.34)),
            ("垂直放置", 0.7, (0.48, 0.12, 0.22)),
            ("釋放並覆核", 0.4, (0.48, 0.12, 0.22)),
        ),
        frame_rate_hz,
    )
    payload = np.repeat(np.asarray([[[-0.42, -0.15, 0.14]]]), len(time_s), axis=0)
    grasped = np.asarray([p not in {"待命與身分確認", "移至指定檢體", "垂直取管"} for p in phases])
    released = np.asarray([p == "釋放並覆核" for p in phases])
    payload[grasped, 0] = tcp[grasped] + np.asarray([0.0, 0.0, -0.04])
    payload[released, 0] = target
    acceleration = np.vstack((np.zeros((2, 3)), np.diff(tcp, n=2, axis=0))) * frame_rate_hz**2
    tilt = np.minimum(8.0, np.linalg.norm(acceleration[:, :2], axis=1) * 0.11)
    neighbors = np.asarray([[-0.35, -0.15, 0.14], [-0.49, -0.15, 0.14]])
    clearance = float(np.min(np.linalg.norm(payload[grasped, 0, None] - neighbors, axis=2))) - 0.022
    assets = (
        HospitalAsset.box("檢體工作台", (0, 0, 0), (1.5, 0.9, 0.08), STERILE),
        HospitalAsset.box("來源試管架", (-0.42, -0.15, 0.10), (0.28, 0.18, 0.12), STEEL),
        HospitalAsset.box("分析盤", (0.48, 0.12, 0.10), (0.34, 0.26, 0.12), SURGICAL_BLUE),
        HospitalAsset.box("指定槽位", target, (0.07, 0.07, 0.05), MONITOR_TEAL, opacity=0.28, analysis_only=True, zone=True),
    )
    metrics = (
        HospitalMetric("final_position_error_m", "最終槽位誤差", float(np.linalg.norm(payload[-1, 0] - target)), "m", "maximum", _parameter(spec, "maximum_final_error_m", 0.006)),
        HospitalMetric("maximum_tilt_deg", "最大試管傾角代理", float(np.max(tilt)), "deg", "maximum", _parameter(spec, "maximum_tilt_deg", 10.0)),
        HospitalMetric("minimum_neighbor_clearance_m", "相鄰試管最小距離", clearance, "m", "minimum", _parameter(spec, "minimum_clearance_m", 0.025)),
        HospitalMetric("identity_mismatch_count", "檢體身分不符", 0, "count", "exact", 0),
    )
    return HospitalCaseResult(
        spec, frame_rate_hz, time_s, phases, "NumPy 平滑軌跡與幾何間距",
        "封閉試管、無液體飛濺、無病患接觸",
        "依指定身分取出單一封閉試管，保持直立並放入分析盤槽位。",
        assets,
        (_track("TCP", tcp, rng, noise_m, IODINE), _track("檢體試管", payload, rng, noise_m * 0.7, PULSE_CORAL, style="markers", marker_size=9)),
        {"試管傾角": tilt, "槽位距離": np.linalg.norm(payload[:, 0] - target, axis=1)},
        {"試管傾角": "deg", "槽位距離": "m"}, metrics,
        ("條碼辨識以正確 oracle 身分代替。", "試管視為封閉剛體，未模擬液體晃動與破裂。"), _events(time_s, phases),
    )


def _sterile_tray(
    spec: HospitalCaseSpec, frame_rate_hz: int, rng: np.random.Generator
) -> HospitalCaseResult:
    noise_m = _parameter(spec, "observation_noise_m", 0.0015)
    target = np.asarray([0.34, 0.12, 0.11])
    time_s, phases, tcp = _timeline(
        (
            ("器械盤盤點", 0.0, (-0.58, -0.34, 0.56)),
            ("接近持針器", 1.0, (-0.32, -0.12, 0.28)),
            ("夾取持針器", 0.5, (-0.32, -0.12, 0.14)),
            ("退出來源區", 0.8, (-0.32, -0.12, 0.38)),
            ("跨越無菌邊界", 1.0, (0.04, 0.02, 0.44)),
            ("對準指定槽", 0.9, (0.34, 0.12, 0.25)),
            ("放置與姿態覆核", 0.6, (0.34, 0.12, 0.16)),
        ), frame_rate_hz,
    )
    tool = np.repeat(np.asarray([[[-0.32, -0.12, 0.09]]]), len(time_s), axis=0)
    attached = np.asarray([p not in {"器械盤盤點", "接近持針器"} for p in phases])
    tool[attached, 0] = tcp[attached] + np.asarray([0.0, 0.0, -0.05])
    tool[-1, 0] = target
    center, half = np.asarray([0.28, 0.08]), np.asarray([0.42, 0.30])
    inside = np.all(np.abs(tool[:, 0, :2] - center) <= half, axis=1)
    # 來源盤到有效區的邊界切換本身不是違規；進入有效區後才持續監測。
    evaluate = np.asarray([p in {"對準指定槽", "放置與姿態覆核"} for p in phases])
    violations = int(np.count_nonzero(evaluate & ~inside))
    assets = (
        HospitalAsset.box("器械工作台", (0, 0, 0), (1.5, 0.9, 0.08), STERILE),
        HospitalAsset.box("來源盤", (-0.34, -0.14, 0.075), (0.38, 0.28, 0.07), STEEL),
        HospitalAsset.box("無菌器械盤", (0.28, 0.08, 0.075), (0.84, 0.60, 0.07), SURGICAL_BLUE, opacity=0.45),
        HospitalAsset.box("無菌有效區", (0.28, 0.08, 0.10), (0.84, 0.60, 0.015), MONITOR_TEAL, opacity=0.16, analysis_only=True, zone=True),
        HospitalAsset.box("污染邊緣", (0.71, 0.08, 0.12), (0.06, 0.60, 0.08), PULSE_CORAL, opacity=0.24, analysis_only=True, zone=True),
        HospitalAsset.box("指定器械槽", target, (0.22, 0.06, 0.025), IODINE, opacity=0.38, analysis_only=True, zone=True),
    )
    metrics = (
        HospitalMetric("final_position_error_m", "最終器械槽誤差", float(np.linalg.norm(tool[-1, 0] - target)), "m", "maximum", _parameter(spec, "maximum_final_error_m", 0.008)),
        HospitalMetric("sterile_zone_violation_count", "無菌區違規幀", violations, "count", "maximum", _parameter(spec, "maximum_sterile_violations", 0)),
        HospitalMetric("instrument_identity_mismatch_count", "器械身分不符", 0, "count", "exact", 0),
    )
    return HospitalCaseResult(
        spec, frame_rate_hz, time_s, phases, "NumPy 幾何路徑與語意區域規則",
        "訓練盤與模型器械；無消毒效力或臨床無菌宣稱",
        "將指定器械由來源盤移入無菌盤槽位，追蹤路徑是否越過污染邊緣。",
        assets, (_track("TCP", tcp, rng, noise_m, IODINE), _track("持針器", tool, rng, noise_m, GRAPHITE, width=9)),
        {"無菌區狀態": inside.astype(float), "槽位距離": np.linalg.norm(tool[:, 0] - target, axis=1)},
        {"無菌區狀態": "bool", "槽位距離": "m"}, metrics,
        ("無菌狀態只由幾何區域表示。", "未模擬微生物、包裝完整性或碰撞污染。"), _events(time_s, phases),
    )


def _bedside_tubing(
    spec: HospitalCaseSpec, frame_rate_hz: int, rng: np.random.Generator
) -> HospitalCaseResult:
    noise_m = _parameter(spec, "observation_noise_m", 0.0025)
    initial = _resample(np.asarray([
        [-0.66, -0.20, 0.58], [-0.46, -0.24, 0.54], [-0.22, -0.18, 0.52],
        [0.02, -0.26, 0.54], [0.25, -0.16, 0.56], [0.43, -0.10, 0.57],
    ]), 41)
    grasp_index = 23
    grasp = initial[grasp_index]
    time_s, phases, tcp = _timeline((
        ("辨識指定管路", 0.0, (-0.50, -0.44, 0.84)),
        ("預抓取", 1.0, tuple(grasp + np.asarray([0.0, 0.0, 0.18]))),
        ("夾取未連接管路", 0.6, tuple(grasp + np.asarray([0.0, 0.0, 0.025]))),
        ("抬升避開床欄", 1.0, (-0.02, -0.34, 0.84)),
        ("繞過監護線束", 1.2, (0.28, -0.36, 0.90)),
        ("移至管路固定夾", 1.1, (0.56, -0.30, 0.78)),
        ("置入固定夾", 0.7, (0.56, -0.30, 0.64)),
        ("釋放與退回", 0.7, (0.42, -0.20, 0.86)),
    ), frame_rate_hz)
    hose = np.repeat(initial[None, :, :], len(time_s), axis=0)
    attached = np.asarray([p in {"夾取未連接管路", "抬升避開床欄", "繞過監護線束", "移至管路固定夾", "置入固定夾"} for p in phases])
    weights = np.exp(-np.abs(np.arange(len(initial)) - grasp_index) / 8.0)
    for index in np.flatnonzero(attached):
        hose[index] = initial + weights[:, None] * (tcp[index] - initial[grasp_index])
        hose[index, :, 2] = np.maximum(hose[index, :, 2], 0.49)
    release = np.flatnonzero(np.asarray([p == "釋放與退回" for p in phases]))
    if len(release):
        hose[release] = hose[release[0] - 1]
    observed = _observed(hose, rng, noise_m)
    rest_length = float(np.sum(np.linalg.norm(np.diff(initial, axis=0), axis=1)))
    lengths = np.sum(np.linalg.norm(np.diff(hose, axis=1), axis=2), axis=1)
    tension = np.maximum(0, lengths - rest_length) * 5.0
    bend = np.asarray([_bend_radius(nodes) for nodes in hose])
    forbidden_center, forbidden_half = np.asarray([0.12, 0.12, 0.69]), np.asarray([0.30, 0.10, 0.18])
    inside = np.all(np.abs(hose - forbidden_center) <= forbidden_half, axis=2)
    forbidden_entries = int(np.count_nonzero(np.any(inside & attached[:, None], axis=1)))
    assets = (
        HospitalAsset.box("病床", (0.10, 0.08, 0.36), (1.45, 0.72, 0.22), "#B8C8CB"),
        HospitalAsset.box("訓練假人", (0.12, 0.10, 0.58), (0.82, 0.30, 0.20), "#D9B8A5", opacity=0.70),
        HospitalAsset.box("病患禁止接觸區", forbidden_center, forbidden_half * 2, PULSE_CORAL, opacity=0.12, analysis_only=True, zone=True),
        HospitalAsset.polyline("床欄", np.asarray([[-0.48, -0.34, 0.54], [0.52, -0.34, 0.54]]), STEEL),
        HospitalAsset.polyline("IV 架", np.asarray([[-0.66, -0.20, 0.44], [-0.66, -0.20, 1.12]]), GRAPHITE),
        HospitalAsset.polyline("其他監護線束", np.asarray([[-0.20, 0.30, 0.60], [0.12, 0.38, 0.55], [0.48, 0.32, 0.58]]), VIOLET),
        HospitalAsset.box("目標固定夾", (0.56, -0.30, 0.62), (0.08, 0.08, 0.12), MONITOR_TEAL, opacity=0.38, analysis_only=True, zone=True),
    )
    metrics = (
        HospitalMetric("maximum_tension_proxy_n", "最大張力代理", float(np.max(tension)), "N", "maximum", _parameter(spec, "maximum_tension_proxy_n", 8)),
        HospitalMetric("minimum_bend_radius_m", "最小彎曲半徑", float(np.min(bend)), "m", "minimum", _parameter(spec, "minimum_bend_radius_m", 0.018)),
        HospitalMetric("forbidden_zone_entry_count", "病患禁區進入幀", forbidden_entries, "count", "maximum", _parameter(spec, "maximum_forbidden_entries", 0)),
        HospitalMetric("maximum_observation_error_m", "最大管路觀測誤差", float(np.max(np.linalg.norm(observed - hose, axis=2))), "m"),
    )
    return HospitalCaseResult(
        spec, frame_rate_hz, time_s, phases, "NumPy 準靜態中心線與未校正張力代理",
        "假人、未連接病患的訓練管路；不包含生命維持設備",
        "辨識並整理一條未連接病患的柔性管路，繞過床欄與其他線束後放入固定夾。",
        assets, (HospitalTrack("目標管路", hose, observed, SURGICAL_BLUE, "lines+markers", 8, 3), _track("TCP", tcp, rng, noise_m * 0.7, IODINE, style="markers", marker_size=8)),
        {"張力代理": tension, "最小彎曲半徑": bend, "禁區節點數": np.sum(inside, axis=1).astype(float)},
        {"張力代理": "N", "最小彎曲半徑": "m", "禁區節點數": "count"}, metrics,
        ("中心線為準靜態代理，不是已校正材料模型。", "禁止區是固定盒體，假人不會移動或呼吸。", "案例排除與病患相連的管路。"), _events(time_s, phases),
    )


def _ward_delivery(
    spec: HospitalCaseSpec, frame_rate_hz: int, rng: np.random.Generator
) -> HospitalCaseResult:
    noise_m = _parameter(spec, "observation_noise_m", 0.02)
    time_s, phases, robot = _timeline((
        ("護理站取件", 0.0, (-1.25, -0.65, 0.18)),
        ("前往走廊交會口", 1.8, (-0.72, -0.08, 0.18)),
        ("等候人員通過", 1.6, (-0.72, -0.08, 0.18)),
        ("門禁開啟", 1.0, (-0.15, -0.08, 0.18)),
        ("進入電梯", 1.3, (0.38, -0.08, 0.18)),
        ("跨樓層轉移", 1.5, (0.38, -0.08, 0.78)),
        ("抵達病區", 1.8, (1.18, 0.56, 0.78)),
        ("完成交接", 0.6, (1.18, 0.56, 0.78)),
    ), frame_rate_hz)
    person = np.empty_like(robot)
    person[:, 0] = -0.15
    person[:, 1] = -0.82
    person[:, 2] = 0.18
    wait_indexes = np.flatnonzero(np.asarray([p == "等候人員通過" for p in phases]))
    person[wait_indexes, 1] = np.linspace(-0.82, 0.78, len(wait_indexes))
    if len(wait_indexes):
        person[wait_indexes[-1] + 1 :, 1] = 0.78
    distance = np.linalg.norm(robot - person, axis=1)
    wait_s = float(np.count_nonzero(np.asarray([p == "等候人員通過" for p in phases])) / frame_rate_hz)
    route_distance = float(np.sum(np.linalg.norm(np.diff(robot, axis=0), axis=1)))
    assets = (
        HospitalAsset.box("一樓走廊", (0, 0, 0.02), (2.9, 1.0, 0.04), STERILE),
        HospitalAsset.box("二樓走廊", (0.72, 0.28, 0.62), (1.9, 1.1, 0.04), "#CFDCDF", opacity=0.55),
        HospitalAsset.box("門禁線", (-0.15, -0.08, 0.26), (0.05, 0.88, 0.52), IODINE, opacity=0.22, analysis_only=True, zone=True),
        HospitalAsset.box("電梯", (0.38, -0.08, 0.35), (0.55, 0.62, 0.70), STEEL, opacity=0.28),
        HospitalAsset.box("護理站", (-1.18, -0.60, 0.14), (0.36, 0.30, 0.22), SURGICAL_BLUE),
        HospitalAsset.box("目的病區", (1.18, 0.56, 0.74), (0.42, 0.32, 0.20), MONITOR_TEAL),
    )
    metrics = (
        HospitalMetric("minimum_human_clearance_m", "最小人員距離", float(np.min(distance)), "m", "minimum", _parameter(spec, "minimum_human_clearance_m", 0.45)),
        HospitalMetric("human_wait_duration_s", "人流等候時間", wait_s, "s", "minimum", _parameter(spec, "required_wait_s", 1)),
        HospitalMetric("route_distance_m", "配送路徑長度", route_distance, "m"),
        HospitalMetric("facility_request_count", "門／電梯請求", 2, "count"),
    )
    return HospitalCaseResult(
        spec, frame_rate_hz, time_s, phases, "NumPy 事件時間線與人員距離代理",
        "單一移動平台與一名行人；無真實門禁、電梯或多車隊調度",
        "模擬耗材從護理站通過人流交會口、門禁與電梯送達另一病區。",
        assets, (_track("配送機器人", robot, rng, noise_m, SURGICAL_BLUE, style="markers", marker_size=11), _track("行人", person, rng, noise_m * 0.5, PULSE_CORAL, style="markers", marker_size=10)),
        {"人員距離": distance, "樓層高度": robot[:, 2]}, {"人員距離": "m", "樓層高度": "m"}, metrics,
        ("人員軌跡預先已知，未做行為預測。", "門與電梯只以事件表示；完整協調應接入 Open-RMF。"), _events(time_s, phases),
    )


def _disinfection_coverage(
    spec: HospitalCaseSpec, frame_rate_hz: int, rng: np.random.Generator
) -> HospitalCaseResult:
    noise_m = _parameter(spec, "observation_noise_m", 0.008)
    rows = np.linspace(-0.58, 0.58, 7)
    keyframes: list[tuple[str, float, tuple[float, float, float]]] = [("建立表面地圖", 0.0, (-0.70, float(rows[0]), 0.46))]
    direction = 1.0
    for index, y_value in enumerate(rows):
        keyframes.append((f"覆蓋掃描 {index + 1}/7", 1.1, (0.70 * direction, float(y_value), 0.46)))
        direction *= -1.0
    keyframes.append(("覆蓋率覆核", 0.6, (0.0, 0.0, 0.62)))
    time_s, phases, nozzle = _timeline(tuple(keyframes), frame_rate_hz)
    grid_x, grid_y = np.linspace(-0.78, 0.78, 20), np.linspace(-0.66, 0.66, 16)
    xx, yy = np.meshgrid(grid_x, grid_y)
    grid = np.column_stack((xx.ravel(), yy.ravel()))
    radius = _parameter(spec, "dose_radius_m", 0.24)
    dose = np.zeros(len(grid))
    coverage = np.zeros(len(time_s))
    active = np.zeros(len(time_s))
    dose_threshold = 0.42
    for index, point in enumerate(nozzle[:, :2]):
        contribution = np.exp(-np.sum((grid - point) ** 2, axis=1) / (2 * radius**2))
        dose += contribution / frame_rate_hz
        coverage[index] = np.mean(dose >= dose_threshold)
        active[index] = float(np.mean(contribution))
    final_coverage = float(np.mean(dose >= dose_threshold))
    assets: list[HospitalAsset] = [
        HospitalAsset.box("病房地面", (0, 0, 0), (1.8, 1.5, 0.05), STERILE),
        HospitalAsset.box("病床遮擋", (0.38, 0.18, 0.28), (0.62, 0.36, 0.50), STEEL, opacity=0.62),
    ]
    cell_size = (grid_x[1] - grid_x[0], grid_y[1] - grid_y[0])
    for index, (x_value, y_value) in enumerate(grid):
        ratio = dose[index] / dose_threshold
        color = MONITOR_TEAL if ratio >= 1 else IODINE if ratio >= 0.65 else PULSE_CORAL
        assets.append(HospitalAsset.box(
            f"覆蓋單元 {index + 1}", (float(x_value), float(y_value), 0.035),
            (cell_size[0] * 0.92, cell_size[1] * 0.92, 0.008), color,
            opacity=0.34, analysis_only=True, zone=True,
        ))
    metrics = (
        HospitalMetric("surface_coverage_ratio", "表面覆蓋率", final_coverage, "ratio", "minimum", _parameter(spec, "minimum_coverage_ratio", 0.9)),
        HospitalMetric("uncovered_cell_count", "未達代理劑量單元", int(np.count_nonzero(dose < dose_threshold)), "count"),
        HospitalMetric("maximum_relative_dose", "最大相對劑量", float(np.max(dose / dose_threshold)), "x"),
    )
    return HospitalCaseResult(
        spec, frame_rate_hz, time_s, phases, "NumPy 高斯距離劑量代理",
        "只評估幾何覆蓋；不代表 UV、藥劑濃度或微生物殺滅效果",
        "以蛇形路徑掃描病房表面，計算每個網格單元的相對覆蓋劑量。",
        tuple(assets), (_track("消毒噴頭", nozzle, rng, noise_m, SURGICAL_BLUE, style="markers", marker_size=9),),
        {"累積覆蓋率": coverage, "即時相對劑量": active}, {"累積覆蓋率": "ratio", "即時相對劑量": "x"}, metrics,
        ("劑量只與平面距離有關，沒有入射角、陰影、風場或材質。", "覆蓋門檻是教學參數，不是消毒效力標準。"), _events(time_s, phases),
    )


def _ultrasound_phantom(
    spec: HospitalCaseSpec, frame_rate_hz: int, rng: np.random.Generator
) -> HospitalCaseResult:
    noise_m = _parameter(spec, "observation_noise_m", 0.001)
    target_force = _parameter(spec, "target_force_n", 3.5)
    time_s, phases, probe = _timeline((
        ("探頭定位", 0.0, (-0.48, -0.22, 0.34)),
        ("建立接觸", 0.8, (-0.42, -0.18, 0.245)),
        ("掃描第一列", 1.8, (0.42, -0.18, 0.245)),
        ("換列", 0.5, (0.42, 0.18, 0.245)),
        ("掃描第二列", 1.8, (-0.42, 0.18, 0.245)),
        ("抬離假體", 0.7, (-0.42, 0.18, 0.34)),
    ), frame_rate_hz)
    contact = probe[:, 2] < 0.28
    penetration = np.maximum(0, 0.25 - probe[:, 2])
    velocity = np.concatenate(([0.0], np.diff(penetration) * frame_rate_hz))
    force = np.maximum(0, target_force / 0.005 * penetration + 1.2 * velocity)
    deformation = penetration * (1 + 0.08 * np.sin(time_s * 3))
    scan_mask = np.asarray([p in {"掃描第一列", "換列", "掃描第二列"} for p in phases])
    contact_ratio = float(np.mean(contact[scan_mask]))
    assets = (
        HospitalAsset.box("超音波假體", (0, 0, 0.15), (1.10, 0.66, 0.20), "#9DC8C1", opacity=0.72),
        HospitalAsset.box("掃描 ROI", (0, 0, 0.255), (0.92, 0.48, 0.012), SURGICAL_BLUE, opacity=0.18, analysis_only=True, zone=True),
    )
    metrics = (
        HospitalMetric("maximum_contact_force_n", "最大接觸力代理", float(np.max(force)), "N", "maximum", _parameter(spec, "maximum_force_n", 7)),
        HospitalMetric("mean_contact_force_n", "平均接觸力代理", float(np.mean(force[contact])), "N"),
        HospitalMetric("contact_ratio", "掃描階段接觸率", contact_ratio, "ratio", "minimum", _parameter(spec, "minimum_contact_ratio", 0.95)),
        HospitalMetric("maximum_deformation_m", "最大表面形變代理", float(np.max(deformation)), "m"),
    )
    return HospitalCaseResult(
        spec, frame_rate_hz, time_s, phases, "Kelvin–Voigt 單點解析代理 / NumPy",
        "商用訓練假體；不產生診斷影像、不接觸病患",
        "控制探頭在假體表面維持接觸並完成雙列掃描，觀察力與形變代理。",
        assets, (_track("超音波探頭", probe, rng, noise_m, SURGICAL_BLUE, marker_size=9),),
        {"接觸力代理": force, "表面形變代理": deformation}, {"接觸力代理": "N", "表面形變代理": "m"}, metrics,
        ("假體以單點 Kelvin–Voigt 關係表示，沒有 FEM 或探頭面積。", "沒有聲學傳播、影像形成與組織分類。"), _events(time_s, phases),
    )


def _catheter_navigation(
    spec: HospitalCaseSpec, frame_rate_hz: int, rng: np.random.Generator
) -> HospitalCaseResult:
    noise_m = _parameter(spec, "observation_noise_m", 0.0008)
    time_s, phases, tip = _timeline((
        ("建立血管中心線", 0.0, (-0.62, 0.0, 0.42)),
        ("導管進入入口", 0.8, (-0.52, 0.0, 0.42)),
        ("通過第一彎道", 1.5, (-0.14, 0.18, 0.48)),
        ("通過第二彎道", 1.5, (0.22, -0.15, 0.55)),
        ("抵達研究目標", 1.4, (0.58, 0.05, 0.62)),
        ("保持與覆核", 0.6, (0.58, 0.05, 0.62)),
    ), frame_rate_hz)
    parameter = np.linspace(0, 2.5 * np.pi, 220)
    vessel = np.column_stack((np.linspace(-0.62, 0.58, 220), 0.17 * np.sin(parameter), np.linspace(0.42, 0.62, 220)))
    progress = np.clip((tip[:, 0] - vessel[0, 0]) / (vessel[-1, 0] - vessel[0, 0]), 0, 1)
    catheter = np.empty((len(time_s), 35, 3))
    for index, value in enumerate(progress):
        count = max(2, int(round(value * (len(vessel) - 1))) + 1)
        catheter[index] = _resample(vessel[:count], 35)
    observed = _observed(catheter, rng, noise_m)
    speed = np.concatenate(([0.0], np.linalg.norm(np.diff(tip, axis=0), axis=1) * frame_rate_hz))
    force = 0.18 + progress * 0.72 + speed * 0.38
    depth = progress * float(np.sum(np.linalg.norm(np.diff(vessel, axis=0), axis=1)))
    violations = np.zeros(len(time_s))
    assets = (
        HospitalAsset.box("透明訓練假體", (0, 0, 0.50), (1.45, 0.62, 0.42), "#D6E7E8", opacity=0.20),
        HospitalAsset.polyline("血管中心線", vessel, PULSE_CORAL, opacity=0.58),
        HospitalAsset.polyline("規劃走廊", vessel, MONITOR_TEAL, opacity=0.32, analysis_only=True),
        HospitalAsset.box("研究目標", (0.58, 0.05, 0.62), (0.08, 0.08, 0.08), IODINE, opacity=0.30, analysis_only=True, zone=True),
    )
    metrics = (
        HospitalMetric("maximum_force_proxy_n", "最大插入力代理", float(np.max(force)), "N", "maximum", _parameter(spec, "maximum_force_proxy_n", 2)),
        HospitalMetric("wall_violation_count", "血管壁違規幀", float(np.sum(violations)), "count", "maximum", _parameter(spec, "maximum_wall_violations", 0)),
        HospitalMetric("final_insertion_depth_m", "最終插入深度", float(depth[-1]), "m"),
        HospitalMetric("maximum_observation_error_m", "最大中心線觀測誤差", float(np.max(np.linalg.norm(observed - catheter, axis=2))), "m"),
    )
    return HospitalCaseResult(
        spec, frame_rate_hz, time_s, phases, "預定中心線＋摩擦／速度代理 / NumPy",
        "透明訓練假體與預定中心線；禁止解讀為導管控制或臨床驗證",
        "沿預定血管中心線逐步插入導管，顯示插入深度、觀測誤差與力代理。",
        assets, (HospitalTrack("導管", catheter, observed, SURGICAL_BLUE, "lines+markers", 8, 2.5),),
        {"插入力代理": force, "插入深度": depth, "壁面違規": violations}, {"插入力代理": "N", "插入深度": "m", "壁面違規": "count"}, metrics,
        ("導管被限制在已知中心線，沒有 Cosserat rod、導絲、血管變形、血流或透視影像。", "力值是速度與路徑進度代理，沒有實驗校正。"), _events(time_s, phases),
    )


_SIMULATORS: dict[str, Callable[[HospitalCaseSpec, int, np.random.Generator], HospitalCaseResult]] = {
    "specimen_transfer": _specimen_transfer,
    "sterile_tray": _sterile_tray,
    "bedside_tubing": _bedside_tubing,
    "ward_delivery": _ward_delivery,
    "disinfection_coverage": _disinfection_coverage,
    "ultrasound_phantom": _ultrasound_phantom,
    "catheter_navigation": _catheter_navigation,
}


def simulate_hospital_suite(spec: HospitalSuiteSpec) -> HospitalSuiteResult:
    """依設定順序執行全部啟用案例；未知案例明確失敗。"""

    results: list[HospitalCaseResult] = []
    for case_spec in spec.cases:
        if not case_spec.enabled:
            continue
        simulator = _SIMULATORS.get(case_spec.case_id)
        if simulator is None:
            raise ValueError(f"尚未實作的 hospital case：{case_spec.case_id}")
        rng = np.random.default_rng(spec.seed + case_spec.order * 1009)
        results.append(simulator(case_spec, spec.frame_rate_hz, rng))
    return HospitalSuiteResult(spec, tuple(results))
