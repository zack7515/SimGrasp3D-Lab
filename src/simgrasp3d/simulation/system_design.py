"""評估手臂、相機、軟管與障礙物的系統設計參數。"""

from __future__ import annotations

import itertools
from dataclasses import replace
from pathlib import Path

import numpy as np

from simgrasp3d.geometry.collision import capsule_clearance, segment_distance
from simgrasp3d.io import load_spec
from simgrasp3d.models.motion import HoseMotionSpec, MotionKeyframeSpec
from simgrasp3d.models.specs import SceneSpec
from simgrasp3d.models.system_design import (
    DesignGate,
    SystemDesignLabResult,
    SystemDesignSnapshot,
    SystemDesignSpec,
)
from simgrasp3d.simulation.waypoint_planner import plan_safe_waypoints


def load_system_design_spec(path: str | Path) -> SystemDesignSpec:
    """讀取並驗證系統設計工作台 JSON。"""

    return load_spec(path, SystemDesignSpec)


def _resample_polyline(points: tuple[tuple[float, float, float], ...], count: int) -> np.ndarray:
    """沿弧長等距取樣，讓夾取比例不依控制點密度改變。"""

    values = np.asarray(points, dtype=np.float64)
    lengths = np.linalg.norm(np.diff(values, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    samples = np.linspace(0.0, cumulative[-1], count)
    return np.column_stack(
        [np.interp(samples, cumulative, values[:, axis]) for axis in range(3)]
    )


def _minimum_bend_radius(points: np.ndarray) -> float:
    """以三點外接圓估計中心線最小局部曲率半徑。"""

    radii: list[float] = []
    for first, middle, last in zip(points[:-2], points[1:-1], points[2:], strict=True):
        a = float(np.linalg.norm(middle - first))
        b = float(np.linalg.norm(last - middle))
        c = float(np.linalg.norm(last - first))
        area_twice = float(np.linalg.norm(np.cross(middle - first, last - first)))
        if min(a, b, c) <= 1e-9 or area_twice <= 1e-8:
            continue
        radii.append(a * b * c / (2.0 * area_twice))
    return min(radii, default=float("inf"))


def _camera_basis(position: np.ndarray, look_at: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = look_at - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray([0.0, 0.0, 1.0]))
    if np.linalg.norm(right) <= 1e-9:
        right = np.asarray([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)
    return right, up, forward


def _camera_coverage(
    points: np.ndarray,
    position: np.ndarray,
    look_at: np.ndarray,
    vertical_fov_deg: float,
    aspect_ratio: float,
    near: float,
    far: float,
    obstacle_segments: np.ndarray,
    obstacle_radii: np.ndarray,
) -> tuple[float, float]:
    """回傳視錐內比例與加入管路遮蔽後的可觀測比例。"""

    right, up, forward = _camera_basis(position, look_at)
    relative = points - position
    depth = relative @ forward
    half_height = np.tan(np.deg2rad(vertical_fov_deg) / 2.0)
    horizontal = relative @ right
    vertical = relative @ up
    visible = (
        (depth >= near)
        & (depth <= far)
        & (np.abs(vertical) <= depth * half_height)
        & (np.abs(horizontal) <= depth * half_height * aspect_ratio)
    )
    unoccluded = visible.copy()
    for index in np.flatnonzero(visible):
        for obstacle, radius in zip(obstacle_segments, obstacle_radii, strict=True):
            if segment_distance(position, points[index], obstacle[0], obstacle[1]) <= radius:
                unoccluded[index] = False
                break
    return float(np.mean(visible)), float(np.mean(unoccluded))


def _depth_uncertainty(
    distance_m: float,
    scene: SceneSpec,
    scale: float,
) -> float:
    """合成軸向與外參誤差的保守 3σ 教學估算。"""

    noise = scene.camera.noise
    axial_sigma = noise.axial_noise_std_base_m + noise.axial_noise_std_per_m2 * distance_m**2
    rotation_sigma_m = distance_m * np.deg2rad(noise.extrinsic_rotation_std_deg)
    combined = np.sqrt(
        axial_sigma**2
        + noise.extrinsic_translation_std_m**2
        + rotation_sigma_m**2
        + (noise.depth_quantization_m / np.sqrt(12.0)) ** 2
    )
    return float(3.0 * combined * scale)


def _planned_path(
    motion: HoseMotionSpec,
    values: dict[str, float],
    grasp_point: np.ndarray,
    goal_point: np.ndarray,
) -> tuple[np.ndarray, int, int, tuple]:
    """建立 pick–lift–transfer–place 路徑並沿用既有 waypoint planner。"""

    lift = values["lift_height_m"]
    command = values["gripper_command_m"]
    approach_height = max(0.10, min(0.18, lift * 0.55))
    pregrasp = grasp_point + np.asarray([0.0, 0.0, approach_height])
    lifted = grasp_point + np.asarray([0.0, 0.0, lift])
    transfer = goal_point + np.asarray([0.0, 0.0, lift])
    start = np.asarray(motion.keyframes[0].tcp_position, dtype=np.float64)
    definitions = (
        ("待機", start, False, 0.0),
        ("預抓取", pregrasp, False, 1.0),
        ("下降", grasp_point, False, 0.8),
        ("閉爪", grasp_point, True, 0.3),
        ("抬升", lifted, True, 1.0),
        ("搬運", transfer, True, 1.6),
        ("放置", goal_point, True, 0.8),
        ("釋放", goal_point, False, 0.3),
        ("退回", transfer, False, 0.8),
    )
    keyframes = tuple(
        MotionKeyframeSpec(
            phase=phase,
            duration_s=duration,
            tcp_position=tuple(float(item) for item in point),
            tcp_rpy_deg=(0.0, 60.0, 0.0),
            gripper_opening_m=max(command, 0.001),
            attached=attached,
        )
        for phase, point, attached, duration in definitions
    )
    obstacle_scale = values["obstacle_radius_scale"]
    obstacles = tuple(replace(item, radius=item.radius * obstacle_scale) for item in motion.obstacles)
    planner = replace(
        motion.waypoint_planner,
        tool_envelope_radius_m=max(
            motion.waypoint_planner.tool_envelope_radius_m,
            command / 2.0 + 0.025,
        ),
    )
    tuned = replace(
        motion,
        safe_clearance_m=values["safety_margin_m"],
        hose=replace(motion.hose, radius=values["hose_radius_m"]),
        obstacles=obstacles,
        keyframes=keyframes,
        waypoint_planner=planner,
    )
    plan = plan_safe_waypoints(tuned)
    path = np.asarray([item.tcp_position for item in plan.keyframes], dtype=np.float64)
    return path, plan.inserted_waypoint_count, plan.unresolved_segment_count, obstacles


def _path_clearance(path: np.ndarray, motion: HoseMotionSpec, obstacles: tuple) -> float:
    radius = motion.waypoint_planner.tool_envelope_radius_m
    values: list[float] = []
    for first, second in itertools.pairwise(path):
        for obstacle in obstacles:
            values.append(
                capsule_clearance(
                    first,
                    second,
                    radius,
                    np.asarray(obstacle.start, dtype=np.float64),
                    np.asarray(obstacle.end, dtype=np.float64),
                    obstacle.radius,
                )
            )
    return min(values, default=float("inf"))


def evaluate_system_design(
    spec: SystemDesignSpec,
    scene: SceneSpec,
    motion: HoseMotionSpec,
    values: dict[str, float] | None = None,
) -> SystemDesignSnapshot:
    """以可解釋幾何近似評估一組系統參數。"""

    tuned_values = spec.default_values | (values or {})
    ranges = {item.key: item for item in spec.parameters}
    unknown = set(tuned_values) - set(ranges)
    if unknown:
        raise ValueError(f"未知設計參數：{', '.join(sorted(unknown))}")
    for key, value in tuned_values.items():
        parameter = ranges[key]
        if not parameter.minimum <= value <= parameter.maximum:
            raise ValueError(f"{key}={value} 超出 [{parameter.minimum}, {parameter.maximum}]")

    hose_points = _resample_polyline(motion.hose.control_points, motion.hose.node_count)
    grasp_index = int(round(tuned_values["grasp_fraction"] * (len(hose_points) - 1)))
    grasp_point = hose_points[grasp_index].copy()
    grasp_point[2] = max(grasp_point[2], motion.table_top_z + tuned_values["hose_radius_m"])
    goal_point = np.asarray(motion.target_position, dtype=np.float64)
    goal_point[2] = max(goal_point[2], motion.table_top_z + tuned_values["hose_radius_m"])

    path, inserted, unresolved, obstacles = _planned_path(
        motion, tuned_values, grasp_point, goal_point
    )
    obstacle_segments = np.asarray(
        [[item.start, item.end] for item in obstacles], dtype=np.float64
    )
    obstacle_radii = np.asarray([item.radius for item in obstacles], dtype=np.float64)
    camera_position = np.asarray(
        [scene.camera.position[0], tuned_values["camera_lateral_m"], tuned_values["camera_height_m"]],
        dtype=np.float64,
    )
    camera_look_at = np.asarray(scene.camera.look_at, dtype=np.float64)
    observed_points = np.vstack((hose_points, goal_point))
    frustum_ratio, visible_ratio = _camera_coverage(
        observed_points,
        camera_position,
        camera_look_at,
        tuned_values["camera_fov_deg"],
        scene.camera.aspect_ratio,
        scene.camera.near,
        scene.camera.far,
        obstacle_segments,
        obstacle_radii,
    )
    grasp_distance = float(np.linalg.norm(grasp_point - camera_position))
    depth_uncertainty = _depth_uncertainty(
        grasp_distance, scene, tuned_values["depth_noise_scale"]
    )

    shoulder = np.asarray(scene.robot.base_pose.xyz, dtype=np.float64)
    shoulder[2] += scene.robot.base_size[2]
    nominal_reach = sum(float(np.linalg.norm(link.translation)) for link in scene.robot.links)
    nominal_reach += float(np.linalg.norm(scene.robot.gripper.tcp_offset))
    maximum_reach = nominal_reach * tuned_values["arm_reach_scale"]
    maximum_request = float(np.max(np.linalg.norm(path - shoulder, axis=1)))
    reach_reserve = maximum_reach - maximum_request
    diameter_error = abs(tuned_values["gripper_command_m"] - 2.0 * tuned_values["hose_radius_m"])
    minimum_bend_radius = _minimum_bend_radius(np.asarray(motion.hose.control_points))
    clearance = _path_clearance(path, replace(motion, waypoint_planner=replace(
        motion.waypoint_planner,
        tool_envelope_radius_m=max(
            motion.waypoint_planner.tool_envelope_radius_m,
            tuned_values["gripper_command_m"] / 2.0 + 0.025,
        ),
    )), obstacles)

    thresholds = spec.thresholds
    gates = (
        DesignGate(
            "camera_coverage", "PERCEPTION", "任務區可觀測率", visible_ratio, "ratio", ">=",
            thresholds["minimum_visibility_ratio"], visible_ratio >= thresholds["minimum_visibility_ratio"],
            "先確認軟管與放置點都落在視錐內，並排除固定管路遮蔽。",
            "調整相機高度、側向位置或 FOV；多視角需求應改成第二台相機。",
        ),
        DesignGate(
            "depth_uncertainty", "CALIBRATION", "抓取點 3σ 深度不確定度", depth_uncertainty, "m", "<=",
            thresholds["maximum_depth_uncertainty_m"], depth_uncertainty <= thresholds["maximum_depth_uncertainty_m"],
            "距離雜訊、深度量化及外參誤差會一起侵蝕抓取容差。",
            "縮短工作距離、改善外參標定，或在抓取前加入近距離重觀測。",
        ),
        DesignGate(
            "reach_reserve", "ROBOT", "工作空間保留量", reach_reserve, "m", ">=",
            thresholds["minimum_reach_reserve_m"], reach_reserve >= thresholds["minimum_reach_reserve_m"],
            "球形包絡只做早期尺寸篩選；通過後仍要跑關節限制與姿態 IK。",
            "移動底座、縮短抬升高度、改變夾取點，或選用較長手臂。",
        ),
        DesignGate(
            "gripper_match", "GRASP", "夾爪開口與管徑差", diameter_error, "m", "<=",
            thresholds["maximum_gripper_diameter_error_m"], diameter_error <= thresholds["maximum_gripper_diameter_error_m"],
            "這裡只檢查幾何相容性，不估計軟管壓縮、摩擦或允許夾持力。",
            "讓閉爪指令接近外徑，再以材料與接觸模型決定實際力控範圍。",
        ),
        DesignGate(
            "bend_radius", "HOSE", "中心線最小彎曲半徑", minimum_bend_radius, "m", ">=",
            tuned_values["hose_min_bend_radius_m"], minimum_bend_radius >= tuned_values["hose_min_bend_radius_m"],
            "控制點的局部曲率先檢查是否違反軟管產品或材料限制。",
            "改變抽取方向、夾取位置或障礙配置；高風險時改用接觸物理驗證。",
        ),
        DesignGate(
            "path_clearance", "PLANNING", "規劃路徑最小淨空", clearance, "m", ">=",
            tuned_values["safety_margin_m"], clearance >= tuned_values["safety_margin_m"] and unresolved == 0,
            "工具包絡沿每段路徑與固定管路做 capsule 距離檢查。",
            "提高抬升、允許更多 waypoint、縮小工具包絡，或重新配置障礙。",
        ),
    )
    metrics: dict[str, float | int] = {
        "passed_gate_count": sum(gate.passed for gate in gates),
        "gate_count": len(gates),
        "frustum_coverage_ratio": frustum_ratio,
        "visible_ratio": visible_ratio,
        "grasp_distance_m": grasp_distance,
        "depth_uncertainty_m": depth_uncertainty,
        "nominal_reach_m": maximum_reach,
        "maximum_reach_request_m": maximum_request,
        "reach_reserve_m": reach_reserve,
        "path_length_m": float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))),
        "minimum_path_clearance_m": clearance,
        "inserted_waypoint_count": inserted,
        "unresolved_segment_count": unresolved,
        "minimum_bend_radius_m": minimum_bend_radius,
    }
    return SystemDesignSnapshot(
        values=tuned_values,
        gates=gates,
        hose_points=hose_points,
        grasp_point=grasp_point,
        goal_point=goal_point,
        planned_path=path,
        camera_position=camera_position,
        camera_look_at=camera_look_at,
        obstacle_segments=obstacle_segments,
        obstacle_radii=obstacle_radii,
        metrics=metrics,
    )


def simulate_system_design_lab(
    spec: SystemDesignSpec,
    scene: SceneSpec,
    motion: HoseMotionSpec,
) -> SystemDesignLabResult:
    """計算基準與所有教學 preset，供測試、頁面與回歸比較。"""

    baseline = evaluate_system_design(spec, scene, motion)
    snapshots = tuple(
        (preset, evaluate_system_design(spec, scene, motion, preset.values))
        for preset in spec.presets
    )
    return SystemDesignLabResult(spec, baseline, snapshots)

