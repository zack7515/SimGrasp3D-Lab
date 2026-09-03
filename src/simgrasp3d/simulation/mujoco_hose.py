"""以 MuJoCo cable plugin 重播 TCP 軌跡並模擬軟管接觸物理。"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from simgrasp3d.geometry.collision import segment_distance
from simgrasp3d.io import load_spec
from simgrasp3d.models.motion import PipeObstacleSpec, TrajectoryData, TrajectoryFrame
from simgrasp3d.models.physics import (
    MujocoHoseSpec,
    PhysicsSweepCase,
    PhysicsSweepData,
)


@dataclass(frozen=True)
class _ContactSnapshot:
    """將軟管對環境與軟管自接觸分開記錄。"""

    environment_count: int
    environment_maximum_force_n: float
    environment_minimum_distance_m: float
    self_count: int
    self_maximum_force_n: float
    self_minimum_distance_m: float


def load_mujoco_hose_spec(path: str | Path) -> MujocoHoseSpec:
    """讀取並驗證 MuJoCo 軟管 baseline JSON。"""

    return load_spec(path, MujocoHoseSpec)


def _require_mujoco() -> Any:
    """延遲載入物理引擎，並提供可操作的安裝提示。"""

    try:
        import mujoco  # type: ignore[import-not-found]
    except ImportError as error:
        raise RuntimeError(
            "尚未安裝 MuJoCo；請執行 python -m pip install -r requirements.txt"
        ) from error
    return mujoco


def _numbers(values: np.ndarray | tuple[float, ...]) -> str:
    """將數值陣列轉成可重現且足夠精確的 MJCF 屬性字串。"""

    return " ".join(f"{float(value):.10g}" for value in np.asarray(values).ravel())


def _build_mjcf(
    initial_nodes: np.ndarray,
    grasp_node_index: int,
    hose_radius_m: float,
    obstacles: tuple[PipeObstacleSpec, ...],
    table_top_z: float,
    spec: MujocoHoseSpec,
) -> str:
    """由專案公尺制幾何建立不依賴外部資產的 MJCF。"""

    obstacle_geoms = "\n".join(
        (
            f'<geom name="pipe_{index}" type="capsule" '
            f'fromto="{_numbers((*obstacle.start, *obstacle.end))}" '
            f'size="{obstacle.radius:.10g}" '
            f'friction="{spec.friction:.10g} {spec.torsional_friction:.10g} {spec.rolling_friction:.10g}" '
            'solref="0.003 1" solimp="0.95 0.99 0.001" '
            'contype="2" conaffinity="1" '
            'rgba="0.32 0.40 0.47 1"/>'
        )
        for index, obstacle in enumerate(obstacles)
    )
    grasp_position = initial_nodes[grasp_node_index]
    body_name = f"HB_{grasp_node_index}"
    return f"""<mujoco model="simgrasp3d_hose">
  <extension><plugin plugin="mujoco.elasticity.cable"/></extension>
  <compiler autolimits="true"/>
  <size memory="32M"/>
  <option timestep="{spec.timestep_s:.10g}" integrator="implicitfast"
          iterations="{spec.solver_iterations}" gravity="0 0 -9.81">
    <flag energy="enable" autoreset="disable"/>
  </option>
  <worldbody>
    <geom name="table" type="plane" pos="0 0 {table_top_z:.10g}"
          size="1.5 1.5 0.02"
          friction="{spec.friction:.10g} {spec.torsional_friction:.10g} {spec.rolling_friction:.10g}"
          solref="0.003 1" solimp="0.95 0.99 0.001"
          contype="2" conaffinity="1"/>
    {obstacle_geoms}
    <body name="grasp_target" mocap="true" pos="{_numbers(grasp_position)}">
      <geom name="grasp_marker" type="sphere" size="0.004"
            contype="0" conaffinity="0" rgba="0.95 0.55 0.12 0.25"/>
    </body>
    <composite prefix="H" type="cable" vertex="{_numbers(initial_nodes)}"
               initial="free">
      <plugin plugin="mujoco.elasticity.cable">
        <config key="twist" value="{spec.twist_pa:.10g}"/>
        <config key="bend" value="{spec.bend_pa:.10g}"/>
        <config key="vmax" value="{spec.maximum_velocity_m_s:.10g}"/>
      </plugin>
      <joint kind="main" damping="{spec.joint_damping:.10g}"
             armature="{spec.armature:.10g}"/>
      <geom type="capsule" size="{hose_radius_m:.10g}"
            density="{spec.density_kg_m3:.10g}" condim="3"
            friction="{spec.friction:.10g} {spec.torsional_friction:.10g} {spec.rolling_friction:.10g}"
            solref="0.003 1" solimp="0.95 0.99 0.001"
            contype="1" conaffinity="2"
            rgba="0.04 0.63 0.68 1"/>
    </composite>
  </worldbody>
  <equality>
    <connect name="grasp_constraint" body1="grasp_target" body2="{body_name}"
             anchor="0 0 0" solref="{spec.attachment_time_constant_s:.10g} 1"/>
  </equality>
</mujoco>"""


def _cable_geom_ids(mujoco: Any, model: Any, segment_count: int) -> np.ndarray:
    ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"HG{index}")
        for index in range(segment_count)
    ]
    if any(identifier < 0 for identifier in ids):
        raise RuntimeError("MuJoCo cable geom 命名與預期不一致")
    return np.asarray(ids, dtype=np.int32)


def _cable_body_ids(mujoco: Any, model: Any, segment_count: int) -> np.ndarray:
    names = ["HB_first"]
    names.extend(f"HB_{index}" for index in range(1, segment_count - 1))
    names.append("HB_last")
    ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in names
    ]
    if any(identifier < 0 for identifier in ids):
        raise RuntimeError("MuJoCo cable body 命名與預期不一致")
    return np.asarray(ids, dtype=np.int32)


def _cable_nodes(
    data: Any,
    geom_ids: np.ndarray,
    body_ids: np.ndarray,
) -> np.ndarray:
    """由 cable capsule 的姿態重建與原輸入同數量的中心線節點。"""

    nodes = data.xpos[body_ids].copy()
    final_geom_id = int(geom_ids[-1])
    # cable body 位於每段起點；最後一段終點可由 capsule 中心鏡射取得。
    final_endpoint = 2.0 * data.geom_xpos[final_geom_id] - nodes[-1]
    return np.vstack((nodes, final_endpoint))


def _hose_clearance(
    nodes: np.ndarray,
    obstacles: tuple[PipeObstacleSpec, ...],
    hose_radius_m: float,
) -> float:
    clearances: list[float] = []
    for obstacle in obstacles:
        start = np.asarray(obstacle.start, dtype=np.float64)
        end = np.asarray(obstacle.end, dtype=np.float64)
        for index in range(len(nodes) - 1):
            clearances.append(
                segment_distance(nodes[index], nodes[index + 1], start, end)
                - hose_radius_m
                - obstacle.radius
            )
    return min(clearances, default=float("inf"))


def _contact_snapshot(
    mujoco: Any,
    model: Any,
    data: Any,
    cable_geom_ids: frozenset[int],
    environment_geom_ids: frozenset[int],
) -> _ContactSnapshot:
    """取得軟管對環境與自接觸的數量、界面力及最深距離。"""

    environment_count = 0
    environment_maximum_force = 0.0
    environment_minimum_distance = 0.0
    self_count = 0
    self_maximum_force = 0.0
    self_minimum_distance = 0.0
    force = np.zeros(6, dtype=np.float64)
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        first_geom = int(contact.geom[0])
        second_geom = int(contact.geom[1])
        is_environment = (
            first_geom in cable_geom_ids and second_geom in environment_geom_ids
        ) or (
            second_geom in cable_geom_ids and first_geom in environment_geom_ids
        )
        is_self = first_geom in cable_geom_ids and second_geom in cable_geom_ids
        if not is_environment and not is_self:
            continue
        mujoco.mj_contactForce(model, data, contact_index, force)
        force_norm = float(np.linalg.norm(force[:3]))
        distance = float(contact.dist)
        if is_environment:
            environment_count += 1
            environment_maximum_force = max(environment_maximum_force, force_norm)
            environment_minimum_distance = min(environment_minimum_distance, distance)
        else:
            self_count += 1
            self_maximum_force = max(self_maximum_force, force_norm)
            self_minimum_distance = min(self_minimum_distance, distance)
    return _ContactSnapshot(
        environment_count=environment_count,
        environment_maximum_force_n=environment_maximum_force,
        environment_minimum_distance_m=environment_minimum_distance,
        self_count=self_count,
        self_maximum_force_n=self_maximum_force,
        self_minimum_distance_m=self_minimum_distance,
    )


def _physics_frame(
    source: TrajectoryFrame,
    nodes: np.ndarray,
    contact_count: int,
    maximum_force_n: float,
    minimum_contact_distance_m: float,
    self_contact_count: int,
    maximum_self_contact_force_n: float,
    minimum_self_contact_distance_m: float,
    energy: np.ndarray,
    grasp_error_m: float,
    initial_length_m: float,
    trajectory: TrajectoryData,
) -> TrajectoryFrame:
    hose_clearance = _hose_clearance(
        nodes,
        trajectory.spec.obstacles,
        trajectory.spec.hose.radius,
    )
    current_length = float(np.linalg.norm(np.diff(nodes, axis=0), axis=1).sum())
    return replace(
        source,
        hose_nodes=nodes.copy(),
        hose_clearance_m=hose_clearance,
        hose_length_ratio=current_length / initial_length_m,
        physics_contact_count=contact_count,
        maximum_contact_force_n=maximum_force_n,
        minimum_contact_distance_m=minimum_contact_distance_m,
        physics_self_contact_count=self_contact_count,
        maximum_self_contact_force_n=maximum_self_contact_force_n,
        minimum_self_contact_distance_m=minimum_self_contact_distance_m,
        potential_energy_j=float(energy[0]),
        kinetic_energy_j=float(energy[1]),
        grasp_constraint_error_m=grasp_error_m,
    )


def simulate_mujoco_hose(
    trajectory: TrajectoryData,
    spec: MujocoHoseSpec,
) -> TrajectoryData:
    """以 MuJoCo 重播規劃後 TCP，回傳相同逐幀資料契約的物理軌跡。"""

    mujoco = _require_mujoco()
    initial_nodes = trajectory.frames[0].hose_nodes.copy()
    initial_length_m = float(
        np.linalg.norm(np.diff(initial_nodes, axis=0), axis=1).sum()
    )
    xml = _build_mjcf(
        initial_nodes,
        trajectory.spec.hose.grasp_node_index,
        trajectory.spec.hose.radius,
        trajectory.spec.obstacles,
        trajectory.spec.table_top_z,
        spec,
    )
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geom_ids = _cable_geom_ids(mujoco, model, len(initial_nodes) - 1)
    body_ids = _cable_body_ids(mujoco, model, len(initial_nodes) - 1)
    cable_geom_ids = frozenset(int(identifier) for identifier in geom_ids)
    environment_geom_ids = frozenset(
        {
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table"),
            *(
                mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    f"pipe_{index}",
                )
                for index in range(len(trajectory.spec.obstacles))
            ),
        }
    )
    grasp_body_name = f"HB_{trajectory.spec.hose.grasp_node_index}"
    grasp_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        grasp_body_name,
    )
    equality_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_EQUALITY,
        "grasp_constraint",
    )
    data.eq_active[equality_id] = 0
    settling_steps = int(round(spec.settling_s / spec.timestep_s))
    if settling_steps:
        mujoco.mj_step(model, data, nstep=settling_steps)
    total_steps = settling_steps

    result_frames: list[TrajectoryFrame] = []
    previous_source = trajectory.frames[0]
    previous_target = previous_source.tcp_position.copy()
    for frame_index, source in enumerate(trajectory.frames):
        maximum_contact_count = 0
        maximum_contact_force_n = 0.0
        minimum_contact_distance_m = 0.0
        maximum_self_contact_count = 0
        maximum_self_contact_force_n = 0.0
        minimum_self_contact_distance_m = 0.0
        if frame_index:
            interval_s = source.time_s - previous_source.time_s
            steps = max(1, int(round(interval_s / spec.timestep_s)))
            for step in range(1, steps + 1):
                fraction = step / steps
                data.mocap_pos[0] = previous_target + fraction * (
                    source.tcp_position - previous_target
                )
                if source.attached and not previous_source.attached:
                    # 閉爪階段開始後即建立柔順約束，避免最後一個子步瞬間吸附。
                    attached = True
                elif previous_source.attached and not source.attached:
                    attached = step < steps
                else:
                    attached = source.attached
                data.eq_active[equality_id] = int(attached)
                mujoco.mj_step(model, data)
            total_steps += steps
        else:
            data.mocap_pos[0] = source.tcp_position
        # 物理引擎以高頻步進，報告只在輸出幀抽樣接觸，避免 Python 迴圈拖慢批次實驗。
        contact = _contact_snapshot(
            mujoco,
            model,
            data,
            cable_geom_ids,
            environment_geom_ids,
        )
        maximum_contact_count = contact.environment_count
        maximum_contact_force_n = contact.environment_maximum_force_n
        minimum_contact_distance_m = contact.environment_minimum_distance_m
        maximum_self_contact_count = contact.self_count
        maximum_self_contact_force_n = contact.self_maximum_force_n
        minimum_self_contact_distance_m = contact.self_minimum_distance_m

        nodes = _cable_nodes(
            data,
            geom_ids,
            body_ids,
        )
        grasp_error_m = (
            float(np.linalg.norm(data.xpos[grasp_body_id] - source.tcp_position))
            if source.attached
            else 0.0
        )
        result_frames.append(
            _physics_frame(
                source,
                nodes,
                maximum_contact_count,
                maximum_contact_force_n,
                minimum_contact_distance_m,
                maximum_self_contact_count,
                maximum_self_contact_force_n,
                minimum_self_contact_distance_m,
                data.energy.copy(),
                grasp_error_m,
                initial_length_m,
                trajectory,
            )
        )
        previous_source = source
        previous_target = source.tcp_position.copy()

    hose_clearances = np.asarray(
        [frame.hose_clearance_m for frame in result_frames]
    )
    length_ratios = np.asarray(
        [frame.hose_length_ratio for frame in result_frames]
    )
    node_speeds = [0.0]
    for previous, current in itertools.pairwise(result_frames):
        delta_time = current.time_s - previous.time_s
        node_speeds.append(
            float(
                np.max(
                    np.linalg.norm(current.hose_nodes - previous.hose_nodes, axis=1)
                    / delta_time
                )
            )
        )
    metrics = dict(trajectory.metrics)
    metrics.update(
        {
            "minimum_hose_clearance_m": float(hose_clearances.min()),
            "hose_contact_frame_count": int(
                np.count_nonzero(hose_clearances <= 0.001)
            ),
            "hose_penetration_frame_count": int(
                np.count_nonzero(
                    hose_clearances < -trajectory.spec.collision_tolerance_m
                )
            ),
            "maximum_hose_length_error_ratio": float(
                np.max(np.abs(length_ratios - 1.0))
            ),
            "physics_step_count": total_steps,
            "maximum_physics_contact_count": max(
                frame.physics_contact_count for frame in result_frames
            ),
            "maximum_contact_force_n": max(
                frame.maximum_contact_force_n for frame in result_frames
            ),
            "minimum_contact_distance_m": min(
                frame.minimum_contact_distance_m for frame in result_frames
            ),
            "maximum_physics_self_contact_count": max(
                frame.physics_self_contact_count for frame in result_frames
            ),
            "maximum_self_contact_force_n": max(
                frame.maximum_self_contact_force_n for frame in result_frames
            ),
            "minimum_self_contact_distance_m": min(
                frame.minimum_self_contact_distance_m for frame in result_frames
            ),
            "maximum_grasp_constraint_error_m": max(
                frame.grasp_constraint_error_m for frame in result_frames
            ),
            "maximum_hose_speed_m_s": max(node_speeds),
            "maximum_total_energy_j": max(
                frame.potential_energy_j + frame.kinetic_energy_j
                for frame in result_frames
            ),
            "physics_nonfinite_frame_count": int(
                sum(
                    not np.all(np.isfinite(frame.hose_nodes))
                    for frame in result_frames
                )
            ),
        }
    )
    return TrajectoryData(
        spec=trajectory.spec,
        planned_keyframes=trajectory.planned_keyframes,
        frames=tuple(result_frames),
        metrics=metrics,
        physics_engine=f"MuJoCo {mujoco.__version__}",
        solver_name="mujoco_elasticity_cable",
    )


def _case_parameters(spec: MujocoHoseSpec) -> dict[str, float]:
    return {
        "timestep_s": spec.timestep_s,
        "bend_pa": spec.bend_pa,
        "twist_pa": spec.twist_pa,
        "friction": spec.friction,
    }


def _case_metrics(
    trajectory: TrajectoryData,
    baseline: TrajectoryData,
) -> dict[str, float | int]:
    metrics = {
        key: trajectory.metrics[key]
        for key in (
            "maximum_hose_length_error_ratio",
            "maximum_grasp_constraint_error_m",
            "maximum_contact_force_n",
            "minimum_contact_distance_m",
            "maximum_hose_speed_m_s",
            "physics_nonfinite_frame_count",
        )
    }
    delta = trajectory.frames[-1].hose_nodes - baseline.frames[-1].hose_nodes
    metrics["final_shape_rms_delta_m"] = float(
        np.sqrt(np.mean(np.sum(delta * delta, axis=1)))
    )
    return metrics


def simulate_physics_sweep(
    trajectory: TrajectoryData,
    spec: MujocoHoseSpec,
) -> PhysicsSweepData:
    """執行 baseline 與彎曲、摩擦、時間步長敏感度案例。"""

    baseline = simulate_mujoco_hose(trajectory, spec)
    cases = [
        PhysicsSweepCase(
            name="baseline",
            parameters=_case_parameters(spec),
            metrics=_case_metrics(baseline, baseline),
        )
    ]
    for variant in spec.sensitivity_variants:
        variant_spec = spec.with_variant(variant)
        result = simulate_mujoco_hose(trajectory, variant_spec)
        cases.append(
            PhysicsSweepCase(
                name=variant.name,
                parameters=_case_parameters(variant_spec),
                metrics=_case_metrics(result, baseline),
            )
        )
    return PhysicsSweepData(
        baseline=baseline,
        cases=tuple(cases),
        engine_version=baseline.physics_engine or "MuJoCo",
    )
