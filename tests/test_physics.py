"""MuJoCo cable baseline 與物理輸出資料測試。"""

import json
from pathlib import Path

import numpy as np

from simgrasp3d.io.physics import export_physics_sweep
from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.models.physics import PhysicsSweepCase, PhysicsSweepData
from simgrasp3d.simulation.mujoco_hose import load_mujoco_hose_spec

PHYSICS_CONFIG_PATH = Path("configs/physics/hose_mujoco_baseline.json")


def test_physics_baseline_is_finite_inextensible_and_attached(
    physics_trajectory: TrajectoryData,
) -> None:
    metrics = physics_trajectory.metrics
    assert physics_trajectory.physics_engine is not None
    assert physics_trajectory.solver_name == "mujoco_elasticity_cable"
    assert metrics["physics_nonfinite_frame_count"] == 0
    assert metrics["maximum_hose_length_error_ratio"] < 1e-6
    assert metrics["maximum_grasp_constraint_error_m"] < 0.015
    assert metrics["minimum_contact_distance_m"] > -0.001
    assert metrics["hose_penetration_frame_count"] <= 1
    assert all(
        np.all(np.isfinite(frame.hose_nodes))
        for frame in physics_trajectory.frames
    )


def test_physics_config_contains_material_and_solver_sensitivity() -> None:
    spec = load_mujoco_hose_spec(PHYSICS_CONFIG_PATH)
    assert spec.bend_pa > 0.0
    assert spec.twist_pa > spec.bend_pa
    assert {variant.name for variant in spec.sensitivity_variants} == {
        "soft_bend",
        "low_friction",
        "coarse_timestep",
    }


def test_physics_export_preserves_engine_and_contact_fields(
    tmp_path: Path,
    physics_trajectory: TrajectoryData,
) -> None:
    case = PhysicsSweepCase(
        name="baseline",
        parameters={"timestep_s": 0.003},
        metrics={"final_shape_rms_delta_m": 0.0},
    )
    sweep = PhysicsSweepData(
        baseline=physics_trajectory,
        cases=(case,),
        engine_version=physics_trajectory.physics_engine or "MuJoCo",
    )
    paths = export_physics_sweep(tmp_path, sweep)
    metadata = json.loads(paths["metrics"].read_text(encoding="utf-8"))
    sensitivity = json.loads(paths["sensitivity"].read_text(encoding="utf-8"))
    assert metadata["physics_engine"].startswith("MuJoCo ")
    assert sensitivity["result_scope"] == "simulation_only"
    with np.load(paths["trajectory"], allow_pickle=False) as data:
        assert data["physics_contact_count"].shape == (116,)
        assert data["potential_energy_j"].shape == (116,)
