"""手臂、相機、軟管與障礙物系統設計工作台測試。"""

from pathlib import Path

from simgrasp3d.io.system_design import export_system_design_result
from simgrasp3d.scene.builder import load_scene_spec
from simgrasp3d.simulation.hose_motion import load_hose_motion_spec
from simgrasp3d.simulation.system_design import (
    evaluate_system_design,
    load_system_design_spec,
    simulate_system_design_lab,
)
from simgrasp3d.visualization.system_design_lab import write_system_design_lab


DESIGN_CONFIG = Path("configs/learning/system_design_lab.json")
SCENE_CONFIG = Path("configs/scenes/tabletop_demo.json")
MOTION_CONFIG = Path("configs/motions/hose_extraction_demo.json")


def _inputs():
    return (
        load_system_design_spec(DESIGN_CONFIG),
        load_scene_spec(SCENE_CONFIG),
        load_hose_motion_spec(MOTION_CONFIG),
    )


def test_baseline_and_presets_expose_independent_failure_causes() -> None:
    spec, scene, motion = _inputs()
    result = simulate_system_design_lab(spec, scene, motion)

    assert result.baseline.metrics["passed_gate_count"] == 6
    assert result.baseline.metrics["gate_count"] == 6
    assert len(result.baseline.hose_points) == motion.hose.node_count
    assert len(result.baseline.planned_path) >= 8

    failures = {
        preset.name: {gate.key for gate in snapshot.gates if not gate.passed}
        for preset, snapshot in result.preset_snapshots
    }
    assert failures["相機過低"] == {"camera_coverage"}
    assert failures["手臂過短"] == {"reach_reserve"}
    assert failures["夾爪不匹配"] == {"gripper_match"}
    assert "path_clearance" in failures["保守避障"]


def test_parameter_validation_and_causal_response() -> None:
    spec, scene, motion = _inputs()
    noisy = evaluate_system_design(spec, scene, motion, {"depth_noise_scale": 3.0})
    gates = {gate.key: gate for gate in noisy.gates}

    assert gates["depth_uncertainty"].passed is False
    assert gates["camera_coverage"].passed is True
    assert noisy.metrics["depth_uncertainty_m"] > spec.thresholds[
        "maximum_depth_uncertainty_m"
    ]


def test_design_lab_is_self_contained_and_exports_inspectable_data(tmp_path: Path) -> None:
    spec, scene, motion = _inputs()
    result = simulate_system_design_lab(spec, scene, motion)
    html_path = write_system_design_lab(result, scene, motion, tmp_path / "lab.html")
    json_path = export_system_design_result(tmp_path / "result.json", result)
    content = html_path.read_text(encoding="utf-8")
    payload = json_path.read_text(encoding="utf-8")

    assert "先畫出安全包絡" in content
    assert 'id="design-view"' in content
    assert 'id="control-groups"' in content
    assert 'id="gate-grid"' in content
    assert "下載參數 JSON" in content
    assert "匯出實驗 CSV" in content
    assert "FAST GEOMETRY ESTIMATOR" in content
    assert "prefers-reduced-motion" in content
    assert "<script src=" not in content
    assert '"schema": "simgrasp3d.system_design_result.v1"' in payload
    assert '"simulation_only": true' in payload

