"""單頁雙畫面模擬驗證報告測試。"""

from pathlib import Path

from simgrasp3d.integration import build_fail_closed_replay, load_integration_spec
from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.models.perception import PerceptionResult
from simgrasp3d.models.physics import PhysicsSweepCase, PhysicsSweepData
from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.sensors.rgbd import simulate_rgbd
from simgrasp3d.visualization.simulation_report import write_simulation_report


CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")
INTEGRATION_CONFIG_PATH = Path("configs/integration/fail_closed_baseline.json")


def test_report_contains_all_views_scenario_and_metrics(
    tmp_path: Path,
    motion_trajectory: TrajectoryData,
) -> None:
    scene = build_scene(load_scene_spec(CONFIG_PATH))
    result = simulate_rgbd(scene)

    path = write_simulation_report(
        scene,
        result,
        tmp_path / "report.html",
        trajectory=motion_trajectory,
    )
    content = path.read_text(encoding="utf-8")

    assert 'id="world-view"' in content
    assert 'id="sensor-view"' in content
    assert 'id="motion-view"' in content
    assert "從世界座標到安全命令" in content
    assert "固定 eye-to-hand / 靜態桌面" in content
    assert "軟管抽取與搬運時間序列" in content
    assert "KINEMATIC LEARNING" in content
    assert 'class="signal-path"' in content
    assert 'class="chapter-nav"' in content
    assert "prefers-reduced-motion" in content
    assert "檢查全部 RGB-D 量測指標" in content
    assert "collision_frame_count" in content
    assert "7.71 mm" in content
    assert "19.84 mm" in content
    assert "<script src=" not in content
    for metric_name in result.metrics:
        assert metric_name in content


def test_report_contains_physics_perception_and_fail_closed_replay(
    tmp_path: Path,
    motion_trajectory: TrajectoryData,
    physics_trajectory: TrajectoryData,
    perception_result: PerceptionResult,
) -> None:
    """完整報告必須呈現 Stage 5～7 的視圖與授權狀態。"""

    scene = build_scene(load_scene_spec(CONFIG_PATH))
    sensor_result = simulate_rgbd(scene)
    case = PhysicsSweepCase(
        name="baseline",
        parameters={"timestep_s": 0.003, "bend_pa": 4.0e6, "friction": 1.0},
        metrics={
            "final_shape_rms_delta_m": 0.0,
            "maximum_contact_force_n": physics_trajectory.metrics[
                "maximum_contact_force_n"
            ],
        },
    )
    sweep = PhysicsSweepData(
        baseline=physics_trajectory,
        cases=(case,),
        engine_version=physics_trajectory.physics_engine or "MuJoCo",
    )
    replay = build_fail_closed_replay(
        physics_trajectory,
        perception_result,
        scene.spec.robot,
        load_integration_spec(INTEGRATION_CONFIG_PATH),
    )

    path = write_simulation_report(
        scene,
        sensor_result,
        tmp_path / "full-report.html",
        trajectory=motion_trajectory,
        physics_sweep=sweep,
        perception=perception_result,
        replay=replay,
    )
    content = path.read_text(encoding="utf-8")

    assert 'id="physics-view"' in content
    assert 'id="perception-view"' in content
    assert "PHYSICS REPLAY" in content
    assert "MuJoCo 軟管接觸物理" in content
    assert "oracle instance baseline" in content
    assert "Fail-closed 規劃與控制重播" in content
    assert "AUTHORIZED" in content
