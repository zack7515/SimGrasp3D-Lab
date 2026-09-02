"""專案主頁、實際執行摘要與跨頁導覽測試。"""

from pathlib import Path

from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.sensors.rgbd import simulate_rgbd
from simgrasp3d.simulation.hose_motion import load_hose_motion_spec
from simgrasp3d.simulation.system_design import (
    load_system_design_spec,
    simulate_system_design_lab,
)
from simgrasp3d.visualization.home_dashboard import write_home_dashboard


SCENE_CONFIG = Path("configs/scenes/tabletop_demo.json")
MOTION_CONFIG = Path("configs/motions/hose_extraction_demo.json")
DESIGN_CONFIG = Path("configs/learning/system_design_lab.json")


def test_home_prioritizes_design_and_uses_actual_run_metrics(
    tmp_path: Path,
    motion_trajectory: TrajectoryData,
) -> None:
    scene = build_scene(load_scene_spec(SCENE_CONFIG))
    sensor = simulate_rgbd(scene)
    design = simulate_system_design_lab(
        load_system_design_spec(DESIGN_CONFIG),
        scene.spec,
        load_hose_motion_spec(MOTION_CONFIG),
    )
    path = write_home_dashboard(
        tmp_path / "index.html",
        scene,
        design=design,
        sensor=sensor,
        trajectory=motion_trajectory,
        design_href="system_design_lab.html",
        report_href="simulation_report.html",
        hospital_href=None,
    )
    content = path.read_text(encoding="utf-8")

    assert "拆成可以驗證的決策" in content
    assert "系統設計實驗室" in content
    assert "6/6 GATES" in content
    assert "7.71 mm" in content
    assert "1.77 mm" in content
    assert 'href="system_design_lab.html"' in content
    assert 'href="simulation_report.html"' in content
    assert "相機、手臂、軟管與規劃路徑系統示意" in content
    assert 'class="system-node"' in content
    assert "prefers-reduced-motion" in content
    assert "<script src=" not in content


def test_home_explains_modules_that_were_not_run(tmp_path: Path) -> None:
    scene = build_scene(load_scene_spec(SCENE_CONFIG))
    path = write_home_dashboard(tmp_path / "partial.html", scene)
    content = path.read_text(encoding="utf-8")

    assert "PARTIAL RUN" in content
    assert "NOT RUN" in content
    assert "unavailable" in content
    assert "simulation-only" in content

