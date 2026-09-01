"""單頁雙畫面模擬驗證報告測試。"""

from pathlib import Path

from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.sensors.rgbd import simulate_rgbd
from simgrasp3d.visualization.simulation_report import write_simulation_report


CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")


def test_report_contains_both_views_scenario_and_all_metrics(tmp_path: Path) -> None:
    scene = build_scene(load_scene_spec(CONFIG_PATH))
    result = simulate_rgbd(scene)

    path = write_simulation_report(scene, result, tmp_path / "report.html")
    content = path.read_text(encoding="utf-8")

    assert 'id="world-view"' in content
    assert 'id="sensor-view"' in content
    assert "世界座標 vs. RGB-D 觀測" in content
    assert "固定 eye-to-hand / 靜態桌面" in content
    assert "7.71 mm" in content
    assert "19.84 mm" in content
    assert "<script src=" not in content
    for metric_name in result.metrics:
        assert metric_name in content
