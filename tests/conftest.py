"""跨測試共用的高成本模擬結果與離線斷言。"""

import re
from pathlib import Path

import pytest

from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.models.perception import PerceptionResult
from simgrasp3d.perception import analyze_rgbd_geometry, load_perception_spec
from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.sensors.rgbd import simulate_rgbd
from simgrasp3d.simulation.hose_motion import (
    load_hose_motion_spec,
    simulate_hose_motion,
)
from simgrasp3d.simulation.mujoco_hose import (
    load_mujoco_hose_spec,
    simulate_mujoco_hose,
)

SCENE_CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")
MOTION_CONFIG_PATH = Path("configs/motions/hose_extraction_demo.json")
PHYSICS_CONFIG_PATH = Path("configs/physics/hose_mujoco_baseline.json")
PERCEPTION_CONFIG_PATH = Path("configs/perception/rgbd_geometry_baseline.json")


@pytest.fixture(scope="session")
def motion_trajectory() -> TrajectoryData:
    """整個測試工作階段只計算一次軟管時間序列。"""

    scene_spec = load_scene_spec(SCENE_CONFIG_PATH)
    motion_spec = load_hose_motion_spec(MOTION_CONFIG_PATH)
    return simulate_hose_motion(motion_spec, scene_spec.robot)


@pytest.fixture(scope="session")
def physics_trajectory(motion_trajectory: TrajectoryData) -> TrajectoryData:
    """整個測試工作階段只執行一次 MuJoCo baseline。"""

    physics_spec = load_mujoco_hose_spec(PHYSICS_CONFIG_PATH)
    return simulate_mujoco_hose(motion_trajectory, physics_spec)


@pytest.fixture(scope="session")
def perception_result() -> PerceptionResult:
    """建立固定 observation 的桌面、物件與抓取幾何結果。"""

    scene_spec = load_scene_spec(SCENE_CONFIG_PATH)
    sensor_result = simulate_rgbd(build_scene(scene_spec))
    table_top_z = scene_spec.table.pose.xyz[2] + scene_spec.table.size[2] / 2.0
    return analyze_rgbd_geometry(
        sensor_result.observation,
        tuple(item.name for item in scene_spec.objects),
        load_perception_spec(PERCEPTION_CONFIG_PATH),
        table_top_z,
    )


def assert_offline_page(page: Path) -> None:
    """頁面只能載入同一輸出樹內、實際存在的本機腳本。"""

    sources = re.findall(r'<script[^>]*\bsrc="([^"]+)"', page.read_text(encoding="utf-8"))
    for source in sources:
        assert "://" not in source, f"不得引用外部資源：{source}"
        assert (page.parent / source).is_file(), f"缺少本機資源：{source}"
