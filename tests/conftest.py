"""跨測試共用的高成本模擬結果。"""

from pathlib import Path

import pytest

from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.scene.builder import load_scene_spec
from simgrasp3d.simulation.hose_motion import load_hose_motion_spec, simulate_hose_motion


SCENE_CONFIG_PATH = Path("configs/scenes/tabletop_demo.json")
MOTION_CONFIG_PATH = Path("configs/motions/hose_extraction_demo.json")


@pytest.fixture(scope="session")
def motion_trajectory() -> TrajectoryData:
    """整個測試工作階段只計算一次軟管時間序列。"""

    scene_spec = load_scene_spec(SCENE_CONFIG_PATH)
    motion_spec = load_hose_motion_spec(MOTION_CONFIG_PATH)
    return simulate_hose_motion(motion_spec, scene_spec.robot)
