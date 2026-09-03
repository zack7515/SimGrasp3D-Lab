"""醫院案例資料、門檻、匯出與多頁分析介面測試。"""

from pathlib import Path

import numpy as np

from conftest import assert_offline_page
from simgrasp3d.io.hospital import export_hospital_suite
from simgrasp3d.simulation.hospital_cases import (
    load_hospital_suite_spec,
    simulate_hospital_suite,
)
from simgrasp3d.visualization.hospital_dashboard import (
    build_hospital_case_figure,
    write_hospital_dashboard,
)

CONFIG_PATH = Path("configs/hospital/hospital_learning_suite.json")


def test_hospital_suite_runs_in_learning_order_with_finite_outputs() -> None:
    suite = simulate_hospital_suite(load_hospital_suite_spec(CONFIG_PATH))

    assert len(suite.cases) == 7
    assert [case.spec.order for case in suite.cases] == list(range(1, 8))
    assert suite.cases[2].spec.case_id == "bedside_tubing"
    assert suite.cases[-1].spec.risk_level == "very_high"
    for case in suite.cases:
        assert np.all(np.isfinite(case.time_s))
        assert np.all(np.diff(case.time_s) > 0.0)
        assert len(case.events) >= 2
        assert case.assumptions
        assert all(metric.passed is not False for metric in case.metrics)
        for track in case.tracks:
            assert np.all(np.isfinite(track.world_positions))
            assert np.all(np.isfinite(track.observed_positions))


def test_hospital_dashboard_exports_index_case_pages_and_data(tmp_path: Path) -> None:
    suite = simulate_hospital_suite(load_hospital_suite_spec(CONFIG_PATH))
    data_paths = export_hospital_suite(tmp_path, suite)
    page_paths = write_hospital_dashboard(tmp_path, suite)

    assert data_paths["summary"].is_file()
    assert len(page_paths) == 8
    index = page_paths["index"].read_text(encoding="utf-8")
    assert "醫院不是一個場景" in index
    assert "七層風險" in index
    assert "SIMULATION" in index
    assert "case-03-bedside_tubing.html" in index

    case_page = page_paths["bedside_tubing"].read_text(encoding="utf-8")
    assert "病床旁多管路整理" in case_page
    assert "GROUND TRUTH / 原始世界" in case_page
    assert "OBSERVATION + SAFETY" in case_page
    assert "教學安全閘門" in case_page
    assert "NOT FOR CLINICAL USE" in case_page
    assert "案例病歷索引" in case_page
    assert_offline_page(page_paths["bedside_tubing"])


def test_hospital_case_figure_has_synchronized_frames() -> None:
    suite = simulate_hospital_suite(load_hospital_suite_spec(CONFIG_PATH))
    case = suite.cases[2]
    figure = build_hospital_case_figure(case)

    assert len(figure.frames) == len(case.time_s)
    assert figure.frames[0].name == "0000"
    assert figure.frames[-1].name == f"{len(case.time_s) - 1:04d}"
